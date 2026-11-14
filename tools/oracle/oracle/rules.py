# SPDX-License-Identifier: Apache-2.0
"""Rules recomputed from raw times and identifiers (spec §19.1).

R0 structure and schema: scenario_begin first, scenario_end last, one of each, one clock
   domain, typed fields. Any problem stops judging: INCONCLUSIVE, because times or ids that
   cannot be trusted cannot prove a violation either.
R1 a command is sent only under the active, unexpired permit (identity and expiry), applied
   only while now < valid_until as SENT, and the apply event repeats the sent identity.
R2 an applied command matches the permit active at apply time, before that permit expires.
R3 every fault (injected, or latched by production) opens an obligation and ends the current
   permit: production must report FAULT_LATCHED within stop_deadline_ns, and motion resumes
   only after an ACCEPTED reset (FAULT_LATCHED -> RECOVERING) followed by an enable_request,
   under a new session and generation. Nothing is applied while it is open.
R4 every outcome belongs to exactly one sent command; a redelivered command may only be
   rejected; every sent command has one outcome.
R5 sessions and generations never go backwards; a revoked session or a superseded or revoked
   generation is never issued again.
R6 per-scenario evidence and deadlines (oracle/obligations.py).
Any violation is FAIL. Missing evidence is INCONCLUSIVE. Only a complete, clean trace is PASS.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from oracle.obligations import check as check_obligations
from oracle.trace import load_trace

PASS, FAIL, INCONCLUSIVE, NOT_RUN = "PASS", "FAIL", "INCONCLUSIVE", "NOT_RUN"

# Contract values the oracle owns. A trace may declare them, but it cannot change them:
# the judged side never supplies its own thresholds. Project-declared (spec §10.1,
# storage analysis/2026-11-13-lease-bound.md): lease <= 48 ms, stop deadline 2W = 4 ms.
MAX_LEASE_NS = 48_000_000
STOP_DEADLINE_NS = 4_000_000
EXIT_CODES = {PASS: 0, FAIL: 1, INCONCLUSIVE: 2, NOT_RUN: 3}
IDENTITY = ("session", "generation", "valid_until_ns")
# Every fault the oracle has rules for. An injected fault outside this set has no obligation,
# so a trace with one cannot PASS: the oracle would not know what it was meant to check.
KNOWN_FAULTS = ("injected_fault", "manager_stall")

INT_FIELDS = {
    "scenario_begin": ("stop_deadline_ns", "lease_ns"),
    "permit_issued": ("manager", "session", "generation", "expires_ns"),
    "permit_revoked": ("manager", "session", "generation"),
    "command_sent": ("seq",) + IDENTITY,
    "command_applied": ("seq",) + IDENTITY,
    "command_rejected_drive": ("seq",),
    "command_redelivered": ("seq",),
    "guard_block": ("seq",),
}
STR_FIELDS = {
    "scenario_begin": ("scenario", "defect", "harness_version"),
    "state_transition": ("from", "to", "event"),
    "fault_injected": ("name",),
    "guard_block": ("verdict",),
    "command_rejected_drive": ("reason",),
}


@dataclass
class Verdict:
    result: str
    reasons: list = field(default_factory=list)


def _schema_problems(events: list) -> list:
    problems = []
    begins = [i for i, e in enumerate(events) if e["kind"] == "scenario_begin"]
    ends = [i for i, e in enumerate(events) if e["kind"] == "scenario_end"]
    if begins != [0]:
        problems.append("scenario_begin must appear exactly once, as the first event")
    if ends != [len(events) - 1]:
        problems.append("scenario_end must appear exactly once, as the last event")
    clocks = {e["clock"] for e in events}
    if len(clocks) != 1:
        problems.append(f"mixed clock domains {sorted(clocks)}")
    for e in events:
        for key in INT_FIELDS.get(e["kind"], ()):
            if type(e.get(key)) is not int:
                problems.append(f"t={e['t_ns']}: {e['kind']}.{key} must be int")
        for key in STR_FIELDS.get(e["kind"], ()):
            if not isinstance(e.get(key), str):
                problems.append(f"t={e['t_ns']}: {e['kind']}.{key} must be str")
    return problems


def judge(events: list, *, allow_unit_scenario: bool = False) -> Verdict:
    problems = _schema_problems(events)
    if not problems:
        begin = events[0]
        if begin["stop_deadline_ns"] != STOP_DEADLINE_NS:
            problems.append(f"declared stop_deadline_ns {begin['stop_deadline_ns']} != contract {STOP_DEADLINE_NS}")
        if not 0 < begin["lease_ns"] <= MAX_LEASE_NS:
            problems.append(f"declared lease_ns {begin['lease_ns']} outside (0, {MAX_LEASE_NS}]")
        if begin["scenario"] == "unit" and not allow_unit_scenario:
            problems.append("scenario 'unit' is reserved for oracle unit tests")
    if problems:
        return Verdict(INCONCLUSIVE, problems)

    fails: list[str] = []
    incomplete: list[str] = []

    active: dict | None = None
    max_session = max_generation = 0
    # Sessions and generations are global, not per manager instance (runtime_core ids.hpp,
    # and the backwards check below already compares across managers): once closed, a value
    # stays closed for every later manager instance, including one after a restart.
    closed_sessions: set = set()
    closed_generations: set = set()
    faults: list[dict] = []
    open_fault: dict | None = None
    sent: dict[int, dict] = {}
    resolved: set[int] = set()
    redelivered: set[int] = set()

    def open_obligation(t: int, latched: bool) -> dict:
        # A fault ends the current execution even if production never reports a revoke:
        # its session and generation may not be used again.
        nonlocal active
        if active is not None:
            closed_sessions.add(active["session"])
            closed_generations.add(active["generation"])
            active = None
        ob = {"t": t, "latched_t": t if latched else None, "reset": False, "recovered": False,
              "requested": False, "enable": False}
        faults.append(ob)
        return ob

    for e in events:
        kind, t = e["kind"], e["t_ns"]
        if kind == "permit_issued":
            m, ses, gen = e["manager"], e["session"], e["generation"]
            if ses in closed_sessions or gen in closed_generations:
                fails.append(f"t={t}: permit reuses a closed session/generation {ses}/{gen}")
            if e["expires_ns"] - t > begin["lease_ns"]:
                fails.append(f"t={t}: permit lasts {e['expires_ns'] - t} ns, longer than the lease {begin['lease_ns']} ns")
            if (active is not None and (m, ses, gen) == (active["manager"], active["session"], active["generation"])
                    and t >= active["expires_ns"]):
                fails.append(f"t={t}: permit renews an expired permit {ses}/{gen}")
            if ses < max_session or gen < max_generation:
                fails.append(f"t={t}: session/generation goes backwards to {ses}/{gen}")
            if active is not None and active["generation"] != gen:
                closed_generations.add(active["generation"])
            max_session, max_generation = max(max_session, ses), max(max_generation, gen)
            active = {"manager": m, "session": ses, "generation": gen, "expires_ns": e["expires_ns"]}
        elif kind == "permit_revoked":
            key = (e["manager"], e["session"], e["generation"])
            closed_generations.add(key[2])
            if active is not None and key == (active["manager"], active["session"], active["generation"]):
                closed_sessions.add(key[1])
                active = None
        elif kind == "fault_injected" and e["name"] not in KNOWN_FAULTS:
            incomplete.append(f"t={t}: unknown fault {e['name']!r} has no obligation")
        elif kind == "fault_injected" and e["name"] == "injected_fault":
            open_fault = open_obligation(t, latched=False)
        elif kind == "state_transition":
            if e["to"] == "FAULT_LATCHED":
                if open_fault is None:
                    open_fault = open_obligation(t, latched=True)
                else:
                    if open_fault["latched_t"] is None:
                        open_fault["latched_t"] = t
                    # A failed recovery re-latches: everything earned since must be earned again.
                    open_fault.update(reset=False, recovered=False, requested=False, enable=False)
            elif e["from"] == "FAULT_LATCHED" and e["to"] == "RECOVERING":
                if open_fault is not None and open_fault["latched_t"] is not None:
                    open_fault["reset"] = True
            elif e["from"] == "RECOVERING" and e["to"] == "READY":
                if open_fault is not None and open_fault["reset"]:
                    open_fault["recovered"] = True
            elif e["from"] == "READY" and e["to"] == "ARMING" and e["event"] == "ENABLE_REQUEST":
                # Only an accepted enable counts, and only after an explicit request that came
                # after recovery finished.
                if open_fault is not None and open_fault["recovered"] and open_fault["requested"]:
                    open_fault["enable"] = True
            elif e["to"] == "ENABLED" and open_fault is not None:
                if not (open_fault["reset"] and open_fault["enable"]):
                    fails.append(f"t={t}: ENABLED after a fault without explicit reset and enable")
                open_fault = None
        elif kind == "enable_request":
            if open_fault is not None and open_fault["recovered"]:
                open_fault["requested"] = True
        elif kind == "command_sent":
            seq = e["seq"]
            if seq in sent:
                incomplete.append(f"t={t}: duplicate command_sent seq {seq}")
                continue
            if active is None:
                fails.append(f"t={t}: seq {seq} sent with no active permit")
            else:
                if t >= active["expires_ns"]:
                    fails.append(f"t={t}: seq {seq} sent after its permit expired")
                if (e["session"], e["generation"]) != (active["session"], active["generation"]):
                    fails.append(f"t={t}: seq {seq} sent as {e['session']}/{e['generation']} "
                                 f"!= permit {active['session']}/{active['generation']}")
                if e["valid_until_ns"] > active["expires_ns"]:
                    fails.append(f"t={t}: seq {seq} outlives its permit at send time")
            sent[seq] = {key: e[key] for key in IDENTITY}
        elif kind == "command_redelivered":
            seq = e["seq"]
            if seq not in resolved:
                fails.append(f"t={t}: redelivery of seq {seq} that was never delivered")
            else:
                redelivered.add(seq)
        elif kind in ("command_applied", "command_rejected_drive"):
            seq = e["seq"]
            if seq not in sent:
                fails.append(f"t={t}: {kind} for seq {seq} that was never sent")
                continue
            if seq in resolved:
                if seq in redelivered:
                    redelivered.discard(seq)
                    if kind == "command_applied":
                        fails.append(f"t={t}: redelivered command applied (seq {seq})")
                else:
                    fails.append(f"t={t}: duplicate outcome for seq {seq}")
                continue
            resolved.add(seq)
            if kind != "command_applied":
                continue
            original = sent[seq]
            if any(e[key] != original[key] for key in IDENTITY):
                fails.append(f"t={t}: seq {seq} apply event differs from sent command")
            if t >= original["valid_until_ns"]:
                fails.append(f"t={t}: seq {seq} applied at or after its sent valid_until (expired)")
            if open_fault is not None:
                fails.append(f"t={t}: seq {seq} applied while latched (fault obligation open)")
            if active is None:
                fails.append(f"t={t}: seq {seq} applied with no active permit")
            elif t >= active["expires_ns"]:
                fails.append(f"t={t}: seq {seq} applied after its permit expired")
            elif (original["session"], original["generation"]) != (active["session"], active["generation"]):
                fails.append(f"t={t}: seq {seq} applied as {original['session']}/{original['generation']} "
                             f"!= permit {active['session']}/{active['generation']}")

    deadline = STOP_DEADLINE_NS
    for ob in faults:
        if ob["latched_t"] is None:
            fails.append(f"fault at t={ob['t']} but production never reported FAULT_LATCHED")
        elif ob["latched_t"] - ob["t"] > deadline:
            fails.append(f"fault at t={ob['t']} latched after {ob['latched_t'] - ob['t']} ns, "
                         f"deadline {deadline} ns")
    if not sent:
        incomplete.append("no command_sent events")
    unresolved = [seq for seq in sent if seq not in resolved]
    if unresolved:
        incomplete.append(f"{len(unresolved)} command_sent without drive outcome")
    if redelivered:
        incomplete.append(f"{len(redelivered)} redelivered command(s) without outcome")
    violations, missing = check_obligations(begin["scenario"], events, deadline)
    fails += violations
    incomplete += missing

    if fails:
        return Verdict(FAIL, fails + incomplete)
    if incomplete:
        return Verdict(INCONCLUSIVE, incomplete)
    return Verdict(PASS, [])


def judge_file(path) -> Verdict:
    loaded = load_trace(path)
    if loaded.status in ("missing", "empty"):
        return Verdict(NOT_RUN, [loaded.error])
    if loaded.status == "invalid":
        return Verdict(INCONCLUSIVE, [loaded.error])
    return judge(loaded.events)
