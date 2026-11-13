# SPDX-License-Identifier: Apache-2.0
"""Per-scenario evidence that the claimed path actually ran, and its deadlines.

check() returns (violations, missing). A violation is a timing rule broken by evidence that
is present (FAIL). Missing evidence means the trace never reached the path it claims to test
(INCONCLUSIVE, never PASS). Field types are validated by rules.judge() before this runs."""
from __future__ import annotations


def _first(events, kind, after=-1, **match):
    for e in events:
        if e["kind"] == kind and e["t_ns"] > after and all(e.get(k) == v for k, v in match.items()):
            return e
    return None


def _late(label, start, stop, deadline):
    if stop - start > deadline:
        return [f"{label}: took {stop - start} ns, deadline {deadline} ns"]
    return []


def check(scenario: str, events: list, deadline: int) -> tuple:
    violations: list[str] = []
    missing: list[str] = []
    if scenario == "unit":
        return violations, missing
    if scenario == "normal":
        if not _first(events, "command_applied"):
            missing.append("normal: no command was applied")
        done = _first(events, "state_transition", event="TRAJECTORY_DONE")
        stop = _first(events, "state_transition", after=done["t_ns"] - 1, event="STOP_CONFIRMED") if done else None
        if not done:
            missing.append("normal: trajectory never completed")
        elif not stop:
            missing.append("normal: stop never confirmed")
        else:
            violations += _late("normal: stop confirmation", done["t_ns"], stop["t_ns"], deadline)
    elif scenario == "manager_stall":
        stall = _first(events, "fault_injected", name="manager_stall")
        if not stall:
            missing.append("manager_stall: injection manager_stall not found")
        stall_t = stall["t_ns"] if stall else 0
        if not _first(events, "command_applied"):
            missing.append("manager_stall: no command applied before the stall")
        if not _first(events, "guard_block", after=stall_t - 1, verdict="PERMIT_EXPIRED"):
            missing.append("manager_stall: no guard_block PERMIT_EXPIRED after the stall")
        permits = [e for e in events if e["kind"] == "permit_issued" and e["t_ns"] <= stall_t]
        stop = _first(events, "drive_local_stop", after=stall_t - 1)
        if not stop:
            missing.append("manager_stall: no drive_local_stop after the stall")
        elif stall and permits:
            violations += _late("manager_stall: drive stop after permit expiry",
                                permits[-1]["expires_ns"], stop["t_ns"], deadline)
    elif scenario == "generation_switch":
        generations = sorted({e["generation"] for e in events if e["kind"] == "permit_issued"})
        if len(generations) < 2:
            missing.append("generation_switch: fewer than two generations issued")
        if not _first(events, "guard_block", verdict="GENERATION_MISMATCH"):
            missing.append("generation_switch: no GENERATION_MISMATCH block")
        if not generations or not _first(events, "command_applied", generation=generations[-1]):
            missing.append("generation_switch: newest generation never applied")
    elif scenario == "fault_reset_reenable":
        fault = _first(events, "fault_injected", name="injected_fault")
        if not fault:
            missing.append("fault_reset_reenable: injected_fault not found")
            return violations, missing
        latch = _first(events, "state_transition", after=fault["t_ns"] - 1, to="FAULT_LATCHED")
        if not latch:
            missing.append("fault_reset_reenable: no FAULT_LATCHED after the fault")
            return violations, missing
        reset = _first(events, "state_transition", after=latch["t_ns"] - 1, to="RECOVERING")
        if not reset:
            missing.append("fault_reset_reenable: no accepted reset after the latch")
            return violations, missing
        enable = _first(events, "enable_request", after=reset["t_ns"] - 1)
        if not enable:
            missing.append("fault_reset_reenable: no enable_request after the accepted reset")
            return violations, missing
        if not _first(events, "command_applied", after=enable["t_ns"] - 1):
            missing.append("fault_reset_reenable: no command applied after the new enable")
        redelivered = _first(events, "command_redelivered", after=enable["t_ns"] - 1)
        if not redelivered or not _first(events, "command_rejected_drive", after=redelivered["t_ns"] - 1,
                                         seq=redelivered["seq"], reason="stale_session"):
            missing.append("fault_reset_reenable: no stale_session rejection of a redelivered command")
    else:
        missing.append(f"no obligations defined for scenario {scenario!r}")
    return violations, missing
