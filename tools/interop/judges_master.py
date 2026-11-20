# SPDX-License-Identifier: Apache-2.0
"""Verdicts of the master-side fault rows X1-X7 (E3c Task 7), as functions over one observation.

A new module next to judges.py (which is 1300 lines and judges the Lely master's rows); the
observation helpers, the INCONCLUSIVE/FAIL convention and the bank-trace encodings are judges.py's.
An observation is plain JSON, so a recorded one is judged again without vcan (see
test_master_fault_judges.py and tools/evidence/rejudge.py):
  host, device   frames on vcan_host and vcan_device: [t, cob_id, hex data], t = the kernel
                 receive stamp as CLOCK_MONOTONIC seconds (conftest.Capture)
  cli            the JSON lines canopen_master_cli printed (without its pdo and sync lines);
                 "t" is CLOCK_MONOTONIC seconds when written, "at_ns" the worker's instant
  bank           the vdrive_bank trace records (t_ns = CLOCK_MONOTONIC nanoseconds)
  proxy_start, proxy_exit   bus_fault_proxy's start and exit records (the window's epoch)
  harness        rules (the text), epoch_ns, arm_at_ns, cli_rc, scenario, stream_ms

Each judge first asserts its preconditions and raises Inconclusive when one is missing (a verdict
that could not be reached is never a pass); a broken pass condition raises Failure. Every
command-loss row is judged in four phases: the healthy arm (all six armed on the committed tag),
the moving baseline (at least 50 dispatched commands actuated on all six), the fault onset (the
first frame the rule acted on, after the arm completed), and the stop observation.
"""
import judges
from judges import Failure, Inconclusive, bank_not_overloaded, bank_started, check, need

NODES = judges.NODES
cia301 = judges.cia301
MAX_FEEDBACK_AGE_S = 0.030  # canopen_master_cli's max_feedback_age (C20: 20..30 ms).
LEASE_MS = 48
MIN_BASELINE = 50
EDGE_S = 0.002  # A frame this close to a window edge may fall on either side of the proxy's clock.
STOP_REASONS = (judges.QS_TIMEOUT, judges.QS_EXPIRY)
# canopen_master_cli's fault names (hal_contract::Fault) and which of them mean a lost transport,
# node or drive, that is the HAL's ERROR level; a command-loss stop raises none of them.
HARDWARE_FAULTS = {"TRANSPORT_LOST", "NODE_LOST", "BOOT_TIMEOUT", "DRIVE_FAULT", "EVENT_OVERFLOW",
                   "ARM_FAILED"}
JUDGES = {}


# ---- observation access ----

def cli(obs, kind, t0=None, t1=None, **fields):
    return [e for e in obs["cli"] if e.get("event") == kind and
            all(e.get(k) == v for k, v in fields.items()) and
            (t0 is None or e["t"] > t0) and (t1 is None or e["t"] < t1)]


def epoch_offset(obs, t):
    return t - epoch(obs)


def at(e):
    """The worker's instant of a master event, seconds."""
    return e["at_ns"] / 1e9


def epoch(obs):
    return obs["harness"]["epoch_ns"] / 1e9


def step(obs, line):
    """(cmd, cmd_done) of the scenario step `line`; None where it did not happen."""
    cmds = [e for e in cli(obs, "cmd") if e["line"] == line]
    if not cmds:
        return None, None
    done = cli(obs, "cmd_done", n=cmds[0]["n"])
    return cmds[0], (done[0] if done else None)


def bank_records(obs, kind, node, t0=None, t1=None):
    return judges.drive_records(obs, kind, node=node, t0=t0, t1=t1)


def device_syncs(obs):
    return judges.device_syncs(obs)


# ---- the proxy rules, replayed ----

def parse_rules(text):
    """B8 rule text -> [{dir, id, mask, every, offset, from_s, to_s, action, args}]."""
    rules = []
    for line in text.splitlines():
        words = line.split("#", 1)[0].split()
        if not words:
            continue
        rule = {"dir": words[0], "every": 1, "offset": 0, "from_s": None, "to_s": None}
        ident, mask = words[1].split("/")
        rule["id"], rule["mask"] = int(ident, 0), int(mask, 0)
        rest = words[2:]
        if rest[0] == "every":
            rule["every"] = int(rest[1])
            rest = rest[2:]
            if rest[0] == "offset":
                rule["offset"] = int(rest[1])
                rest = rest[2:]
        if rest[0] == "window":
            rule["from_s"], rule["to_s"] = int(rest[1]) / 1000, int(rest[2]) / 1000
            rest = rest[3:]
        rule["action"], rule["args"] = rest[0], rest[1:]
        rules.append(rule)
    return rules


def replay(obs, rules, tol):
    """Per rule, the frames it acted on [(t, cob, data)], by the capture of its source interface
    and its window moved `tol` s inward (tol > 0) or outward (tol < 0): the occurrence predicate
    counts every frame of the identifier (B8), the window only selects among them."""
    t0 = epoch(obs)
    matched = [[] for _ in rules]
    for direction in ("h2d", "d2h"):
        seen = [0] * len(rules)
        for f in judges.source_frames(obs, direction):
            for i, r in enumerate(rules):
                if r["dir"] != direction or f[1] & r["mask"] != r["id"] & r["mask"]:
                    continue
                n = seen[i]
                seen[i] += 1
                if n % r["every"] != r["offset"]:
                    continue
                since = f[0] - t0
                if r["from_s"] is not None and not r["from_s"] + tol <= since < r["to_s"] - tol:
                    continue
                matched[i].append(f)
                break
    return matched


def fault_rules(obs):
    """(rules, the frames each acted on): the proxy's own match counts must lie between what the
    captures select with the window shrunk and grown by EDGE_S (the two clocks differ by a few
    hundred microseconds), every rule must have matched something, and the proxy must have
    sent everything."""
    rules = parse_rules(obs["harness"]["rules"])
    need(rules and all(r["from_s"] is not None for r in rules), "a rule without a window")
    inner, outer = replay(obs, rules, EDGE_S), replay(obs, rules, -EDGE_S)
    got = [r["matched"] for r in obs["proxy_exit"]["rules"]]
    need(len(got) == len(rules) and all(len(a) <= g <= len(b)
                                        for a, g, b in zip(inner, got, outer)),
         f"proxy rule match counts {got}, the captures of the source interfaces select "
         f"{[len(a) for a in inner]}..{[len(b) for b in outer]}")
    need(all(got), f"the window matched nothing: proxy match counts {got}")
    need(obs["proxy_exit"].get("send_failed") == {"h2d": 0, "d2h": 0},
         f"the proxy failed to send: {obs['proxy_exit'].get('send_failed')}")
    # The frames the proxy acted on, inside both bounds: certain.
    return rules, inner


# ---- preconditions shared by every row ----

def complete(obs, scenario):
    """Traces complete, processes ran, the scenario ended the way the row declares; Inconclusive
    otherwise. A stream row's CLI exits 0 with scenario_done; the arming row's exits 1."""
    bank = obs.get("bank") or []
    need(bank, "no vdrive_bank trace records")
    bank_started(bank)
    bank_not_overloaded(bank)
    overflow = [r for r in bank if r.get("trace_overflow", 0) != 0]
    need(not overflow, f"vdrive_bank trace incomplete: trace_overflow = "
                       f"{overflow[0].get('trace_overflow') if overflow else 0}")
    need(bank[-1].get("event") == "exit", f"vdrive_bank trace has no final exit record: "
                                          f"{bank[-1:]}")
    proxy = obs.get("proxy_exit") or {}
    need(proxy.get("event") == "exit" and proxy.get("trace_overflow") == 0,
         f"bus_fault_proxy trace incomplete: {proxy}")
    need(obs["host"] and obs["device"], "an empty capture")
    need(cli(obs, "start"), "canopen_master_cli printed no start line")
    exits = cli(obs, "exit")
    need(len(exits) == 1, f"canopen_master_cli wrote no single exit record: {exits}")
    need(exits[0]["event_overflows"] == 0 and exits[0]["tx_eagain"] == 0 and
         exits[0]["tx_enobufs"] == 0, f"the master's worker lost events or frames: {exits[0]}")
    end = cli(obs, "scenario_done") + cli(obs, "scenario_failed")
    need(len(end) == 1, f"no single end of the scenario: {end}")
    want = "scenario_failed" if scenario == "arm" else "scenario_done"
    need(end[0]["event"] == want and obs["harness"]["cli_rc"] == (1 if scenario == "arm" else 0),
         f"the scenario ended {end[0]}, exit status {obs['harness']['cli_rc']}")
    need(obs["harness"]["scenario"] == scenario, f"the row ran scenario "
                                                  f"{obs['harness']['scenario']}")
    return end[0]


def brought_up(obs):
    """configure and activate completed. Returns the arm's (cmd, cmd_done or None)."""
    for line in ("configure", "activate"):
        _, done = step(obs, line)
        need(done is not None and done["ok"], f"step {line} did not complete: {done}")
    arm, done = step(obs, "arm")
    need(arm is not None, "the arm was never started")
    return arm, done


def committed_tags(obs, t_arm):
    """{node: tag} of the last non-zero 0x2012 upload of each node (the arm's commit read-back)."""
    tags = {}
    for e in cli(obs, "sdo_ok", t0=t_arm, index=0x2012):
        if e["value"]:
            tags[e["node"]] = e["value"]
    return tags


def healthy_arm(obs, t_arm_cmd, t_armed):
    """The arm completed on all six: each node's committed tag (0x2012) is the applied_tag its
    TPDO3 reported on vcan_host by the time the CLI saw the arm complete (the C17 'armed' rule)."""
    tags = committed_tags(obs, t_arm_cmd)
    need(sorted(tags) == list(NODES), f"committed tags of nodes {sorted(tags)}")
    for n in NODES:
        applied = [decode_tpdo3(d) for _, _, d in judges.frames(
            obs, "host", cia301.TPDO_BASE[3] + n, None, t_armed + EDGE_S) if len(d) == 8]
        need(tags[n] in applied, f"node {n}: no TPDO3 applied_tag {tags[n]} before the arm "
                                 f"completed")
    return tags


def decode_tpdo3(data):
    return int.from_bytes(data[6:8], "little")


def baseline(obs, tags, t_onset):
    """At least MIN_BASELINE dispatched approved commands before the onset, each actuated on all
    six drives under its committed tag; returns the dispatched seqs (all of them, in order)."""
    dispatched = [e for e in cli(obs, "dispatch") if e["approved"]]
    before = [e["seq"] for e in dispatched if at(e) < t_onset - 0.020]
    actuated = {n: {r["seq"] for r in bank_records(obs, "actuated", n) if r["tag"] == tags[n]}
                for n in NODES}
    missing = [(s, n) for s in before for n in NODES if s not in actuated[n]]
    need(len(before) >= MIN_BASELINE and not missing,
         f"moving baseline: {len(before)} commands dispatched before the onset, not actuated on "
         f"all six: {missing[:6]}")
    return [e["seq"] for e in dispatched]


def four_phases(obs, matched):
    """The shared preconditions of the command-loss rows. Returns (tags, t_onset, t_armed)."""
    complete(obs, "stream")
    arm, done = brought_up(obs)
    need(done is not None and done["ok"], f"the healthy arm did not complete: {done}")
    t_armed = done["t"]
    t_onset = min(f[0] for m in matched for f in m)
    need(t_onset > t_armed, f"the first matched frame ({t_onset - epoch(obs):.4f} s after the "
                            f"proxy's start) precedes the arm's completion "
                            f"({t_armed - epoch(obs):.4f} s)")
    tags = healthy_arm(obs, arm["t"], t_armed)
    baseline(obs, tags, t_onset)
    early = [e for e in obs["cli"] if e.get("event") in ("stop", "hb_lost", "emcy", "boot_timeout")
             and e["t"] < t_onset]
    faults = [e for e in cli(obs, "faults", t1=t_onset) if e["bits"]]
    need(not early and not faults, f"a stop, fault or event before the onset: "
                                   f"{(early + faults)[:3]}")
    healthy_writes(obs, t_onset)
    return tags, t_onset, t_armed


def healthy_writes(obs, t_onset):
    """Every Hal::write before the onset returned kSend. The CLI prints a `verdict` line at the
    first write and at every change, so a refused write always leaves a non-SEND line; one is a
    defect of the host (FAIL), not a missing precondition."""
    lines = cli(obs, "verdict", t1=t_onset)
    need(lines, "the stream wrote nothing before the onset")
    bad = [v for v in lines if v["verdict"] != "SEND"]
    check(not bad, f"Hal::write returned {bad[0]['verdict']} at seq {bad[0]['seq']} "
                   f"{epoch_offset(obs, bad[0]['t']):.4f} s after the proxy's start, before the "
                   f"onset (feedback age {bad[0]['age_ns'] / 1e6:.1f} ms)" if bad else "")


# ---- the stop observation ----

def first_stop(obs, t_onset):
    """The CLI's first group stop (C18 STOPPING) at or after the onset: the inhibit became visible
    to the HAL. Returns its time."""
    stops = [e for e in cli(obs, "stop", t0=t_onset) if "STOPPING" in e["stop"]]
    check(stops, "the master never reported a stop in progress after the onset")
    return stops[0]["t"]


def inhibit_latched(obs, t_onset):
    """The first `faults` line after the onset with kInhibited; later faults lines keep it."""
    lines = cli(obs, "faults", t0=t_onset)
    found = [e for e in lines if "INHIBITED" in e["faults"]]
    check(found, f"kInhibited never raised after the onset: {lines}")
    cleared = [e for e in lines if e["t"] > found[0]["t"] and "INHIBITED" not in e["faults"]]
    check(not cleared, f"the inhibit was cleared at {cleared[0]['t'] - epoch(obs):.3f} s"
          if cleared else "")
    verdicts = cli(obs, "verdict", t0=found[0]["t"] - 0.0005)
    check(verdicts and verdicts[0]["verdict"] == "INHIBITED" and
          all(v["verdict"] == "INHIBITED" for v in verdicts),
          f"Hal::write after the inhibit: {[v['verdict'] for v in verdicts]}")
    return found[0]["t"]


def hardware_error_free(obs):
    names = {n for e in cli(obs, "faults") for n in e["faults"]}
    check(not names & HARDWARE_FAULTS, f"hardware-level faults raised: {sorted(names)}")
    return sorted(names)


def group_confirmed(obs):
    """The last `stop` line reports all six CONFIRMED."""
    stops = cli(obs, "stop")
    check(stops and stops[-1]["stop"] == ["CONFIRMED"] * 6,
          f"final stop status {stops[-1]['stop'] if stops else None}, not all six confirmed")
    return stops[-1]["t"]


def stop_times(obs, node, t_onset):
    """The instant the drive left motion: its own policy stop (bank quick_stop) or the master's
    Quick Stop controlword (0x02) as it appeared on vcan_device, the earlier one; None if neither."""
    own = [r["t_ns"] / 1e9 for r in bank_records(obs, "quick_stop", node, t0=t_onset)]
    cw = [t for t, _, d in judges.frames(obs, "device", cia301.RPDO_BASE[3] + node, t_onset)
          if d[:2] == b"\x02\x00"]
    found = own[:1] + cw[:1]
    return min(found) if found else None


def no_motion_after_stop(obs, t_onset, nodes=NODES):
    """No drive actuated after it stopped (its own policy stop or the master's Quick Stop)."""
    out = {}
    for n in nodes:
        t_stop = stop_times(obs, n, t_onset)
        check(t_stop is not None, f"node {n} never stopped after the onset")
        late = bank_records(obs, "actuated", n, t0=t_stop + 0.001)
        check(not late, f"node {n} actuated seq {late[0]['seq']} {late[0]['t_ns'] / 1e9 - t_stop:.4f}"
                        f" s after it stopped" if late else "")
        out[n] = t_stop
    return out


def no_dispatch_after(obs, t_inhibit):
    late = [e for e in cli(obs, "dispatch") if e["approved"] and at(e) > t_inhibit]
    check(not late, f"{len(late)} command(s) dispatched after the inhibit, the first seq "
                    f"{late[0]['seq']}" if late else "")


# ---- X1, X2: feedback outages ----

def outage(obs, node=3):
    """The rules of the two outage rows: node 3's TPDO1-TPDO3 dropped d2h in one window.
    Returns (rules, matched, window_s)."""
    rules, matched = fault_rules(obs)
    cobs = {cia301.TPDO_BASE[n] + node for n in (1, 2, 3)}
    need(len(rules) == 3 and {r["id"] for r in rules} == cobs and
         all(r["dir"] == "d2h" and r["action"] == "drop" and r["mask"] == 0x7FF and
             r["every"] == 1 for r in rules), "the rules are not TPDO1-TPDO3 of node 3 dropped")
    need(len({(r["from_s"], r["to_s"]) for r in rules}) == 1, "the rules have different windows")
    return rules, matched, rules[0]["to_s"] - rules[0]["from_s"]


def feedback_events(obs, t_onset):
    """(the first stale line after the onset, the first fresh line after it)."""
    lines = cli(obs, "feedback", t0=t_onset)
    stale = [e for e in lines if not e["fresh"]]
    check(stale, "the published feedback never went stale after the onset")
    fresh = [e for e in lines if e["fresh"] and e["t"] > stale[0]["t"]]
    need(fresh, "the feedback never returned after the outage")
    return stale[0], fresh[0]


def only_feedback_caused_it(obs, t_onset, t_inhibit):
    """Nothing but the feedback age can have latched the inhibit: no state change, EMCY, boot
    timeout or heartbeat loss reached the master between the onset and the latch."""
    other = [e for e in obs["cli"] if e.get("event") in ("state", "emcy", "boot", "boot_timeout",
                                                          "hb_lost") and t_onset < e["t"] <= t_inhibit]
    check(not other, f"events besides the feedback age before the inhibit: {other[:3]}")


def judge_outage(obs, row):
    rules, matched, width = outage(obs)
    need(0.030 < width < LEASE_MS / 1000 if row == "X1" else width > LEASE_MS / 1000,
         f"{row}: the outage is {width * 1000:.0f} ms")
    tags, t_onset, t_armed = four_phases(obs, matched)
    stale, fresh = feedback_events(obs, t_onset)
    check(stale["age_ns"] > MAX_FEEDBACK_AGE_S * 1e9,
          f"the feedback went stale at an age of {stale['age_ns'] / 1e6:.1f} ms, not above 30 ms")
    t_inhibit = inhibit_latched(obs, t_onset)
    # C17 latches when the age passes max_feedback_age: at most max_feedback_age + T_bus after
    # the onset (the last sample may be a bus period older than it), plus the CLI's two 2 ms ticks.
    bound = MAX_FEEDBACK_AGE_S + 0.010 + 0.004
    check(stale["t"] <= t_inhibit + 1e-6 and t_inhibit - t_onset <= bound,
          f"the inhibit at {t_inhibit - t_onset:.3f} s after the onset (bound {bound:.3f} s), "
          f"the stale feedback at {stale['t'] - t_onset:.3f} s")
    only_feedback_caused_it(obs, t_onset, t_inhibit)
    t_stopping = first_stop(obs, t_onset)
    no_dispatch_after(obs, t_stopping)
    check(not any(e["approved"] and at(e) > fresh["t"] for e in cli(obs, "dispatch")),
          "a command was dispatched after the feedback returned")
    stops = no_motion_after_stop(obs, t_onset)
    t_confirmed = group_confirmed(obs)
    faults = hardware_error_free(obs)
    notes = [f"{len(matched[0]) + len(matched[1]) + len(matched[2])} TPDO1-TPDO3 of node 3 dropped "
             f"for {width * 1000:.0f} ms from {t_onset - epoch(obs):.3f} s (arm complete "
             f"{t_armed - epoch(obs):.3f} s); feedback stale {(stale['t'] - t_onset) * 1000:.0f} ms "
             f"after the onset at age {stale['age_ns'] / 1e6:.1f} ms; inhibit latched "
             f"{(t_inhibit - t_onset) * 1000:.0f} ms after the onset by the feedback age alone; "
             f"group stop confirmed on all six {(t_confirmed - t_onset) * 1000:.0f} ms after it; "
             f"no command dispatched after the latch, none actuated after a drive's stop"]
    return notes, (obs, t_onset, t_inhibit, fresh, stale, stops, faults)


def judge_x1(obs):
    """35 ms of node 3's TPDOs lost (longer than max_feedback_age, shorter than the lease): the
    inhibit latches on the feedback age, the group stops, nothing is dispatched after the latch
    nor after the feedback returned."""
    notes, _ = judge_outage(obs, "X1")
    return notes


def judge_x2(obs):
    """100 ms of node 3's TPDOs lost: the published feedback age exceeds max_feedback_age and C17
    latches the inhibit on it within max_feedback_age + T_bus of the onset (from then on writes
    return kInhibited, so no separate kFeedbackStale from age is observable); the observation
    recovers (fresh samples,
    no hardware-level fault) while the motion stays inhibited: the inhibit is never cleared,
    nothing is dispatched, no drive actuates, node 3 stays in Quick Stop Active."""
    notes, (obs, t_onset, t_inhibit, fresh, stale, stops, faults) = judge_outage(obs, "X2")
    window_end = parse_rules(obs["harness"]["rules"])[0]["to_s"] + epoch(obs)
    check(fresh["t"] > window_end - EDGE_S and fresh["age_ns"] <= MAX_FEEDBACK_AGE_S * 1e9,
          f"the feedback returned at {fresh['t'] - epoch(obs):.3f} s, the window ended "
          f"{window_end - epoch(obs):.3f} s")
    resumed = judges.frames(obs, "host", cia301.TPDO_BASE[1] + judges.NODE, window_end + EDGE_S)
    check(len(resumed) >= 5, f"only {len(resumed)} TPDO1 of node 3 reached vcan_host after the "
                             f"outage")
    after = [judges.decode("TPDO1", d)["statusword"] & 0x6F for _, _, d in resumed if len(d) == 8]
    check(after and all(s == judges.QUICK_STOP_ACTIVE for s in after),
          f"node 3 left Quick Stop Active after the outage: statuswords "
          f"{sorted({hex(s) for s in after})}")
    check(not any(r["t_ns"] / 1e9 > t_inhibit for r in bank_records(obs, "actuated", 3)),
          "node 3 actuated after the inhibit")
    verdicts = cli(obs, "stream_end")[0]["verdicts"]
    return notes + [f"writes by verdict over the run {verdicts}; the observation recovered {(fresh['t'] - window_end) * 1000:.0f} ms "
                    f"after the window ({len(resumed)} TPDO1 of node 3 since, all Quick Stop "
                    f"Active; faults ever raised: {faults}); the inhibit stayed latched to the "
                    f"end, no command and no kActuated followed"]


# ---- X3: one node loses its commands ----

def judge_x3(obs):
    """Every RPDO2 of node 3 lost: node 3 stops on its own (policy stop, missing halves), the
    master sees it and inhibits the group, all six stop and are confirmed, nothing actuates
    after."""
    rules, matched = fault_rules(obs)
    need(len(rules) == 1 and rules[0]["dir"] == "h2d" and rules[0]["action"] == "drop" and
         rules[0]["id"] == cia301.RPDO_BASE[2] + 3 and rules[0]["every"] == 1,
         "the rule is not 'drop every RPDO2 of node 3'")
    tags, t_onset, t_armed = four_phases(obs, matched)
    # A seq repeated in a later slot may have reached the drive in an earlier copy: only the seqs
    # no copy of which reached vcan_device are lost.
    delivered = {v["command_seq"] for _, _, d in judges.frames(
        obs, "device", cia301.RPDO_BASE[2] + 3) if len(d) == 8
        for v in [judges.decode("RPDO2", d)]}
    lost = sorted({judges.decode("RPDO2", d)["command_seq"] for _, _, d in matched[0]} - delivered)
    need(lost, "every dropped RPDO2 was repeated through to the drive")
    first_dropped = lost[0]
    t_own = [r for r in bank_records(obs, "quick_stop", 3, t0=t_onset)]
    check(t_own and t_own[0]["reason"] in STOP_REASONS,
          f"node 3's first stop {[judges.QS_NAMES[r['reason']] for r in t_own]}, expected "
          f"timeout or expiry")
    t_node3 = t_own[0]["t_ns"] / 1e9
    check(t_node3 - t_onset < 0.100, f"node 3 stopped {t_node3 - t_onset:.3f} s after the onset")
    act = [r["seq"] for r in bank_records(obs, "actuated", 3, t0=t_onset)]
    check(all(s < first_dropped for s in act),
          f"node 3 actuated seq(s) {[s for s in act if s >= first_dropped]} from "
          f"{first_dropped} on, whose RPDO2 was dropped")
    halves = sum(r["count"] for r in bank_records(obs, "dropped", 3, t0=t_onset)
                 if r["drop"] == judges.DROP_MISSING_HALF)
    check(halves >= 1, "node 3 counted no missing half for the RPDO1 whose RPDO2 was lost")
    t_inhibit = inhibit_latched(obs, t_onset)
    check(t_inhibit > t_node3 - 0.001 and t_inhibit - t_node3 < 0.100,
          f"the inhibit at {t_inhibit - t_node3:.3f} s after node 3's own stop")
    t_stopping = first_stop(obs, t_onset)
    no_dispatch_after(obs, t_stopping)
    no_motion_after_stop(obs, t_onset)
    t_confirmed = group_confirmed(obs)
    return [f"{len(matched[0])} RPDO2 of node 3 dropped from {t_onset - epoch(obs):.3f} s (first "
            f"seq {first_dropped}); node 3 stopped on its own "
            f"{judges.QS_NAMES[t_own[0]['reason']]} {(t_node3 - t_onset) * 1000:.0f} ms later "
            f"({halves} missing-half drops, last actuated seq {max(act)}); the master inhibited "
            f"{(t_inhibit - t_node3) * 1000:.0f} ms after that and all six were confirmed "
            f"stopped {(t_confirmed - t_node3) * 1000:.0f} ms after it; no command dispatched "
            f"after the inhibit, no drive actuated after its stop"]


# ---- X4: heartbeat loss ----

def judge_x4(obs):
    """Every heartbeat of node 4 lost: the heartbeat consumer reports hb_lost 250-400 ms after the
    last heartbeat that reached the master, raises the node-lost fault, inhibits the group, and
    sends no NMT frame (no reset, no stop) in reaction."""
    rules, matched = fault_rules(obs)
    hb = cia301.HEARTBEAT_BASE + 4
    need(len(rules) == 1 and rules[0]["dir"] == "d2h" and rules[0]["action"] == "drop" and
         rules[0]["id"] == hb and rules[0]["every"] == 1, "the rule is not 'drop node 4 heartbeats'")
    tags, t_onset, t_armed = four_phases(obs, matched)
    delivered = [t for t, _, _ in judges.frames(obs, "host", hb)]
    need(delivered and delivered[-1] < t_onset + 0.2, "no heartbeat of node 4 reached vcan_host")
    t_last = delivered[-1]
    lost = cli(obs, "hb_lost")
    check([e["node"] for e in lost] == [4], f"hb_lost events for nodes {[e['node'] for e in lost]}")
    lag = at(lost[0]) - t_last
    check(0.250 - 0.001 <= lag <= 0.400,
          f"hb_lost {lag * 1000:.1f} ms after the last heartbeat that reached the master")
    check(not cli(obs, "hb_resumed") and not cli(obs, "boot_timeout"),
          "an unexpected heartbeat event")
    node_lost = [e for e in cli(obs, "faults", t0=t_onset) if "NODE_LOST" in e["faults"]]
    check(node_lost and node_lost[0]["t"] >= lost[0]["t"] - 0.0005,
          f"kNodeLost not raised with the heartbeat loss: {node_lost[:1]}")
    nmt = judges.frames(obs, "host", cia301.NMT_COB, t_armed)
    check(not nmt, f"NMT frame(s) from the master after the arm: {judges.hexes(nmt[:3])}")
    t_inhibit = inhibit_latched(obs, t_onset)
    check(0 <= t_inhibit - lost[0]["t"] < 0.050,
          f"the inhibit {(t_inhibit - lost[0]['t']) * 1000:.1f} ms after hb_lost")
    t_stopping = first_stop(obs, t_onset)
    no_dispatch_after(obs, t_stopping)
    no_motion_after_stop(obs, t_onset)
    t_confirmed = group_confirmed(obs)
    return [f"{len(matched[0])} heartbeats of node 4 dropped from {t_onset - epoch(obs):.3f} s; "
            f"hb_lost {lag * 1000:.0f} ms after the last one that arrived (consumer time 250 ms); "
            f"kNodeLost raised, inhibit {(t_inhibit - lost[0]['t']) * 1000:.0f} ms later, all six "
            f"confirmed stopped {(t_confirmed - lost[0]['t']) * 1000:.0f} ms after hb_lost; no "
            f"NMT frame from the master after the arm; no command dispatched after the inhibit"]


# ---- X5: an SDO answer lost during the arm ----

def judge_x5(obs):
    """Node 5's SDO answers lost from before the arm: the arm's commit times out on node 5,
    arming reports failure, and Enable Operation (0x0F) is never sent to any node."""
    rules, matched = fault_rules(obs)
    need(len(rules) == 1 and rules[0]["dir"] == "d2h" and rules[0]["action"] == "drop" and
         rules[0]["id"] == cia301.SDO_TX_BASE + 5 and rules[0]["every"] == 1,
         "the rule is not 'drop node 5's SDO answers'")
    complete(obs, "arm")
    arm, done = brought_up(obs)
    t_onset = matched[0][0][0]
    # The CLI prints the arm's line just after posting the op, so its time may trail the first
    # request by microseconds; the answers of configure and activate are a second earlier.
    need(t_onset > arm["t"] - EDGE_S, f"the first dropped SDO answer ({t_onset - epoch(obs):.4f} s) "
                             f"precedes the start of the arm ({arm['t'] - epoch(obs):.4f} s): "
                             f"configure or activate lost an answer")
    cws = [f for f in judges.frames(obs, "host") if cia301.RPDO_BASE[3] < f[1] <=
           cia301.RPDO_BASE[3] + 6]
    need(cws, "the master sent no controlword at all: activation was not attempted")
    check(done is not None and not done["ok"], f"the arm completed: {done}")
    check(done["error"], "the failed arm carries no reason")
    timeouts = cli(obs, "sdo_timeout", t0=arm["t"], node=5)
    check(timeouts, "no SDO timeout for node 5 during the arm")
    check(any(e["node"] == 5 for e in cli(obs, "sdo_timeout")) and
          all(e["node"] == 5 for e in cli(obs, "sdo_timeout")),
          f"SDO timeouts of nodes {sorted({e['node'] for e in cli(obs, 'sdo_timeout')})}")
    enables = [f for f in cws if f[2][:2] == b"\x0f\x00"]
    check(not enables, f"Enable Operation sent: {judges.hexes(enables[:3])}")
    check(not [e for e in cli(obs, "dispatch") if e["approved"]], "a command was dispatched")
    check(not [r for r in obs["bank"] if r.get("event") == "drive" and r["kind"] == "actuated"],
          "a drive actuated")
    flags = [e for e in cli(obs, "faults") if "ARM_FAILED" in e["faults"]]
    check(flags, "kArmFailed never raised")
    return [f"{len(matched[0])} SDO answers of node 5 dropped from {t_onset - epoch(obs):.3f} s "
            f"(arm started {arm['t'] - epoch(obs):.3f} s); SDO timeout of node 5 "
            f"{(timeouts[0]['t'] - arm['t']) * 1000:.0f} ms into the arm, arm failed "
            f"({done['error']!r}) with kArmFailed; {len(cws)} controlwords sent (none 0x0F), no "
            f"command dispatched, no drive actuated"]


# ---- X6, X7: delayed RPDO2 ----

def rpdo_arrivals(obs, node, tag, t0=None, t1=None):
    """[(t, kind 1|2, seq, data)] of node's RPDO1/RPDO2 of `tag` on vcan_device."""
    out = []
    for n in (1, 2):
        for t, _, d in judges.frames(obs, "device", cia301.RPDO_BASE[n] + node, t0, t1):
            if len(d) == 8:
                v = judges.decode(f"RPDO{n}", d)
                if v["exec_tag"] == tag:
                    out.append((t, n, v["command_seq"], d))
    return sorted(out)


def judge_x6(obs):
    """Every RPDO2 of every node held 60 ms (past the 48 ms lease): the drives stop on their own
    (command timeout or expiry) before the first late frame arrives, which revokes the tag; every
    late frame is then one other-tag drop, never a not-valid one; the master sees the drives'
    stop, inhibits the group, and all six are confirmed stopped."""
    rules, matched = fault_rules(obs)
    need(len(rules) == 1 and rules[0]["dir"] == "h2d" and rules[0]["action"] == "delay" and
         rules[0]["id"] == cia301.RPDO_BASE[2] and rules[0]["mask"] == 0x7F8 and
         rules[0]["every"] == 1 and int(rules[0]["args"][0]) > LEASE_MS,
         "the rule is not 'delay every RPDO2 past the lease'")
    tags, t_onset, t_armed = four_phases(obs, matched)
    t_inhibit = inhibit_latched(obs, t_onset)
    notes = []
    for n in NODES:
        own = bank_records(obs, "quick_stop", n, t0=t_onset)
        check(own and own[0]["reason"] in STOP_REASONS,
              f"node {n}: first stop {[judges.QS_NAMES[r['reason']] for r in own]}, expected "
              f"timeout or expiry")
        t_stop = own[0]["t_ns"] / 1e9
        check(t_stop < t_inhibit + 0.0005,
              f"node {n} stopped {t_stop - t_inhibit:.3f} s after the master's inhibit: the "
              f"drive's own stop must come first")
        arrivals = rpdo_arrivals(obs, n, tags[n], t_onset - 0.1)
        live = [a for a in arrivals if a[0] < t_stop and a[1] == 2 and a[0] > t_onset]
        not_valid = sum(r["count"] for r in bank_records(obs, "dropped", n)
                        if r["drop"] == judges.DROP_NOT_VALID)
        check(not_valid == len(live), f"node {n}: {len(live)} delayed RPDO2 arrived before its "
                                      f"stop, {not_valid} not-valid drops")
        covered = other_tag_after_stop(obs, n, arrivals, t_stop)
        need(covered >= 1, f"node {n}: no late frame arrived in a judged SYNC interval")
        notes.append(covered)
    t_stopping = first_stop(obs, t_onset)
    no_dispatch_after(obs, t_stopping)
    no_motion_after_stop(obs, t_onset)
    group_confirmed(obs)
    return [f"{len(matched[0])} RPDO2 of all six held 60 ms from {t_onset - epoch(obs):.3f} s; "
            f"every drive stopped on its own (command timeout or expiry) before the first late "
            f"frame and before the master's inhibit ({(t_inhibit - t_onset) * 1000:.0f} ms after "
            f"the onset); late frames judged per node in SYNC intervals after the stop, each one "
            f"other-tag drop at its closing SYNC ({notes} frames per node, none not-valid); all "
            f"six confirmed stopped; no command dispatched after the inhibit, no actuation "
            f"after a stop"]


def other_tag_after_stop(obs, node, arrivals, t_stop):
    """For the SYNC intervals opening 5 ms or more after t_stop: the node's other-tag drops at the
    closing SYNC equal the RPDO1/RPDO2 frames of its tag arriving in the interval (D8: the tag is
    classified first). Returns the frames judged."""
    syncs = device_syncs(obs)
    judged = [i for i in range(len(syncs) - 1) if syncs[i] >= t_stop + 0.005]
    need(len(judged) >= 10, f"only {len(judged)} SYNC intervals after the stop")
    drops = judges.drops_by_sync(obs, judges.DROP_OTHER_TAG, syncs, node=node)
    total = 0
    for i in judged:
        n = len([a for a in arrivals if syncs[i] < a[0] < syncs[i + 1]])
        check(drops.get(i + 1, 0) == n, f"node {node}, SYNC interval {i}: {n} RPDO1/RPDO2, "
                                        f"{drops.get(i + 1, 0)} other-tag drops at its SYNC")
        total += n
    return total


def judge_x7(obs):
    """Every fourth RPDO2 of node 3 held 60 ms between 2.3 and 3.0 s (as E3b F7): the fault window
    ends and healthy approvals continue until every delayed frame has arrived; each delayed frame
    that arrives while node 3's tag is current is exactly one not-valid drop, trailing arrivals
    after node 3's stop are other-tag (named, the F7 tail rule), at least 10 live arrivals are
    required, and no actuation carries an expired deadline."""
    rules, matched = fault_rules(obs)
    need(len(rules) == 1 and rules[0]["dir"] == "h2d" and rules[0]["action"] == "delay" and
         rules[0]["id"] == cia301.RPDO_BASE[2] + 3 and rules[0]["every"] > 1 and
         int(rules[0]["args"][0]) > LEASE_MS, "the rule is not a minority delay of node 3's RPDO2")
    tags, t_onset, t_armed = four_phases(obs, matched)
    tag = tags[3]
    delay_s = int(rules[0]["args"][0]) / 1000
    delayed = [(t, d) for t, _, d in matched[0] if judges.decode("RPDO2", d)["exec_tag"] == tag]
    device = rpdo_arrivals(obs, 3, tag)
    held = []  # (arrival time, seq) of each delayed frame, matched by its bytes one delay later
    for t_sent, d in delayed:
        late = [(t, s) for t, k, s, dd in device if k == 2 and dd == d and
                delay_s - judges.HELD_SLACK_S <= t - t_sent <= delay_s + judges.HELD_SLACK_S]
        held += late[:1]
    need(len(held) == len(delayed), f"{len(delayed)} RPDO2 delayed, {len(held)} on vcan_device one "
                                    f"delay later")
    # A seq the master repeated in a later slot may also have an on-time copy: only a seq with
    # every copy delayed can never actuate.
    copies = {}
    for _, k, s, _ in rpdo_arrivals(obs, 3, tag):
        copies[s] = copies.get(s, 0) + (k == 2)
    delayed_seqs = {s for _, s in held if copies.get(s, 0) == len([1 for _, ss in held if ss == s])}
    stops = bank_records(obs, "quick_stop", 3)
    t_stop = stops[0]["t_ns"] / 1e9 if stops else float("inf")
    live = [s for t, s in held if t < t_stop]
    trailing = [s for t, s in held if t > t_stop]
    need(len(live) >= 10, f"only {len(live)} delayed RPDO2 reached node 3 with its tag current")
    # Healthy approvals continue until every delayed frame has arrived, and nothing stopped
    # before then: no drive stop, no group stop, no inhibit.
    last_arrival = max(t for t, _ in held)
    approved = [at(e) for e in cli(obs, "dispatch") if e["approved"]]
    check(approved and max(approved) > last_arrival,
          f"the last approved command ({max(approved) - epoch(obs):.3f} s) precedes the last "
          f"delayed arrival ({last_arrival - epoch(obs):.3f} s)")
    t_end = cli(obs, "stream_end")[0]["t"]
    early = [r for r in obs["bank"] if r.get("event") == "drive" and r["kind"] == "quick_stop" and
             r["t_ns"] / 1e9 < max(approved)]
    check(not early, f"a drive stopped while the stream ran: {early[:2]}")
    inhibited = [e for e in obs["cli"] if e.get("event") in ("stop", "faults") and
                 e["t"] < t_end and (e["event"] == "stop" or e["bits"])]
    check(not inhibited, f"the master stopped or faulted while the stream ran: {inhibited[:2]}")
    tail = sorted(s for s in {r["seq"] for r in bank_records(obs, "actuated", 3)})[-1]
    check(all(s > tail - judges.DELAY_TAIL_SEQS for s in trailing),
          f"delayed RPDO2 of seqs {trailing} reached node 3 after its stop, earlier than the last "
          f"{judges.DELAY_TAIL_SEQS} seqs")
    not_valid = sum(r["count"] for r in bank_records(obs, "dropped", 3)
                    if r["drop"] == judges.DROP_NOT_VALID)
    check(not_valid == len(live), f"{len(live)} delayed RPDO2 reached node 3 with the tag "
                                  f"current, {not_valid} not-valid drops")
    hit = sorted({r["seq"] for r in bank_records(obs, "actuated", 3)} & delayed_seqs)
    check(not hit, f"delayed seqs actuated on node 3: {hit}")
    # No actuation with an expired deadline: each actuated seq's RPDO2 was valid at the instant
    # (D5: valid_until in microseconds mod 2^32; the deadline is open while now < valid_until).
    valid_until = {s: judges.decode("RPDO2", d)["valid_until_us"] for _, k, s, d in device
                   if k == 2}
    expired = []
    for r in bank_records(obs, "actuated", 3):
        check(r["seq"] in valid_until, f"node 3 actuated seq {r['seq']} that has no RPDO2 on "
                                       f"vcan_device")
        ahead = (valid_until[r["seq"]] - (r["t_ns"] // 1000) + 2**31) % 2**32 - 2**31
        if ahead <= 0:
            expired.append((r["seq"], ahead))
    check(not expired, f"node 3 actuated with an expired deadline: {expired[:3]}")
    gaps = sum(r["count"] for r in bank_records(obs, "dropped", 3)
               if r["drop"] == judges.DROP_MISSING_HALF)
    check(gaps >= len(delayed_seqs), f"{len(delayed_seqs)} delayed seqs, only {gaps} missing-half "
                                     f"drops (the lone RPDO1)")
    dispatched = {e["seq"] for e in cli(obs, "dispatch") if e["approved"]}
    for n in NODES:
        if n == 3:
            continue
        act = {r["seq"] for r in bank_records(obs, "actuated", n)}
        check(not [s for s in dispatched if s not in act and s < tail - judges.DELAY_TAIL_SEQS],
              f"node {n} missed dispatched seqs: the delay of node 3 reached another node")
    # The stop observation: the stream ended and every drive stopped on its own, all confirmed.
    for n in NODES:
        own = bank_records(obs, "quick_stop", n)
        check(own and own[0]["reason"] in STOP_REASONS and own[0]["t_ns"] / 1e9 > t_end - 0.05,
              f"node {n}: stop after the stream {[judges.QS_NAMES[r['reason']] for r in own]}")
    group_confirmed(obs)
    return [f"{len(delayed)} RPDO2 of node 3 delayed 60 ms ({len(held)} on vcan_device one delay "
            f"later; seqs {sorted(delayed_seqs)[:4]}...): {len(live)} reached the drive with the "
            f"tag current = {not_valid} not-valid drops; trailing after its stop: {trailing}; "
            f"none of those seqs actuated, no actuation with an expired deadline, {gaps} "
            f"missing-half drops; approvals continued {(max(approved) - last_arrival) * 1000:.0f} "
            f"ms past the last delayed arrival with no stop; then every drive stopped on its own "
            f"and all six were confirmed"]


JUDGES.update({"X1": judge_x1, "X2": judge_x2, "X3": judge_x3, "X4": judge_x4, "X5": judge_x5,
               "X6": judge_x6, "X7": judge_x7})


def verdict(obs):
    """(status, text): PASS with notes, FAIL or INCONCLUSIVE with the reason."""
    try:
        notes = JUDGES[obs["row"]](obs)
    except Inconclusive as e:
        return "INCONCLUSIVE", str(e)
    except Failure as e:
        return "FAIL", str(e)
    return "PASS", "; ".join(notes)
