# SPDX-License-Identifier: Apache-2.0
"""Verdicts of the interop matrix M1-M6 (E3b Task 6), as functions over one captured observation.

An observation is plain JSON, so a recorded one can be judged again without vcan:
  host, device   frames seen on vcan_host (what Lely saw) and vcan_device (what the drives
                 saw): [t, cob_id, hex data], t = the kernel receive stamp as
                 CLOCK_MONOTONIC seconds (conftest.Capture)
  peer           the JSON lines lely_peer printed (Lely's own decoded events)
  bank           the vdrive_bank trace records (t_ns = CLOCK_MONOTONIC nanoseconds)
  proxy_exit     bus_fault_proxy's exit record
  harness        what the test itself did and when (injections, raw frames, marks)

Each judge first asserts its preconditions and raises Inconclusive when one is missing (a
verdict that could not be reached is never a pass); a broken pass condition raises Failure.
Expectations come from cia301.py (the standard's tables), wire.py (the EDS encoder) and od.py
(EDS defaults), never from the C++ code under test. The verdict reads only standard-part
observables (spec 7.2); the drive extension is used only to make PDO traffic happen.
"""
import bisect
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(pathlib.Path(__file__).parent))
sys.path.insert(0, str(ROOT / "tools" / "od"))

import cia301  # noqa: E402
import observation  # noqa: E402
import od  # noqa: E402
import wire  # noqa: E402

EDS_PATH = ROOT / "config" / "od" / "vdrive_arm.eds"
NODE = 3  # The node the matrix judges (M2 judges node 2's heartbeat).
NODES = (1, 2, 3, 4, 5, 6)
GRACE_S = 0.005  # A frame this soon after an NMT command may predate the node's processing.
HB_TIMEOUT_BOUND_S = 1.0  # M2: broad bound for Lely's heartbeat-timeout report.
# B15: objects no master may download to unless the scenario asked for it.
CONST_OBJECTS = ({0x1017} | set(range(0x1400, 0x1403)) | set(range(0x1600, 0x1603)) |
                 set(range(0x1800, 0x1803)) | set(range(0x1A00, 0x1A03)))
OPERATION_ENABLED = 0x27  # CiA 402 statusword, mask 0x6F (used only as a precondition).


class Inconclusive(Exception):
    pass


class Failure(Exception):
    pass


def need(cond, message):
    if not cond:
        raise Inconclusive(message)


def check(cond, message):
    if not cond:
        raise Failure(message)


_EDS = None
_OD = None


def eds():
    global _EDS
    if _EDS is None:
        _EDS = wire.Eds(EDS_PATH)
    return _EDS


def eds_default(index, sub=0):
    global _OD
    if _OD is None:
        _OD = od.load(EDS_PATH)
    return _OD.entry(index, sub).default().value


def pdo_layout(label):
    return next(p for p in eds().pdos() if p["label"] == label)


# ---- observation access ----

def frames(obs, side, cob=None, t0=None, t1=None):
    """[(t, COB-ID, bytes)] on `side`, t strictly between t0 and t1 (observation.frames)."""
    return observation.frames(obs, side, cob, t0, t1)


def events(obs, kind, t0=None, t1=None, **fields):
    return [e for e in obs["peer"] if e.get("event") == kind and
            all(e.get(k) == v for k, v in fields.items()) and
            (t0 is None or e["t"] > t0) and (t1 is None or e["t"] < t1)]


def records(obs, record, t0=None, t1=None, **fields):
    return [r for r in obs["bank"] if r.get("event") == record and
            all(r.get(k) == v for k, v in fields.items()) and
            (t0 is None or r["t_ns"] / 1e9 > t0) and (t1 is None or r["t_ns"] / 1e9 < t1)]


def first_event_time(obs, kind, **fields):
    found = events(obs, kind, **fields)
    need(found, f"no Lely '{kind}' event {fields}")
    return found[0]["t"]


def hexes(fs):
    return [f"{t:.4f} {c:03X}#{d.hex()}" for t, c, d in fs]


# ---- preconditions shared by every row ----

def bank_started(bank):
    """A vdrive_bank trace whose start succeeded: a `start` record and no `start_failed` (the
    bank writes one when it ends before its loop runs, e.g. timerfd_settime refused)."""
    need(any(r.get("event") == "start" for r in bank), "vdrive_bank trace has no start record")
    failed = [r for r in bank if r.get("event") == "start_failed"]
    need(not failed, f"vdrive_bank start_failed: {failed[:1]}")


def bank_not_overloaded(bank):
    """No vdrive_bank `overload` record.
    The bank writes one when a wake finds three releases of its 2 ms period skipped, and then
    faults every drive with 0x6100 (spec 10.2), so what the row saw after it is the overload's
    doing, not the row's.
    It is checked before the peer's and the CLI's ends, since a failed scenario is its symptom."""
    overloads = [r for r in bank if r.get("event") == "overload"]
    first = overloads[0] if overloads else {}
    need(not overloads,
         f"vdrive_bank overload at {first.get('t_ns', 0) / 1e9:.6f} s: "
         f"{first.get('consecutive_skipped')} releases of its 2 ms period skipped in a row, every "
         f"drive faulted 0x{first.get('error_code', 0):04X} (spec 10.2; {len(overloads)} overload "
         f"record(s)); the row is not judged")


def complete(obs, scenario):
    """Traces complete, processes ran, the scenario finished; Inconclusive otherwise."""
    bank = obs.get("bank") or []
    need(bank, "no vdrive_bank trace records")
    bank_started(bank)
    bank_not_overloaded(bank)
    overflow = [r for r in bank if r.get("trace_overflow", 0) != 0]
    need(not overflow, f"vdrive_bank trace incomplete: trace_overflow = "
                       f"{overflow[0].get('trace_overflow') if overflow else 0}")
    killed = obs["harness"].get("bank_killed")
    if killed:
        need(killed.get("event") == "killed", f"no harness killed record: {killed}")
    else:
        need(bank[-1].get("event") == "exit", f"vdrive_bank trace has no final exit record: "
                                              f"{bank[-1:]}")
    proxy = obs.get("proxy_exit") or {}
    need(proxy.get("event") == "exit" and proxy.get("trace_overflow") == 0,
         f"bus_fault_proxy trace incomplete: {proxy}")
    need(events(obs, "start"), "lely_peer printed no start line")
    failed = events(obs, "scenario_failed")
    need(not failed, f"the peer scenario failed: {failed}")
    done = events(obs, "scenario_done", scenario=scenario)
    need(len(done) == 1, f"no single scenario_done for {scenario}: {done}")
    need(obs["host"] and obs["device"], "an empty capture")
    return done[0]


def master_traffic(obs):
    """B15: NMT on the wire is exactly the one startup Reset Communication (broadcast) plus the
    commands the scenario asked Lely to send; no download to a const object that the scenario
    did not request."""
    t_start = first_event_time(obs, "start")
    nmt = [(d[0], d[1]) for t, c, d in frames(obs, "host", cia301.NMT_COB, t_start - 1.0)
           if len(d) == 2]
    asked = [(e["cs"], e["node"]) for e in events(obs, "nmt_sent")]
    check(nmt == [(cia301.NMT_RESET_COMMUNICATION, 0)] + asked,
          f"NMT on the wire {[f'{cs:02X} {n:02X}' for cs, n in nmt]} is not the startup reset "
          f"plus the requested {asked}")
    requested = {}
    for e in obs["peer"]:
        if e.get("event", "").startswith("sdo_") and e.get("op") == "write":
            requested[e["index"]] = requested.get(e["index"], 0) + 1
    seen = {}
    for t, c, d in frames(obs, "host"):
        if cia301.SDO_RX_BASE < c <= cia301.SDO_RX_BASE + 0x7F and cia301.is_download_initiate(d):
            index, _ = cia301.multiplexer(d)
            if index in CONST_OBJECTS:
                seen[index] = seen.get(index, 0) + 1
    for index, count in seen.items():
        check(count <= requested.get(index, 0),
              f"{count} download(s) to const object 0x{index:04X} on the wire, "
              f"{requested.get(index, 0)} requested by the scenario")


def lely_pdo_groups(obs, node, t1=None):
    """Lely's rpdo lines for node, regrouped into the TPDOs they came from (by the EDS mapping
    order); returns {label: [values by field name]}."""
    layouts = [pdo_layout(label) for label in ("TPDO1", "TPDO2", "TPDO3")]
    stream = [e for e in events(obs, "rpdo", node=node, t1=t1)]
    unread = events(obs, "rpdo_unread")
    check(not unread, f"Lely decoded entries the peer could not read: {unread[:3]}")
    groups = {p["label"]: [] for p in layouts}
    i = 0
    while i < len(stream):
        for p in layouts:
            keys = [(f["index"], f["sub"]) for f in p["fields"]]
            chunk = stream[i:i + len(keys)]
            if [(e["index"], e["sub"]) for e in chunk] == keys:
                groups[p["label"]].append({f["name"]: e["value"]
                                           for f, e in zip(p["fields"], chunk)})
                i += len(keys)
                break
        else:
            if i + 3 > len(stream):
                break  # A PDO whose last entries came after the cut-off.
            raise Failure(f"Lely's rpdo lines at {i} match no TPDO mapping: {stream[i:i + 3]}")
    return groups


# ---- M1 ----

M1_SEQUENCE = [cia301.NMT_START, cia301.NMT_STOP, cia301.NMT_ENTER_PRE_OPERATIONAL,
               cia301.NMT_RESET_COMMUNICATION, cia301.NMT_RESET_NODE]
TPDO_COBS = tuple(cia301.TPDO_BASE[n] + NODE for n in (1, 2, 3))


def judge_m1(obs):
    done = complete(obs, "console")
    master_traffic(obs)
    notes = []
    cmds = [(t, d[0]) for t, c, d in frames(obs, "host", cia301.NMT_COB)
            if len(d) == 2 and d[1] == NODE]
    need([cs for _, cs in cmds] == M1_SEQUENCE,
         f"NMT to node {NODE} on the wire {[hex(cs) for _, cs in cmds]}, not the M1 sequence")
    ends = [t for t, _ in cmds[1:]] + [done["t"]]
    # Lely's own events are windowed by Lely's own clock of the commands (nmt_sent), the wire by
    # the capture: Lely reports a reset before its NMT frame is stamped by the capture thread.
    sent = [e["t"] for e in events(obs, "nmt_sent", node=NODE)]
    need(len(sent) == len(M1_SEQUENCE), f"Lely reported {len(sent)} NMT commands sent")
    lely_ends = dict(zip([t for t, _ in cmds], sent[1:] + [done["t"]]))
    lely_starts = dict(zip([t for t, _ in cmds], sent))
    hb_cob = cia301.HEARTBEAT_BASE + NODE
    sdo_tx = cia301.SDO_TX_BASE + NODE
    previous = cia301.STATE_PRE_OPERATIONAL  # The state after boot, before the first command.
    for (t0, cs), t1 in zip(cmds, ends):
        before, previous = previous, cia301.STATE_AFTER[cs]
        expected = cia301.STATE_AFTER[cs]
        beats = frames(obs, "host", hb_cob, t0, t1)
        need(beats, f"no heartbeat of node {NODE} after NMT 0x{cs:02X}")
        lt1 = lely_ends[t0]
        states = [e["state"] for e in events(obs, "state", t0 - GRACE_S, lt1, node=NODE)]
        if cs in cia301.RESETS:
            # A heartbeat of the state before the reset, sent just before the node handled the
            # command, may cross the command in the proxy and be stamped after it. Only that is
            # tolerated: the previous state's byte, ahead of the boot-up, within GRACE_S.
            crossing = []
            while (beats and beats[0][0] <= t0 + GRACE_S and
                   beats[0][2] == bytes([before]) and before != cia301.STATE_BOOT_UP):
                crossing.append(beats.pop(0))
            check(beats and beats[0][2] == bytes([cia301.STATE_BOOT_UP]),
                  f"NMT 0x{cs:02X}: first message {hexes(beats[:1])}, not the boot-up "
                  f"(after {len(crossing)} crossing heartbeat(s) within {GRACE_S} s)")
            t_boot = beats[0][0]
            early = [f for f in beats[1:] if f[0] <= t_boot + GRACE_S]
            check(not early, f"NMT 0x{cs:02X}: heartbeat right after the boot-up {hexes(early)}")
            check(len(beats) >= 2 and all(d == bytes([expected]) for _, _, d in beats[1:]),
                  f"NMT 0x{cs:02X}: heartbeats after the boot-up {hexes(beats[1:])}")
            boots = events(obs, "boot", t0 - GRACE_S, lt1, node=NODE)
            check(boots and boots[-1]["es"] == "" and boots[-1]["state"] == expected,
                  f"NMT 0x{cs:02X}: Lely boot events {boots}")
            check(cia301.STATE_BOOT_UP in states, f"NMT 0x{cs:02X}: Lely states {states}")
            notes.append(f"0x{cs:02X}: {len(crossing)} crossing, boot-up then {len(beats) - 1} x "
                         f"0x{expected:02X}; Lely states {states}, boot state "
                         f"{boots[-1]['state']}")
            continue
        judged = [f for f in beats if f[0] > t0 + GRACE_S]
        need(judged, f"no heartbeat to judge after NMT 0x{cs:02X}")
        check(all(d == bytes([expected]) for _, _, d in judged),
              f"NMT 0x{cs:02X}: heartbeat bytes {hexes(judged)}, CiA 301 says 0x{expected:02X}")
        check(states and all(s == expected for s in states),
              f"NMT 0x{cs:02X}: Lely state events {states}, expected {expected}")
        syncs = [t for t, _, _ in frames(obs, "host", cia301.SYNC_COB, t0 + 0.02, t1 - 0.02)]
        need(len(syncs) >= 5, f"NMT 0x{cs:02X}: only {len(syncs)} SYNC frames in the state")
        tpdos = frames(obs, "host", TPDO_COBS, t0 + GRACE_S, t1)
        lely_pdos = events(obs, "rpdo", t0 + GRACE_S, lt1, node=NODE)
        if expected not in cia301.PDO_STATES:
            # Lely applies a synchronous TPDO at the next SYNC (CiA 301 synchronous
            # reception): the TPDO sent before the command is reported after Lely's first SYNC
            # line after it. Lely prints each SYNC's rpdo lines from its event loop before it
            # handles the next SYNC, so Lely's lines are judged from its second SYNC line after
            # the command on: ordering, not a time window (one SYNC of observation is spent).
            lt0 = lely_starts[t0]
            lely_syncs = [e["t"] for e in events(obs, "sync", lt0)]
            need(len(lely_syncs) >= 2, f"NMT 0x{cs:02X}: fewer than two Lely SYNCs after the "
                                       f"command")
            lely_pdos = [e for e in lely_pdos if e["t"] > lely_syncs[1]]
        if expected in cia301.PDO_STATES:
            for a, b in zip(syncs, syncs[1:]):
                got = sorted(c for t, c, _ in tpdos if a < t < b)
                check(got == sorted(TPDO_COBS),
                      f"Operational: TPDOs after the SYNC at {a:.4f}: {[hex(c) for c in got]}")
            sw = [e for e in lely_pdos if e["index"] == 0x6041]
            check(len(sw) >= len(syncs) - 1,
                  f"Operational: Lely decoded {len(sw)} TPDO1 for {len(syncs)} SYNCs")
            notes.append(f"0x{cs:02X}: {len(judged)} x 0x{expected:02X}; TPDO1-3 after each of "
                         f"{len(syncs) - 1} SYNCs; Lely decoded {len(sw)} TPDO1")
        else:
            check(not tpdos, f"state 0x{expected:02X}: TPDOs after SYNC {hexes(tpdos[:3])}")
            check(not lely_pdos, f"state 0x{expected:02X}: Lely decoded PDO values")
            notes.append(f"0x{cs:02X}: {len(judged)} x 0x{expected:02X}; no TPDO over "
                         f"{len(syncs)} SYNCs; Lely decoded none")
        reads = [e for e in obs["peer"] if e.get("event", "").startswith("sdo_") and
                 e.get("index") == 0x1000 and e.get("node") == NODE and t0 < e["t"] < t1]
        responses = frames(obs, "host", sdo_tx, t0, t1)
        requests = [f for f in frames(obs, "host", cia301.SDO_RX_BASE + NODE, t0, t1)
                    if f[2] == cia301.sdo_upload_request(NODE, 0x1000, 0)[1]]
        if cs == cia301.NMT_STOP:
            need(requests and reads, "Stopped: the scenario's SDO upload of 0x1000 did not run")
            check([e["event"] for e in reads] == ["sdo_timeout"],
                  f"Stopped: Lely's upload of 0x1000 ended {reads}, not sdo_timeout")
            check(not responses, f"Stopped: SDO response on the wire {hexes(responses)}")
            notes.append("Stopped: upload of 0x1000 sent, no 0x583, Lely sdo_timeout")
        elif cs == cia301.NMT_ENTER_PRE_OPERATIONAL:
            need(requests and reads, "Pre-operational: the SDO upload of 0x1000 did not run")
            value = eds_default(0x1000)
            check([(e["event"], e.get("value")) for e in reads] == [("sdo_ok", value)],
                  f"Pre-operational: Lely's upload of 0x1000 ended {reads}")
            check(cia301.sdo_upload_response(NODE, 0x1000, 0, 4, value)[1] in
                  [d for _, _, d in responses], f"Pre-operational: responses {hexes(responses)}")
            notes.append(f"Pre-operational: upload of 0x1000 answered 0x{value:08X}")
    return notes


# ---- M2 ----

M2_NODE = 2


def judge_m2(obs):
    complete(obs, "console")
    master_traffic(obs)
    h = obs["harness"]
    t_bank, t_unmute_b, t_mute_b, t_unmute_c = (h["t_bank_start"], h["t_unmute_b"], h["t_mute_b"],
                                                h["t_unmute_c"])
    t_kill = h["bank_killed"]["t_mono"]
    t_end = h["t_end"]
    notes = []
    need(first_event_time(obs, "start") < t_bank, "the peer was not started before the bank")
    need(any(r.get("text") == f"ok hb_mute {M2_NODE} 1" for r in records(obs, "inject")),
         "the bank did not record the start-time mute")
    hb_cob = cia301.HEARTBEAT_BASE + M2_NODE
    # Phase A: first heartbeat missing.
    phase_a = frames(obs, "host", hb_cob, t_bank, t_unmute_b)
    need(phase_a, f"no message of node {M2_NODE} after the bank started")
    check([d for _, _, d in phase_a] == [bytes([cia301.STATE_BOOT_UP])],
          f"phase A: node {M2_NODE} sent {hexes(phase_a)}, expected the boot-up only")
    t_boot = phase_a[0][0]
    sdo = frames(obs, "host", (cia301.SDO_RX_BASE + M2_NODE, cia301.SDO_TX_BASE + M2_NODE),
                 t_boot, t_unmute_b)
    device_type = eds_default(0x1000)
    for frame in (cia301.sdo_upload_request(M2_NODE, 0x1000, 0),
                  cia301.sdo_upload_response(M2_NODE, 0x1000, 0, 4, device_type)):
        check(frame in [(c, d) for _, c, d in sdo],
              f"phase A: Lely's boot upload {frame[0]:03X}#{frame[1].hex()} not on the wire")
    for sub in (1, 2, 3, 4):
        ok = events(obs, "sdo_ok", t_boot, t_unmute_b, node=M2_NODE, index=0x1018, sub=sub)
        need(ok or events(obs, "sdo_abort", node=M2_NODE, index=0x1018, sub=sub),
             f"phase A: the identity upload 0x1018:{sub} did not run")
        value = eds_default(0x1018, sub)
        check([e["value"] for e in ok] == [value], f"phase A: 0x1018:{sub} = {ok}, EDS {value}")
        check((cia301.SDO_TX_BASE + M2_NODE,
               cia301.sdo_upload_response(M2_NODE, 0x1018, sub, 4, value)[1]) in
              [(c, d) for _, c, d in sdo], f"phase A: 0x1018:{sub} response not on the wire")
    boots = events(obs, "boot", t_bank, t_unmute_b)
    by_node = {n: [e["es"] for e in boots if e["node"] == n] for n in NODES}
    check(by_node[M2_NODE] == ["K"], f"phase A: Lely boot status of node {M2_NODE}: "
                                     f"{by_node[M2_NODE]}, expected exactly 'K'")
    for n in NODES:
        if n != M2_NODE:
            check(by_node[n] == [""], f"phase A: Lely boot status of node {n}: {by_node[n]}")
    check(not events(obs, "hb", t_bank, t_unmute_b), "phase A: a heartbeat event in Lely")
    notes.append(f"A: node 2 boot-up only, boot upload 0x1000 = 0x{device_type:08X}, identity "
                 f"0x1018:1-4 uploaded; Lely boot 'K' for node 2, '' for 1,3-6")
    # Phase B: running timeout.
    beats_b = frames(obs, "host", hb_cob, t_unmute_b, t_mute_b)
    need(len(beats_b) >= 3, f"phase B: {len(beats_b)} heartbeats of node 2 before the mute")
    check(all(d == bytes([cia301.STATE_PRE_OPERATIONAL]) for _, _, d in beats_b),
          f"phase B: heartbeat bytes {hexes(beats_b)}")
    check(not events(obs, "hb", t_unmute_b, t_mute_b, node=M2_NODE, occurred=True),
          "phase B: Lely reported a timeout while heartbeats were sent")
    occurred = events(obs, "hb", t_mute_b, t_unmute_c, node=M2_NODE, occurred=True)
    check(occurred and occurred[0]["t"] - t_mute_b <= HB_TIMEOUT_BOUND_S,
          f"phase B: Lely hb occurred for node 2 {occurred} (mute at {t_mute_b:.4f})")
    notes.append(f"B: {len(beats_b)} heartbeats, Lely hb occurred "
                 f"{occurred[0]['t'] - t_mute_b:.3f} s after the mute, "
                 f"{occurred[0]['t'] - beats_b[-1][0]:.3f} s after the last heartbeat")
    # Phase C: the process dies.
    need(events(obs, "hb", t_unmute_c, t_kill, node=M2_NODE, occurred=False),
         "phase C: Lely did not report node 2's heartbeat resolved before the kill")
    need(len(frames(obs, "host", hb_cob, t_unmute_c, t_kill)) >= 3,
         "phase C: fewer than 3 heartbeats of node 2 before the kill")
    need(t_end - t_kill >= 2 * HB_TIMEOUT_BOUND_S, "the capture ended too soon after the kill")
    delays = {}
    for n in NODES:
        hb = events(obs, "hb", t_kill, t_end, node=n, occurred=True)
        check(hb and hb[0]["t"] - t_kill <= HB_TIMEOUT_BOUND_S,
              f"phase C: Lely hb occurred for node {n}: {hb}")
        delays[n] = round(hb[0]["t"] - t_kill, 3)
    check(not frames(obs, "host", cia301.NMT_COB, t_bank), "NMT sent after the bank started")
    notes.append(f"C: SIGKILL of the bank = six-node common failure; Lely hb occurred for all "
                 f"six, delays {delays}; no NMT after the bank started")
    return notes


# ---- M3 ----

# In the order sdo_suite performs them.
M3_UPLOADS = [(0x1000, 0, 4), (0x1017, 0, 2), (0x1018, 1, 4)]
# (index, sub, size, value written, CiA 301 abort code or None)
M3_DOWNLOADS = [(0x6060, 0, 1, 8, None),
                (0x1000, 0, 4, 0, cia301.ABORT_READ_ONLY),
                (0x1017, 0, 2, 100, cia301.ABORT_READ_ONLY),
                (0x6060, 0, 1, 9, cia301.ABORT_INVALID_VALUE),
                (0x6060, 0, 4, 8, cia301.ABORT_LENGTH)]


def subsequence(expected, actual):
    """True when every item of expected occurs in actual, in order."""
    it = iter(actual)
    return all(any(x == y for y in it) for x in expected)


def judge_m3(obs):
    done = complete(obs, "sdo_suite")
    master_traffic(obs)
    t_start = first_event_time(obs, "boot", node=NODE)
    notes = []
    requests = [d for _, _, d in frames(obs, "device", cia301.SDO_RX_BASE + NODE, t_start)]
    responses = [d for _, _, d in frames(obs, "host", cia301.SDO_TX_BASE + NODE, t_start)]
    need(requests and responses, "no SDO traffic of the suite on the wire")
    want_req, want_resp = [], []
    for index, sub, size in M3_UPLOADS:
        value = eds_default(index, sub)
        ok = events(obs, "sdo_ok", node=NODE, op="read", index=index, sub=sub)
        need(ok, f"the upload of 0x{index:04X}:{sub} did not complete")
        check([e["value"] for e in ok] == [value],
              f"Lely uploaded 0x{index:04X}:{sub} = {[e['value'] for e in ok]}, EDS {value}")
        want_req.append(cia301.sdo_upload_request(NODE, index, sub)[1])
        want_resp.append(cia301.sdo_upload_response(NODE, index, sub, size, value)[1])
        notes.append(f"upload 0x{index:04X}:{sub} = {value} (EDS), response "
                     f"{want_resp[-1].hex()}")
    check(subsequence(want_req, requests), "upload requests on vcan_device differ from CiA 301")
    check(subsequence(want_resp, responses), "upload responses on vcan_host differ from CiA 301")
    want_req, want_resp, lely = [], [], []
    for index, sub, size, value, abort in M3_DOWNLOADS:
        want_req.append(cia301.sdo_download_request(NODE, index, sub, size, value)[1])
        want_resp.append(cia301.sdo_download_response(NODE, index, sub)[1] if abort is None
                         else cia301.sdo_abort(NODE, index, sub, abort)[1])
        lely.append(("sdo_ok", index, None) if abort is None else ("sdo_abort", index, abort))
    check(subsequence(want_req, requests),
          f"download requests on vcan_device {[d.hex() for d in requests]} differ from CiA 301 "
          f"{[d.hex() for d in want_req]}")
    check(subsequence(want_resp, responses),
          f"download responses on vcan_host {[d.hex() for d in responses]} differ from CiA 301 "
          f"{[d.hex() for d in want_resp]}")
    got = [(e["event"], e["index"], e.get("code")) for e in obs["peer"]
           if e.get("event") in ("sdo_ok", "sdo_abort") and e.get("op") == "write"]
    check(got == lely, f"Lely's download results {got}, expected {lely}")
    upload_abort = (cia301.sdo_upload_request(NODE, 0x2FFF, 0)[1],
                    cia301.sdo_abort(NODE, 0x2FFF, 0, cia301.ABORT_NO_OBJECT)[1])
    check(upload_abort[0] in requests and upload_abort[1] in responses,
          "upload of 0x2FFF: request or abort 0x06020000 not on the wire")
    check([e["code"] for e in events(obs, "sdo_abort", op="read")] == [cia301.ABORT_NO_OBJECT],
          f"Lely's upload aborts {events(obs, 'sdo_abort', op='read')}")
    check(not events(obs, "sdo_timeout"), "an SDO transfer timed out")
    notes.append("download 0x6060 = 8: request " + want_req[0].hex() + ", confirm " +
                 want_resp[0].hex() + " (the closed profile has no multi-valued standard "
                 "writable object: this proves the transaction, not a mutation)")
    notes.append("aborts on the wire and in Lely: " + ", ".join(
        f"0x{i:04X}={a:08X}" for i, _, _, _, a in M3_DOWNLOADS if a) + ", 0x2FFF=06020000")
    notes.append(f"peer counters {done}")
    return notes


# ---- M4 ----

def judge_m4(obs):
    done = complete(obs, "pdo_stream")
    master_traffic(obs)
    h = obs["harness"]
    tag = done["tag"]
    need(tag != 0 and done["sent"] == h["count"], f"the stream did not run: {done}")
    need(any(e["index"] == 0x6041 and e["value"] & 0x6F == OPERATION_ENABLED
             for e in events(obs, "rpdo", node=NODE)), "node 3 never reported Operation Enabled")
    notes = []
    # TPDO1-3 as Lely saw them, against Lely's decoded values encoded by wire.py.
    cut = done["t"] - 0.05
    groups = lely_pdo_groups(obs, NODE)
    for n in (1, 2, 3):
        layout = pdo_layout(f"TPDO{n}")
        on_wire = frames(obs, "host", cia301.TPDO_BASE[n] + NODE, t1=cut)
        need(len(on_wire) >= 20, f"TPDO{n}: only {len(on_wire)} frames")
        check(all(len(d) == 8 for _, _, d in on_wire),
              f"TPDO{n}: DLC {sorted({len(d) for _, _, d in on_wire})}, expected 8")
        decoded = groups[layout["label"]]
        check(len(decoded) >= len(on_wire),
              f"TPDO{n}: {len(on_wire)} frames, Lely decoded {len(decoded)}")
        for i, (t, _, data) in enumerate(on_wire):
            expected = bytes(wire.pdo_frame(layout, NODE, decoded[i], "lely")["data"])
            check(expected == data, f"TPDO{n} #{i} at {t:.4f}: wire {data.hex()}, wire.py "
                                    f"encoding of Lely's values {decoded[i]} is {expected.hex()}")
        notes.append(f"TPDO{n}: {len(on_wire)} frames, DLC 8, each equal to wire.py(Lely's "
                     f"decoded values)")
    # RPDO1-3 as the drive saw them, against wire.py encodings of what the peer wrote.
    injected = [bytes.fromhex(x) for x in h["injected"]]
    rpdo1, rpdo2 = pdo_layout("RPDO1"), pdo_layout("RPDO2")
    device = {n: [d for _, _, d in frames(obs, "device", cia301.RPDO_BASE[n] + NODE)]
              for n in (1, 2, 3)}
    for data in injected:
        for n in (1, 2):
            if data in device[n]:
                device[n].remove(data)
                break
        else:
            raise Inconclusive(f"the injected frame {data.hex()} is not on vcan_device")
    for n in (1, 2):
        check(all(len(d) == 8 for d in device[n]),
              f"RPDO{n} from Lely with DLC {sorted({len(d) for d in device[n]})}")
    stream = events(obs, "stream", node=NODE)
    need(len(stream) == h["count"], f"{len(stream)} stream lines for {h['count']} updates")
    for e in stream:
        f1 = bytes(wire.pdo_frame(rpdo1, NODE, {"target_position": e["target"],
                                                "command_seq": e["seq"], "exec_tag": e["tag"]},
                                  "s")["data"])
        f2 = bytes(wire.pdo_frame(rpdo2, NODE, {"valid_until_us": e["valid_until"],
                                                "command_seq": e["seq"], "exec_tag": e["tag"]},
                                  "s")["data"])
        check(f1 in device[1], f"RPDO1 seq {e['seq']}: {f1.hex()} not on vcan_device")
        check(f2 in device[2], f"RPDO2 seq {e['seq']}: {f2.hex()} not on vcan_device")
    rpdo3 = pdo_layout("RPDO3")
    cws = [bytes(wire.pdo_frame(rpdo3, NODE, {"controlword": e["value"]}, "c")["data"])
           for e in events(obs, "cw", node=NODE)]
    need(cws, "the scenario wrote no controlword")
    check(device[3] == cws, f"RPDO3 on vcan_device {[d.hex() for d in device[3]]}, wire.py "
                            f"{[d.hex() for d in cws]}")
    notes.append(f"RPDO1/RPDO2: {len(stream)} updates, DLC 8, equal to wire.py; RPDO3 "
                 f"{[d.hex() for d in cws]} (DLC 2)")
    # The DLC 7 RPDO1 written by the test.
    # The window is closed on the bank's own clock: `before` is the last snapshot before the
    # harness sent the DLC 7 frame; `after` is the first snapshot after the bank actuated the
    # control pair sent behind it, so the bank had dispatched the DLC 7 frame by then (a snapshot
    # between the send and the dispatch still shows the old count). Lely sends only DLC 8 RPDOs,
    # so nothing else in the window can raise `malformed`.
    bad, control = h["bad"], h["control"]
    snaps = records(obs, "dispatch", node=NODE)
    actuated = records(obs, "drive", node=NODE, kind="actuated")
    controls = [r for r in actuated if r["seq"] == control["seq"] and r["tag"] == tag]
    need(controls, f"the DLC 8 control pair (seq {control['seq']}) was not actuated, so the "
                   f"missing actuation of the DLC 7 frame would prove nothing")
    before = [r for r in snaps if r["t_ns"] / 1e9 < bad["t"]]
    after = [r for r in snaps if r["t_ns"] > controls[0]["t_ns"]]
    need(before and after, "no dispatch snapshot before the DLC 7 frame and after the control "
                           "pair's actuation")
    delta = after[0]["malformed"] - before[-1]["malformed"]
    check(delta == 1, f"node 3 malformed count rose by {delta} across the DLC 7 RPDO1")
    check(not any(r["seq"] == bad["seq"] for r in actuated),
          f"the DLC 7 RPDO1 (seq {bad['seq']}) was actuated")
    notes.append(f"DLC 7 RPDO1 seq {bad['seq']}: malformed {before[-1]['malformed']} -> "
                 f"{after[0]['malformed']} in the first snapshot after the control pair (seq "
                 f"{control['seq']}, DLC 8) was actuated; no kActuated of the DLC 7 frame")
    return notes


# ---- M5 ----

def judge_m5(obs):
    done = complete(obs, "pdo_stream")
    master_traffic(obs)
    tag = done["tag"]
    need(tag != 0 and done["sent"] >= 20, f"the stream did not run: {done}")
    syncs = [t for t, _, _ in frames(obs, "device", cia301.SYNC_COB)]
    need(len(syncs) >= 20, "too few SYNC frames on vcan_device")
    need(len(events(obs, "sync")) >= len(events(obs, "stream")),
         "Lely reported fewer of its own SYNCs than updates")

    def interval(t):
        return bisect.bisect_left(syncs, t) - 1

    tag_bytes = tag.to_bytes(2, "little")
    first = {}
    for n in (1, 2):
        for t, _, d in frames(obs, "device", cia301.RPDO_BASE[n] + NODE):
            if len(d) == 8 and d[6:8] == tag_bytes:
                first.setdefault((n, int.from_bytes(d[4:6], "little")), t)
    t_oe = min((e["t"] for e in events(obs, "rpdo", node=NODE, index=0x6041)
                if e["value"] & 0x6F == OPERATION_ENABLED), default=None)
    need(t_oe is not None, "node 3 never reported Operation Enabled")
    population = []
    for e in events(obs, "stream", node=NODE):
        k = e["seq"]
        need((1, k) in first and (2, k) in first, f"seq {k}: RPDO1/RPDO2 not on vcan_device")
        j1, j2 = interval(first[(1, k)]), interval(first[(2, k)])
        need(j1 == j2 and j1 >= 0, f"seq {k}: RPDO1 and RPDO2 not between the same SYNCs")
        if first[(1, k)] > t_oe:
            population.append((k, j1))
    need(len(population) >= 20, f"only {len(population)} commands after Operation Enabled")
    actuated = {r["seq"] for r in records(obs, "drive", node=NODE, kind="actuated", tag=tag)}
    missing = [k for k, _ in population if k not in actuated]
    need(not missing, f"required kActuated missing for seq {missing}")
    tpdo2 = [(interval(t), int.from_bytes(d[4:6], "little"), int.from_bytes(d[6:8], "little"))
             for t, _, d in frames(obs, "device", cia301.TPDO_BASE[2] + NODE) if len(d) == 8]
    for k, j in population:
        seen = [i for i, _, applied in tpdo2 if applied == k]
        check(seen and seen[0] == j + 1,
              f"seq {k} sent after SYNC {j}: applied_seq first in the TPDO2 after SYNC "
              f"{seen[0] if seen else None}, expected {j + 1}")
    lo, hi = population[0][1], population[-1][1] + 2
    window = [(i, s) for i, s, _ in tpdo2 if lo <= i <= hi]
    check([i for i, _ in window] == list(range(lo, hi + 1)),
          f"not exactly one TPDO2 per SYNC in [{lo}, {hi}]: {[i for i, _ in window]}")
    steps = {(b - a) % 65536 for (_, a), (_, b) in zip(window, window[1:])}
    check(steps == {1}, f"sample_seq steps per SYNC {steps}, expected +1")
    lely_applied = {e["value"] for e in events(obs, "rpdo", node=NODE, index=0x2030, sub=2)}
    check(all(k in lely_applied for k, _ in population),
          f"Lely did not decode applied_seq {[k for k, _ in population if k not in lely_applied]}")
    return [f"{len(population)} commands: RPDO1/2 seq k between SYNC j and j+1 on vcan_device; "
            f"TPDO2 applied_seq = k first after SYNC j+1, never after SYNC j; sample_seq +1 over "
            f"{len(window)} SYNCs; Lely decoded every applied_seq; Lely produced "
            f"{len(events(obs, 'sync'))} SYNCs"]


# ---- M6 ----

def judge_m6(obs):
    complete(obs, "console")
    master_traffic(obs)
    h = obs["harness"]
    need(h["acks"] == ["ok"] * 3, f"an injection was refused: {h['acks']}")
    t_f1, t_f2, t_reset, t_stop, t_f3, t_end = (h[k] for k in ("t_f1", "t_f2", "t_reset",
                                                                "t_stop", "t_f3", "t_end"))
    texts = [r["text"] for r in records(obs, "inject")]
    need(texts == [f"ok fault {NODE} 8611", f"ok fault {NODE} 1234", f"ok fault {NODE} 8611"],
         f"the bank did not apply the injections: {texts}")
    emcy_cob = cia301.EMCY_BASE + NODE
    notes = []

    def emcys(t0, t1):
        return ([d for _, _, d in frames(obs, "host", emcy_cob, t0, t1)],
                [(e["code"], e["register"]) for e in events(obs, "emcy", t0, t1, node=NODE)])

    on_wire, lely = emcys(t_f1, t_f2)
    expected = cia301.emcy(NODE, 0x8611, cia301.ERROR_REGISTER_GENERIC)
    check(on_wire == [expected[1]], f"fault 8611: EMCY on the wire {[d.hex() for d in on_wire]}, "
                                    f"CiA 301 {expected[1].hex()}")
    check(lely == [(0x8611, cia301.ERROR_REGISTER_GENERIC)], f"fault 8611: Lely emcy {lely}")
    reg = events(obs, "sdo_ok", t_f1, t_f2, node=NODE, index=0x1001)
    need(reg, "the upload of 0x1001 after the fault did not complete")
    check([e["value"] for e in reg] == [cia301.ERROR_REGISTER_GENERIC], f"0x1001 = {reg}")
    check(cia301.sdo_upload_response(NODE, 0x1001, 0, 1, cia301.ERROR_REGISTER_GENERIC)[1] in
          [d for _, _, d in frames(obs, "host", cia301.SDO_TX_BASE + NODE, t_f1, t_f2)],
          "0x1001 response not on the wire")
    notes.append(f"fault 8611: wire {on_wire[0].hex()}, Lely emcy {lely}, 0x1001 = 0x01")
    on_wire, lely = emcys(t_f2, t_reset)
    check(not on_wire and not lely, f"second fault while in Fault: EMCY {on_wire} / {lely}")
    check(not records(obs, "emcy_sent", t_f2, t_reset), "second fault: emcy_sent in the trace")
    notes.append("fault 1234 while in Fault: no EMCY on the wire or in Lely")
    cw = [d for _, _, d in frames(obs, "device", cia301.RPDO_BASE[3] + NODE, t_f2, t_stop)]
    need(cw == [bytes([0x80, 0x00])], f"the Fault Reset controlword did not reach the drive: {cw}")
    on_wire, lely = emcys(t_reset - 0.01, t_stop)
    reset = cia301.emcy(NODE, cia301.EMCY_ERROR_RESET, cia301.ERROR_REGISTER_NONE)
    check(on_wire == [reset[1]], f"Fault Reset: EMCY on the wire {[d.hex() for d in on_wire]}")
    check(lely == [(cia301.EMCY_ERROR_RESET, cia301.ERROR_REGISTER_NONE)],
          f"Fault Reset: Lely emcy {lely}")
    notes.append(f"Fault Reset by RPDO3: error reset {on_wire[0].hex()} on the wire and in Lely")
    need(events(obs, "state", t_stop, t_f3, node=NODE, state=cia301.STATE_STOPPED),
         "Lely did not report node 3 Stopped before the injection")
    need(bytes([cia301.STATE_STOPPED]) in
         [d for _, _, d in frames(obs, "host", cia301.HEARTBEAT_BASE + NODE, t_stop, t_end)],
         "no Stopped heartbeat before the injection")
    on_wire, lely = emcys(t_stop, t_end)
    check(not on_wire and not lely, f"fault in Stopped: EMCY {on_wire} / {lely}")
    suppressed = records(obs, "emcy_suppressed", t_f3 - 0.01, t_end, node=NODE)
    check([(r["code"], r["state"]) for r in suppressed] == [(0x8611, cia301.STATE_STOPPED)],
          f"fault in Stopped: trace {suppressed}")
    notes.append("fault 8611 in Stopped: no EMCY on the wire or in Lely; trace emcy_suppressed")
    return notes


JUDGES = {"M1": judge_m1, "M2": judge_m2, "M3": judge_m3, "M4": judge_m4, "M5": judge_m5,
          "M6": judge_m6}


def verdict(obs):
    """(status, text): PASS with notes, FAIL or INCONCLUSIVE with the reason."""
    try:
        notes = JUDGES[obs["row"]](obs)
    except Inconclusive as e:
        return "INCONCLUSIVE", str(e)
    except Failure as e:
        return "FAIL", str(e)
    return "PASS", "; ".join(notes)


# ---- Proxy faults through to the drive (E3b Task 7) ----
#
# Rows F1-F6 run pdo_stream --node 3 --count 60 through one rule file of config/interop/faults;
# R1 is the restart row. Preconditions first: the scenario finished, each rule's match count in
# the proxy's exit record equals the frames the rule selects in the capture of its source
# interface (B8 replayed here from the rule text), the 60 seqs are on vcan_host, the traces are
# complete. Then the verdict from the bank trace and the TPDO2 applied_seq on vcan_host.

STREAM_COUNT = 60
# Encodings of the vdrive_bank trace's `drive` records (drive_core DropReason, QuickStopReason).
DROP_OTHER_TAG, DROP_STALE_SEQ, DROP_NOT_VALID, DROP_MISSING_HALF = 0, 1, 2, 3
QS_TIMEOUT, QS_INTEGRITY, QS_EXPIRY = 1, 3, 4
QS_NAMES = {0: "controlword", 1: "timeout", 2: "sync_loss", 3: "integrity", 4: "expiry",
            5: "comm_state"}
QUICK_STOP_ACTIVE = 0x07  # CiA 402 statusword, mask 0x6F
# 0x2030:05 drive_flags (EDS [2030sub5]): bit 0 policy latched, bit 2 integrity.
DRIVE_FLAGS = (0x2030, 5)
FLAG_POLICY_LATCHED = 0x01
FLAG_INTEGRITY = 0x04
# B8 directions: a rule sees the frames read on its source interface. The capture of an
# interface also holds what the proxy wrote there, so the source frames are taken by producer:
# the master produces NMT, SYNC, RPDOs and SDO requests; the drives the rest.
MASTER_COBS = {"nmt": (0x000, 0x000), "sync": (0x080, 0x080), "rpdo": (0x201, 0x47F),
               "sdo_rx": (0x601, 0x67F)}


def master_produced(cob):
    return any(lo <= cob <= hi for lo, hi in MASTER_COBS.values()) and \
        not 0x280 <= cob <= 0x2FF and not 0x380 <= cob <= 0x3FF


def parse_rules(text):
    """B8 rule text -> [{dir, id, mask, every, offset, action}] (the predicates this judge
    replays; a window predicate is not used by these rows and is refused)."""
    rules = []
    for line in text.splitlines():
        words = line.split("#", 1)[0].split()
        if not words:
            continue
        rule = {"dir": words[0], "every": 1, "offset": 0}
        ident, mask = words[1].split("/")
        rule["id"], rule["mask"] = int(ident, 0), int(mask, 0)
        rest = words[2:]
        if rest[0] == "every":
            rule["every"] = int(rest[1])
            rest = rest[2:]
            if rest[0] == "offset":
                rule["offset"] = int(rest[1])
                rest = rest[2:]
        need(rest[0] != "window", "a window predicate is not replayed by this judge")
        rule["action"] = rest[0]
        rule["args"] = rest[1:]
        rules.append(rule)
    return rules


def source_frames(obs, direction):
    """The frames a rule of `direction` saw: what the master wrote on vcan_host (h2d), what the
    drives wrote on vcan_device (d2h)."""
    if direction == "h2d":
        return [f for f in frames(obs, "host") if master_produced(f[1])]
    return [f for f in frames(obs, "device") if not master_produced(f[1])]


def expected_matches(obs, rules):
    """Per rule, the frames it acted on: first rule whose identifier and occurrence predicate
    hold wins (B8); a rule counts the occurrences of its identifier among the frames that reach
    it. Returns [[(t, cob, data), ...] per rule]."""
    matched = [[] for _ in rules]
    for direction in ("h2d", "d2h"):
        seen = [0] * len(rules)
        for f in source_frames(obs, direction):
            for i, r in enumerate(rules):
                if r["dir"] != direction or f[1] & r["mask"] != r["id"] & r["mask"]:
                    continue
                n = seen[i]
                seen[i] += 1
                if n % r["every"] == r["offset"]:
                    matched[i].append(f)
                    break
    return matched


def decode(label, data):
    """Field values of one PDO frame by the EDS mapping (wire.py's layout)."""
    layout = pdo_layout(label)
    need(len(data) == layout["length"], f"{label} frame of {len(data)} bytes")
    return observation.pdo_fields(layout, data)


def encode(label, values):
    """One PDO frame's bytes by the EDS mapping (wire.py's encoder)."""
    return bytes(wire.pdo_frame(pdo_layout(label), NODE, values, "judge")["data"])


def rpdo_seqs(obs, side, n, tag, t0=None, t1=None):
    """[(t, seq, data)] of RPDOn of node 3 with `tag` on `side`."""
    out = []
    for t, _, d in frames(obs, side, cia301.RPDO_BASE[n] + NODE, t0, t1):
        if len(d) == 8:
            v = decode(f"RPDO{n}", d)
            if v["exec_tag"] == tag:
                out.append((t, v["command_seq"], d))
    return out


def drive_records(obs, kind=None, node=NODE, t0=None, t1=None, bank="bank"):
    return observation.drive_records(obs, node, kind, t0, t1, bank)


def dropped(obs, reason, **kw):
    return sum(r["count"] for r in drive_records(obs, "dropped", **kw) if r["drop"] == reason)


def applied_on_host(obs, t0=None, t1=None):
    return [decode("TPDO2", d)["applied_seq"]
            for _, _, d in frames(obs, "host", cia301.TPDO_BASE[2] + NODE, t0, t1) if len(d) == 8]


def drive_flags(obs):
    """0x2030:05 from the harness's SDO upload on vcan_host after the stream (Inconclusive if
    the drive did not answer it)."""
    t = obs["harness"].get("t_flags")
    need(t is not None, "the harness did not read drive_flags")
    index, sub = DRIVE_FLAGS
    for _, _, d in frames(obs, "host", cia301.SDO_TX_BASE + NODE, t):
        if len(d) == 8 and cia301.multiplexer(d) == (index, sub) and d[0] >> 5 == \
                cia301.SCS_UPLOAD_INITIATE:
            return int.from_bytes(d[4:6], "little")
    raise Inconclusive("no SDO upload response for 0x2030:05 on vcan_host")


def last_statusword(obs):
    tpdo1 = [d for _, _, d in frames(obs, "host", cia301.TPDO_BASE[1] + NODE) if len(d) == 8]
    need(tpdo1, "no TPDO1 of node 3 on vcan_host")
    return decode("TPDO1", tpdo1[-1])["statusword"]


def stream_preconditions(obs, bank_obs=None):
    """The shared preconditions of F1-F6; returns (tag, seqs, rules, matched frames)."""
    done = complete(obs, "pdo_stream")
    master_traffic(obs)
    h = obs["harness"]
    tag = done["tag"]
    stream = events(obs, "stream", node=NODE)
    seqs = [e["seq"] for e in stream]
    need(done["sent"] == h["count"] and seqs == list(range(1, h["count"] + 1)) and
         all(e["tag"] == tag for e in stream) and tag != 0,
         f"the stream did not run as declared: {done}, seqs {seqs[:3]}..{seqs[-3:]}")
    rules = parse_rules(h["rules"])
    want = expected_matches(obs, rules)
    got = [r["matched"] for r in obs["proxy_exit"]["rules"]]
    need(got == [len(m) for m in want],
         f"proxy rule match counts {got}, the captures of the source interfaces select "
         f"{[len(m) for m in want]}")
    for n in (1, 2):
        on_host = {s for _, s, _ in rpdo_seqs(obs, "host", n, tag)}
        missing = [s for s in seqs if s not in on_host]
        need(not missing, f"RPDO{n} of seqs {missing} not on vcan_host")
    need(obs["proxy_exit"].get("send_failed") == {"h2d": 0, "d2h": 0},
         f"the proxy failed to send: {obs['proxy_exit'].get('send_failed')}")
    return tag, seqs, rules, want


def device_syncs(obs):
    return [t for t, _, _ in frames(obs, "device", cia301.SYNC_COB)]


def drops_by_sync(obs, reason, syncs, node=NODE):
    """{SYNC index: count} of the bank's drop records of `reason` for node 3. A drop record is
    written at the SYNC that ends its interval; it is matched to the nearest SYNC on vcan_device
    (the two clocks are one CLOCK_MONOTONIC; the capture stamps within a millisecond or two of the
    bank, a SYNC period is 10 ms)."""
    out = {}
    for r in drive_records(obs, "dropped", node=node):
        if r["drop"] != reason:
            continue
        t = r["t_ns"] / 1e9
        i = bisect.bisect_left(syncs, t)
        near = min((j for j in (i - 1, i) if 0 <= j < len(syncs)), key=lambda j: abs(syncs[j] - t))
        need(abs(syncs[near] - t) < 0.004, f"drop record at {t:.4f} is not at a SYNC")
        out[near] = out.get(near, 0) + r["count"]
    return out


def judged_span(obs, t_stop):
    """The SYNC intervals judged after a stop: those opening 5 ms or more after t_stop. Returns
    (the SYNC times, the indices of the judged intervals' opening SYNCs, a predicate telling
    whether a device time lies inside a judged interval)."""
    syncs = device_syncs(obs)
    judged = [i for i in range(len(syncs) - 1) if syncs[i] >= t_stop + 0.005]
    need(len(judged) >= 10, f"only {len(judged)} SYNC intervals after the stop")
    lo, hi = syncs[judged[0]], syncs[judged[-1] + 1]
    return syncs, judged, (lambda t: lo < t < hi)


def other_tag_after(obs, syncs, judged):
    """Checks, for every judged SYNC interval, that the other-tag drops at its closing SYNC
    equal the RPDO1/RPDO2 frames of node 3 inside it; returns the frames judged."""
    drops = drops_by_sync(obs, DROP_OTHER_TAG, syncs)
    cobs = (cia301.RPDO_BASE[1] + NODE, cia301.RPDO_BASE[2] + NODE)
    total = 0
    for i in judged:
        n = len(frames(obs, "device", cobs, syncs[i], syncs[i + 1]))
        check(drops.get(i + 1, 0) == n, f"SYNC interval {i}: {n} RPDO1/RPDO2 of node 3, "
                                        f"{drops.get(i + 1, 0)} other-tag drops at its SYNC")
        total += n
    return total


def actuated_seqs(obs, tag, **kw):
    return [r["seq"] for r in drive_records(obs, "actuated", **kw) if r["tag"] == tag]


def judge_f1(obs):
    """Delay past the deadline: dropped, never actuated; Quick Stop Active, latched."""
    tag, seqs, rules, matched = stream_preconditions(obs)
    delayed = [decode("RPDO2", d)["command_seq"] for _, _, d in matched[0]
               if decode("RPDO2", d)["exec_tag"] == tag]
    need(sorted(set(delayed)) == seqs, f"delayed RPDO2 seqs {sorted(set(delayed))[:5]}...")
    act = actuated_seqs(obs, tag)
    check(not act, f"delayed seqs actuated: {act}")
    stops = drive_records(obs, "quick_stop")
    check(stops and stops[0]["reason"] in (QS_TIMEOUT, QS_EXPIRY),
          f"first quick stop {[(QS_NAMES[r['reason']]) for r in stops]}, expected timeout or "
          f"expiry")
    t_stop = stops[0]["t_ns"] / 1e9
    on_device = rpdo_seqs(obs, "device", 2, tag)
    live = [s for t, s, _ in on_device if t < t_stop]
    not_valid = dropped(obs, DROP_NOT_VALID)
    # A delayed RPDO2 reaching the drive while its tag is current is a not-valid drop at
    # receipt (D5). After the stop revoked the tag, D8 classifies the tag first: every RPDO1 and
    # RPDO2 of node 3 between two SYNCs is then one other-tag drop at the second SYNC, exactly.
    check(not_valid == len(live), f"{len(live)} delayed RPDO2 before the stop, {not_valid} "
                                  f"not-valid drops")
    need({s for _, s, _ in on_device} == set(seqs),
         f"stream seqs whose delayed RPDO2 never reached the drive: "
         f"{sorted(set(seqs) - {s for _, s, _ in on_device})}")
    syncs, judged, accounted = judged_span(obs, t_stop)
    t_last_sync = syncs[-1]
    covered = [s for t, s, _ in on_device if accounted(t)]
    trailing = [s for t, s, _ in on_device if t > t_last_sync]
    # Every delayed RPDO2 on vcan_device is accounted for exactly once: before the stop (a
    # not-valid drop), inside a judged SYNC interval (an other-tag drop at its closing SYNC), or
    # after the last SYNC on the bus (no SYNC ever closed its interval; named).
    unaccounted = [(round(t - t_stop, 4), s) for t, s, _ in on_device
                   if t > t_stop and not accounted(t) and t < t_last_sync]
    need(not unaccounted, f"delayed RPDO2 (s after the stop, seq) not accounted for by a "
                          f"judged SYNC interval: {unaccounted}")
    late = other_tag_after(obs, syncs, judged)
    sw = last_statusword(obs)
    check(sw & 0x6F == QUICK_STOP_ACTIVE, f"last statusword 0x{sw:04X}, not Quick Stop Active")
    flags = drive_flags(obs)
    check(flags & FLAG_POLICY_LATCHED, f"drive_flags 0x{flags:04X}: policy latch not set")
    applied = set(applied_on_host(obs)) - {0}
    check(not applied, f"TPDO2 applied_seq on vcan_host {sorted(applied)}")
    return [f"{len(delayed)} RPDO2 delayed 60 ms ({len(live)} reached the drive before the stop: "
            f"{not_valid} not-valid drops; {len(covered)} after it in judged SYNC intervals among "
            f"{late} RPDO1/RPDO2, each one other-tag drop at its SYNC; trailing after the last "
            f"SYNC: seqs {trailing}); none actuated; quick stop "
            f"{QS_NAMES[stops[0]['reason']]}; statusword 0x{sw:04X}; drive_flags 0x{flags:04X}; "
            f"TPDO2 applied_seq stayed 0"]


JUDGES["F1"] = judge_f1


def no_integrity(obs, **kw):
    integrity = drive_records(obs, "integrity", **kw)
    check(not integrity, f"kIntegrity for node 3 at {[r['t_ns'] for r in integrity]}")


def actuated_once(obs, tag, **kw):
    act = actuated_seqs(obs, tag, **kw)
    check(len(act) == len(set(act)), f"a seq actuated twice: {act}")
    return set(act)


def applied_matches(obs, act):
    applied = set(applied_on_host(obs)) - {0}
    check(applied == act, f"TPDO2 applied_seq on vcan_host {sorted(applied ^ act)} differ from "
                          f"the actuated seqs")


# A lost update is an ordinary gap only while the previous command outlasts it. Lely sends a
# write at the next SYNC and the drive actuates it at the one after, so after one lost update
# the next command is actuated 4 T_bus = 40 ms after the previous write: a validity of 40 ms
# expires at that instant, and whether the drive stops depends on which comes first in its loop.
# The rows with a gap need a validity longer by at least 3 W (the lease allows 48 ms). Which
# seqs a rule hits depends on how many frames of its identifier Lely sent during the commit, so
# the first update may be lost too: the guards count from the entry into Operation Enabled, and
# the entry must come after Lely sent the first update (the peer enables after the second), or a
# lost first update puts the first actuation on the 30 ms command timeout.
GAP_VALIDITY_MIN_US = 40_000 + 3 * 2_000
# The drive's lease (drive_core kLeaseUs), plus 1 µs for the stream line's printed time.
GAP_VALIDITY_MAX_US = 48_000 + 1


def gap_margins(obs, tag):
    """Preconditions of a row whose stream has gaps (F2, F7)."""
    def ahead(e):  # valid_until - write time, signed in the 2^32 µs wire domain
        return (e["valid_until"] - int(e["t"] * 1e6) + 2**31) % 2**32 - 2**31

    out = [e["seq"] for e in events(obs, "stream", node=NODE)
           if not GAP_VALIDITY_MIN_US <= ahead(e) <= GAP_VALIDITY_MAX_US]
    need(not out, f"stream seqs {out[:3]}.. valid for outside [{GAP_VALIDITY_MIN_US // 1000}, "
                  f"48] ms: a lost update lands on the expiry (or past the lease)")
    first = min(t for t, s, _ in rpdo_seqs(obs, "host", 1, tag) if s == 1)
    enable = [t for t, _, d in frames(obs, "host", cia301.RPDO_BASE[3] + NODE)
              if d[:2] == b"\x0f\x00"]
    need(enable and enable[0] > first, "Enable Operation was not sent after the first update: "
                                        "a lost first update lands on the command timeout")


def judge_f2(obs):
    """Every third RPDO1 lost: ordinary gaps, no integrity violation."""
    tag, seqs, rules, matched = stream_preconditions(obs)
    reached = {s for _, s, _ in rpdo_seqs(obs, "device", 1, tag)}
    lost = [s for s in seqs if s not in reached]
    need(len(lost) >= len(seqs) // 3 - 1, f"only {len(lost)} stream RPDO1 lost: {lost}")
    gap_margins(obs, tag)
    no_integrity(obs)
    act = actuated_once(obs, tag)
    last = seqs[-1]
    expected = set(seqs) - set(lost)
    check(act - {last} == expected - {last},
          f"actuated minus expected {sorted(act - expected)}, expected minus actuated "
          f"{sorted(expected - act)}")
    gaps = dropped(obs, DROP_MISSING_HALF)
    check(gaps >= len(set(lost) - {last}),
          f"{len(lost)} lost RPDO1, only {gaps} missing-half drops (an ordinary gap)")
    applied_matches(obs, act)
    t_last = max(r["t_ns"] for r in drive_records(obs, "actuated") if r["tag"] == tag) / 1e9
    stops = drive_records(obs, "quick_stop", t1=t_last)
    check(not stops, f"quick stop during the stream: {stops}")
    return [f"{len(lost)} of {len(seqs)} stream RPDO1 lost ({lost[:6]}...): {gaps} missing-half "
            f"drops, no kIntegrity, no quick stop while streaming; actuated = the other "
            f"{len(act)} seqs; TPDO2 applied_seq on vcan_host the same set"]


def pdo_frames_to_node(obs):
    cobs = tuple(cia301.RPDO_BASE[n] + NODE for n in (1, 2, 3))
    return len(frames(obs, "device", cobs))


def last_dispatch(obs):
    snaps = [r for r in obs["bank"] if r.get("event") == "dispatch" and r.get("node") == NODE]
    need(snaps, "no dispatch snapshot of node 3")
    return snaps[-1]


def judge_f3(obs):
    """Identical duplicates of RPDO1 and RPDO2 change nothing; each is counted."""
    base = obs["baseline"]
    tag_b, seqs_b, _, _ = stream_preconditions(base)
    act_b = actuated_once(base, tag_b)
    tag, seqs, rules, matched = stream_preconditions(obs)
    need([r["action"] for r in rules] == ["dup", "dup"], "the rule file is not two dup rules")
    act = actuated_once(obs, tag)
    need(len(act_b) >= 50, f"the no-rule run actuated only {len(act_b)} seqs")
    check(act == act_b, f"actuated seqs differ from the no-rule run: {sorted(act ^ act_b)}")
    no_integrity(obs)
    counts = obs["proxy_exit"]["rules"]
    for i, n in enumerate((1, 2)):
        cob = cia301.RPDO_BASE[n] + NODE
        check(counts[i]["duplicated"] == counts[i]["matched"],
              f"rule {i}: {counts[i]['duplicated']} duplicated of {counts[i]['matched']} matched")
        on_host = [d for _, c, d in source_frames(obs, "h2d") if c == cob]
        on_device = [d for _, _, d in frames(obs, "device", cob)]
        check(on_device == [d for d in on_host for _ in (0, 1)],
              f"RPDO{n}: {len(on_device)} frames on vcan_device for {len(on_host)} on vcan_host, "
              f"not each twice in order")
    snap = last_dispatch(obs)
    check(snap["event"] == "dispatch" and
          snap["pdo"] + snap["gated"] + snap["malformed"] == pdo_frames_to_node(obs),
          f"node 3 dispatch counts {snap}, {pdo_frames_to_node(obs)} RPDO frames on vcan_device")
    applied_matches(obs, act)
    return [f"{counts[0]['matched']} RPDO1 and {counts[1]['matched']} RPDO2 each forwarded twice; "
            f"node 3 counted all {pdo_frames_to_node(obs)} RPDO frames (pdo {snap['pdo']}, gated "
            f"{snap['gated']}); actuated {len(act)} seqs = the no-rule run's; no kIntegrity"]


def judge_f4(obs):
    """RPDO1 released behind its RPDO2 inside the same SYNC window still pairs."""
    tag, seqs, rules, matched = stream_preconditions(obs)
    reordered = obs["proxy_exit"]["rules"][0]["reordered"]
    need(reordered >= 50, f"only {reordered} frames reordered")
    syncs = device_syncs(obs)
    first = {}
    for n in (1, 2):
        for t, s, _ in rpdo_seqs(obs, "device", n, tag):
            first.setdefault((n, s), t)
    swapped = 0
    for s in seqs:
        need((1, s) in first and (2, s) in first, f"seq {s}: RPDO1/RPDO2 not on vcan_device")
        t1, t2 = first[(1, s)], first[(2, s)]
        j1, j2 = bisect.bisect_left(syncs, t1), bisect.bisect_left(syncs, t2)
        check(j1 == j2, f"seq {s}: RPDO1 and RPDO2 in different SYNC windows")
        swapped += t2 < t1
    need(swapped >= 50, f"only {swapped} seqs reached the drive RPDO2 first")
    no_integrity(obs)
    act = actuated_once(obs, tag)
    check(act == set(seqs), f"seqs not actuated: {sorted(set(seqs) - act)}")
    applied_matches(obs, act)
    return [f"{reordered} reordered by the proxy; {swapped} of {len(seqs)} seqs reached the drive "
            f"RPDO2 first, each pair inside one SYNC window; every seq actuated, no kIntegrity"]


def judge_f5(obs):
    """A conflicting copy of an RPDO1 (same seq, other target) is an integrity stop."""
    tag, seqs, rules, matched = stream_preconditions(obs)
    rule = rules[0]
    byte, mask = int(rule["args"][0], 0), int(rule["args"][1], 0)
    hits = [d for _, _, d in matched[0] if decode("RPDO1", d)["exec_tag"] == tag]
    need(hits, f"no matched RPDO1 carries the stream's tag ({len(matched[0])} matched)")
    original = hits[0]
    copy = bytearray(original)
    copy[byte] ^= mask
    copy = bytes(copy)
    seq = decode("RPDO1", original)["command_seq"]
    device = frames(obs, "device", cia301.RPDO_BASE[1] + NODE)
    at = [i for i, (_, _, d) in enumerate(device) if d == original]
    check(at and at[0] + 1 < len(device) and device[at[0] + 1][2] == copy,
          f"seq {seq}: the original {original.hex()} and its copy {copy.hex()} are not "
          f"consecutive on vcan_device")
    t_copy = device[at[0] + 1][0]
    integrity = drive_records(obs, "integrity")
    check(integrity and abs(integrity[0]["t_ns"] / 1e9 - t_copy) < 0.005,
          f"kIntegrity {[r['t_ns'] for r in integrity]} not at the copy ({t_copy:.4f})")
    stops = drive_records(obs, "quick_stop")
    check(stops and stops[0]["reason"] == QS_INTEGRITY,
          f"quick stops {[QS_NAMES[r['reason']] for r in stops]}, expected integrity first")
    act = actuated_once(obs, tag)
    check(seq not in act and all(s < seq for s in act),
          f"seq {seq} or a later one was actuated: {sorted(s for s in act if s >= seq)}")
    sw = last_statusword(obs)
    check(sw & 0x6F == QUICK_STOP_ACTIVE, f"last statusword 0x{sw:04X}, not Quick Stop Active")
    flags = drive_flags(obs)
    check(flags & FLAG_POLICY_LATCHED and flags & FLAG_INTEGRITY,
          f"drive_flags 0x{flags:04X}: policy latch and integrity bits not both set")
    applied = set(applied_on_host(obs)) - {0}
    check(applied <= act, f"TPDO2 applied_seq {sorted(applied - act)} never actuated")
    return [f"seq {seq}: {original.hex()} then {copy.hex()} on vcan_device; kIntegrity at the "
            f"copy; quick stop integrity; drive_flags 0x{flags:04X}; statusword 0x{sw:04X}; "
            f"actuated {len(act)} seqs, all before {seq}; {len(matched[0]) - len(hits)} matches "
            f"of another tag before the stream"]


def judge_f6(obs):
    """TPDO1 of node 3 lost on the way to the master: feedback hidden, the drive unaffected."""
    tag, seqs, rules, matched = stream_preconditions(obs)
    tpdo1 = cia301.TPDO_BASE[1] + NODE
    need(len(matched[0]) >= len(seqs), f"only {len(matched[0])} TPDO1 dropped")
    on_host = frames(obs, "host", tpdo1)
    check(not on_host, f"TPDO1 of node 3 on vcan_host: {hexes(on_host[:3])}")
    lely = [e for e in events(obs, "rpdo", node=NODE) if e["index"] in (0x6041, 0x6064)]
    check(not lely, f"Lely decoded TPDO1 entries: {lely[:3]}")
    polled = events(obs, "sdo_ok", node=NODE, index=0x6041)
    need(polled, "the peer did not read the statusword by SDO")
    act = actuated_once(obs, tag)
    check(act == set(seqs), f"seqs not actuated: {sorted(set(seqs) - act)}")
    applied_matches(obs, act)
    return [f"{len(matched[0])} TPDO1 dropped: none on vcan_host, Lely decoded no TPDO1 entry (the "
            f"peer read the statusword {len(polled)} times by SDO); every seq actuated, TPDO2 "
            f"applied_seq on vcan_host the same set"]


JUDGES.update({"F2": judge_f2, "F3": judge_f3, "F4": judge_f4, "F5": judge_f5, "F6": judge_f6})


# ---- R1: restart rejects captured old-tag frames ----

def view(obs, peer, bank):
    """One process pair of a multi-run observation, in the shape the helpers above read."""
    return {"row": obs["row"], "host": obs["host"], "device": obs["device"],
            "peer": obs["peers"][peer], "bank": obs["banks"][bank],
            "proxy_exit": obs["proxy_exit"], "harness": obs["harness"]}


def judge_r1(obs):
    h = obs["harness"]
    first, last = view(obs, 0, 0), view(obs, -1, -1)
    # R1 does not go through complete(), so every bank run is checked for an overload here.
    for bank in obs["banks"]:
        bank_not_overloaded(bank)
    for v in (first, last):
        overflow = [r for r in v["bank"] if r.get("trace_overflow", 0) != 0]
        need(not overflow, "vdrive_bank trace incomplete: trace_overflow != 0")
    need(h["bank_killed"][0].get("event") == "killed", f"no killed record: {h['bank_killed']}")
    need(last["bank"][-1].get("event") == "exit", "the restarted bank's trace has no exit record")
    need(obs["proxy_exit"].get("event") == "exit" and obs["proxy_exit"]["trace_overflow"] == 0,
         f"bus_fault_proxy trace incomplete: {obs['proxy_exit']}")
    done = [events(view(obs, i, -1), "scenario_done") for i in range(len(obs["peers"]))]
    need(all(len(d) == 1 for d in done) and not any(
        events(view(obs, i, -1), "scenario_failed") for i in range(len(obs["peers"]))),
         f"a peer scenario did not finish: {done}")
    old_tag, new_tag, stream_tag = done[0][0]["tag"], h["new_tag"], done[-1][0]["tag"]
    need(done[0][0]["sent"] == h["count_first"] and done[-1][0]["sent"] == h["count_second"],
         f"the streams did not run as declared: {done[0]} {done[-1]}")
    reads = [e["value"] for e in obs["peers"][-2] if e.get("event") == "sdo_ok" and
             e.get("index") == 0x2012]
    need(reads and reads[-1] == new_tag, f"the new tag {new_tag} is not the 0x2012 upload {reads}")
    need(len(h["collisions"]) == len(obs["banks"]) - 2,
         f"{len(obs['banks'])} banks for {len(h['collisions'])} recorded collisions")
    check(new_tag != old_tag, f"the restarted drive committed the old tag {old_tag}")
    boot_first = [r for r in first["bank"] if r.get("event") == "start"][0]["boot_ids"]
    boot_last = [r for r in last["bank"] if r.get("event") == "start"][0]["boot_ids"]
    need(boot_first != boot_last, "the restarted bank drew the same boot_ids")
    replay = [bytes.fromhex(d) for _, _, d in h["replay"]]
    need(len(replay) >= 2 * h["count_first"], f"only {len(replay)} old frames captured")
    need(all(len(d) == 8 and d[6:8] == old_tag.to_bytes(2, "little") for d in replay),
         "a replayed frame does not carry the old tag")
    t_sync, t_next = h["t_sync"], h["t_second_stream"]
    t_replay = h["replay"][0][0]
    syncs = frames(obs, "device", cia301.SYNC_COB, t_replay - 0.001, t_next)
    need(len(syncs) == 1 and syncs[0][0] >= t_sync, f"SYNCs around the replay: {hexes(syncs)}")
    # Every RPDO1/RPDO2 of node 3 since the SYNC before the replay is dropped at the next SYNC:
    # the replayed frames, and any frame the console's master sent after its own last SYNC.
    cobs = (cia301.RPDO_BASE[1] + NODE, cia301.RPDO_BASE[2] + NODE)
    t_prev = frames(obs, "device", cia301.SYNC_COB, None, t_replay)[-1][0]
    on_device = [d for _, _, d in frames(obs, "device", cobs, t_prev, t_sync)]
    # The control: one RPDO1 of the new tag, the last frame before the SYNC.
    control = bytes.fromhex(h["control"][2])
    need(decode("RPDO1", control)["exec_tag"] == new_tag, "the control does not carry the new tag")
    need(on_device[-1:] == [control] and
         on_device[len(on_device) - len(replay) - 1:-1] == replay,
         f"the {len(replay)} replayed frames and then the control are not the last RPDO1/RPDO2 "
         f"on vcan_device before the SYNC")
    extra = on_device[:len(on_device) - len(replay) - 1]
    need(all(d[6:8] != new_tag.to_bytes(2, "little") for d in extra),
         f"a frame of the new tag before the replay: {[d.hex() for d in extra]}")
    drops = drive_records(last, "dropped", t0=t_sync, t1=t_next)
    other = sum(r["count"] for r in drops if r["drop"] == DROP_OTHER_TAG)
    lone = sum(r["count"] for r in drops if r["drop"] == DROP_MISSING_HALF)
    rest = [r for r in drops if r["drop"] not in (DROP_OTHER_TAG, DROP_MISSING_HALF)]
    # With the new tag current at the SYNC, the control is a lone half, not an other-tag drop;
    # with the tag revoked it would be one more other-tag drop and the replay would prove nothing.
    check(other == len(on_device) - 1 and lone == 1 and not rest,
          f"{len(replay)} old-tag frames replayed after {len(extra)} of the master's and before "
          f"the control of the new tag: {other} other-tag drops (expected {len(on_device) - 1}), "
          f"{lone} missing-half drop(s) for the control (expected 1); other drops {rest}")
    act_old = [r for r in drive_records(last, "actuated") if r["tag"] == old_tag]
    check(not act_old, f"old-tag seqs actuated after the restart: {[r['seq'] for r in act_old]}")
    check(not drive_records(last, "actuated", t1=t_next), "a command actuated before the stream")
    no_integrity(last)
    applied = applied_on_host(obs, t_sync, t_next)
    need(applied, "no TPDO2 of node 3 on vcan_host after the replay SYNC")
    check(set(applied) == {0}, f"TPDO2 applied_seq after the replay {applied}")
    act = actuated_once(last, stream_tag, t0=t_next)
    check(len(act) >= 8, f"the stream after the restart actuated only {sorted(act)}")
    return [f"tag {old_tag} before the kill, {new_tag} after the restart ({len(h['collisions'])} "
            f"collision(s)); {len(replay)} old-tag RPDO1/RPDO2 replayed on vcan_device (after "
            f"{len(extra)} tag-{[int.from_bytes(d[6:8], 'little') for d in extra]} frames of the "
            f"console's master): {other} other-tag drops at the next SYNC, the control RPDO1 of "
            f"the new tag a lone half (tag still current); none actuated, TPDO2 "
            f"applied_seq 0; then "
            f"pdo_stream --count {h['count_second']} actuated {len(act)}"]


JUDGES["R1"] = judge_r1


DELAY_TAIL_SEQS = 6  # F7: 60 ms of delay is six SYNC periods of stream.
# F7: a held frame reaches vcan_device its delay after it was read on vcan_host, within this
# (both captures stamp in one CLOCK_MONOTONIC; well under the 10 ms between Lely's repeats).
HELD_SLACK_S = 0.004


def judge_f7(obs):
    """A minority of RPDO2 delayed past their deadline: each reaches the drive while its tag is
    current and is exactly one not-valid drop at receipt; those seqs never actuate, the others
    do (ordinary gaps), and the drive stays enabled."""
    tag, seqs, rules, matched = stream_preconditions(obs)
    need(rules[0]["action"] == "delay" and rules[0]["every"] > 1,
         "the rule is not a minority delay")
    last = seqs[-1]
    delayed = [d for _, _, d in matched[0] if decode("RPDO2", d)["exec_tag"] == tag]
    first_rpdo2 = {}
    for t, s, d in rpdo_seqs(obs, "host", 2, tag):
        first_rpdo2.setdefault(s, d)
    delayed_seqs = {s for s, d in first_rpdo2.items() if d in delayed}
    need(len(delayed_seqs - {last}) >= len(seqs) // rules[0]["every"] - 1,
         f"only {len(delayed_seqs)} stream seqs had their RPDO2 delayed")
    gap_margins(obs, tag)
    # The delayed copy of each held frame: the same bytes on vcan_device one delay later. Lely
    # repeats seq 60's frame at every SYNC after the stream, so a held repeat has identical
    # copies that were forwarded at once; those are not the held one.
    delay_s = int(rules[0]["args"][0]) / 1000
    device_rpdo2 = rpdo_seqs(obs, "device", 2, tag)
    on_device = []
    for t_held, _, d in matched[0]:
        if decode("RPDO2", d)["exec_tag"] != tag:
            continue
        late = [(t, s) for t, s, dd in device_rpdo2
                if dd == d and delay_s - HELD_SLACK_S <= t - t_held <= delay_s + HELD_SLACK_S]
        on_device += late[:1]
    need(len(on_device) == len(delayed), f"{len(delayed)} RPDO2 delayed, {len(on_device)} on "
                                         f"vcan_device one delay later")
    act = actuated_once(obs, tag)
    t_last_act = max(r["t_ns"] for r in drive_records(obs, "actuated") if r["tag"] == tag) / 1e9
    early = drive_records(obs, "quick_stop", t1=t_last_act)
    check(not early, f"quick stop while streaming: {[QS_NAMES[r['reason']] for r in early]}")
    stops = drive_records(obs, "quick_stop")
    t_stop = stops[0]["t_ns"] / 1e9 if stops else float("inf")
    live = [s for t, s in on_device if t < t_stop]
    trailing = [s for t, s in on_device if t > t_stop]
    # The stream ends, the last command expires and the drive stops: an RPDO2 delayed in the
    # last 60 ms of the stream arrives after that stop, with the tag revoked (named, not counted
    # as not-valid). The stop is after the last actuation (checked above).
    tail = last - DELAY_TAIL_SEQS
    need(all(s > tail for s in trailing),
         f"delayed RPDO2 of seqs {trailing} reached the drive after its stop, earlier than the "
         f"last {DELAY_TAIL_SEQS} seqs")
    not_valid = dropped(obs, DROP_NOT_VALID)
    check(not_valid == len(live), f"{len(live)} delayed RPDO2 reached the drive with the tag "
                                  f"current, {not_valid} not-valid drops")
    hit = sorted(act & (delayed_seqs - {last}))
    check(not hit, f"delayed seqs actuated: {hit}")
    expected = set(seqs) - delayed_seqs
    check(expected <= act, f"non-delayed seqs not actuated: {sorted(expected - act)}")
    gaps = dropped(obs, DROP_MISSING_HALF)
    check(gaps >= len(delayed_seqs - {last}),
          f"{len(delayed_seqs)} delayed seqs, only {gaps} missing-half drops (the lone RPDO1)")
    applied_matches(obs, act)
    return [f"{len(delayed)} stream RPDO2 delayed 60 ms (seqs {sorted(delayed_seqs)}): {len(live)} "
            f"reached the drive with the tag current, {not_valid} not-valid drops; trailing "
            f"after the drive's stop: {trailing}; none of those seqs actuated, the other "
            f"{len(expected)} all actuated ({gaps} missing-half drops); no quick stop while "
            f"streaming"]


JUDGES["F7"] = judge_f7


# ---- Our master against CANopenNode: the interop matrix N1-N7 (E3c Task 8) ----
#
# canopen_master_cli on vcan_host, bus_fault_proxy with no rules, canopend (CANopenNode v4.1,
# node 2; N7: nodes 1-6) on vcan_device. An observation (test_master_matrix.py) holds:
#   host, device   frames on both links, as above
#   cli            the JSON lines canopen_master_cli printed (console: its cmd/cmd_done lines carry
#                  the console line and its number n; master events carry at_ns)
#   canopend       one record per canopend process: node, started / exited (CLOCK_MONOTONIC s),
#                  exit_code, log [[t, line]] (its stderr, the only report it has of its NMT state)
#   proxy_exit     bus_fault_proxy's exit record (no rules)
#   harness        what the test did: the console numbers n of its steps, signals and their times
# Expectations come from cia301.py and canopennode_facts.py (the pinned build's observed
# behaviour, calibrated by test_canopennode_config.py), never from the master's code.

import canopennode_facts as cnf  # noqa: E402

CN_NODE = cnf.NODE_ID
CN_HB = cia301.HEARTBEAT_BASE + CN_NODE
CN_SDO_TX = cia301.SDO_TX_BASE + CN_NODE
CN_SDO_RX = cia301.SDO_RX_BASE + CN_NODE
CN_TPDO1 = cia301.TPDO_BASE[1] + CN_NODE
CN_RPDO1 = cia301.RPDO_BASE[1] + CN_NODE
CN_EMCY = cnf.EMCY_COB_ID_1014
CN_LOOP = cnf.LOOPBACK_OBJECT
CN_SETTLE_S = 0.005  # A frame this soon after an NMT command may predate canopend's processing.
CN_EVENT_S = 0.020  # A master event follows the frame it reports within this.
# The canopend log line of each heartbeat state byte (canopennode_facts.LOG_NMT).
CN_STATE_LOG = {cia301.STATE_OPERATIONAL: cnf.LOG_NMT["operational"],
                cia301.STATE_STOPPED: cnf.LOG_NMT["stopped"],
                cia301.STATE_PRE_OPERATIONAL: cnf.LOG_NMT["pre_operational"]}
CN_BOOT_TIMEOUT_S = (0.9, 1.1)  # N2: C7's 1000 ms boot window, +- 100 ms.
CN_HB_LOST_S = (0.250, 0.400)  # N2: C7's 250 ms consumer time, observed by 400 ms.
CN_MIN_HEARTBEATS = 3
CN_MIN_TPDOS = 5
CN_N5_VALUES = 20
# N7: what generic_cia402_profile() lacks against --required all, as the activation gate names it
# (runtime_core test_capability.cpp: three unsupported, two unknown).
CN_GENERIC_MISSING = {"DEVICE_SIDE_EXPIRY": "UNSUPPORTED", "EXECUTION_GENERATION": "UNSUPPORTED",
                      "APPLY_ACK": "UNSUPPORTED", "LOCAL_STOP": "UNKNOWN",
                      "SOURCE_TIMESTAMP": "UNKNOWN"}
CN_SESSION_NODES = (1, 2, 3, 4, 5, 6)


def cn_events(obs, kind, t0=None, t1=None, **fields):
    return [e for e in obs["cli"] if e.get("event") == kind and
            all(e.get(k) == v for k, v in fields.items()) and
            (t0 is None or e["t"] > t0) and (t1 is None or e["t"] < t1)]


def cn_at(e):
    """The worker's instant of a master event, seconds."""
    return e["at_ns"] / 1e9


def cn_step(obs, n):
    """(index of the cmd line, cmd, cmd_done) of console line n; Inconclusive when it is missing
    or was not carried out."""
    cmds = [(i, e) for i, e in enumerate(obs["cli"]) if e.get("event") == "cmd" and e["n"] == n]
    need(len(cmds) == 1, f"console line {n}: {len(cmds)} cmd lines")
    done = cn_events(obs, "cmd_done", n=n)
    need(len(done) == 1 and done[0]["ok"], f"console line {n} ({cmds[0][1]['line']!r}) was not "
                                           f"carried out: {done}")
    return cmds[0][0], cmds[0][1], done[0]


def cn_sdo(obs, n):
    """The one sdo_* result of console line n (it is printed between its cmd and cmd_done)."""
    i, cmd, done = cn_step(obs, n)
    found = []
    for e in obs["cli"][i + 1:]:
        if e.get("event") == "cmd_done" and e.get("n") == n:
            break
        if e.get("event", "").startswith("sdo_"):
            found.append(e)
    need(len(found) == 1, f"console line {n} ({cmd['line']!r}): {len(found)} SDO results")
    return found[0]


def cn_sdo_value(obs, n):
    e = cn_sdo(obs, n)
    need(e["event"] == "sdo_ok", f"console line {n}: {e}")
    return e["value"]


def cn_sdo_wire(obs, n, side="host"):
    """(requests, responses, client aborts) of console line n's SDO exchange on one link: node
    2's frames between its cmd and cmd_done lines. A client abort is what our master sends when
    its request times out (CiA 301: abort transfer on the client-to-server identifier).
    On vcan_device a frame the master sends last (the abort) may arrive after cmd_done, since the
    proxy's forwarding is not ordered against the CLI's print; judge_n1 widens that search."""
    _, cmd, done = cn_step(obs, n)
    sent = frames(obs, side, CN_SDO_RX, cmd["t"], done["t"])
    reqs = [f for f in sent if f[2][:1] and f[2][0] >> 5 != cia301.CS_ABORT]
    aborts = [f for f in sent if f[2][:1] and f[2][0] >> 5 == cia301.CS_ABORT]
    resps = frames(obs, side, CN_SDO_TX, cmd["t"], done["t"])
    return reqs, resps, aborts


def cn_line(obs, n):
    return cn_step(obs, n)[1]["line"]


def cn_complete(obs, scenario="console", ending="scenario_done"):
    """The CLI ran to its scenario's end (`ending`, or one of a tuple of them), the proxy forwarded
    with no rules and complete counts, both captures saw traffic, and every canopend reported
    running.
    Inconclusive otherwise."""
    cli = obs.get("cli") or []
    need(len(cn_events(obs, "start")) == 1, "canopen_master_cli printed no single start line")
    need(cli and cli[-1].get("event") == "exit", f"the CLI's last line is not exit: {cli[-1:]}")
    need(cli[-1].get("event_overflows") == 0, f"the CLI lost events: {cli[-1]}")
    endings = (ending,) if isinstance(ending, str) else ending
    end = [e for name in endings for e in cn_events(obs, name, scenario=scenario)]
    need(len(end) == 1, f"no single {' or '.join(endings)} for {scenario}: "
                        f"{cn_events(obs, 'scenario_failed') or cn_events(obs, 'scenario_done')}")
    proxy = obs.get("proxy_exit") or {}
    need(proxy.get("event") == "exit" and proxy.get("trace_overflow") == 0,
         f"bus_fault_proxy trace incomplete: {proxy}")
    need(proxy.get("rules") == [], f"bus_fault_proxy ran with rules: {proxy.get('rules')}")
    need(proxy.get("send_failed") == {"h2d": 0, "d2h": 0} and proxy.get("dropped_at_exit") == 0,
         f"bus_fault_proxy lost frames: {proxy}")
    need(obs["host"] and obs["device"], "an empty capture")
    daemons = obs.get("canopend") or []
    need(daemons, "no canopend record")
    for d in daemons:
        need(any(line == cnf.log_running(d["node"]) for _, line in d["log"]),
             f"canopend node {d['node']} never reported running: {d['log'][-3:]}")
    return end[0]


def cn_states(obs, side, node, t0=None, t1=None):
    """[(t, state byte)] of the node's heartbeat/boot-up frames on one link."""
    return [(t, d[0]) for t, _, d in frames(obs, side, cia301.HEARTBEAT_BASE + node, t0, t1)
            if len(d) == 1]


def cn_nmt_frames(obs, side):
    return [(t, d) for t, _, d in frames(obs, side, cia301.NMT_COB)]


def cn_log_states(daemon, t0=None, t1=None):
    """canopend's NMT log lines (from canopennode_facts.LOG_NMT) after t0, before t1."""
    known = set(cnf.LOG_NMT.values())
    return [line for t, line in daemon["log"]
            if line in known and (t0 is None or t > t0) and (t1 is None or t < t1)]


def cn_after(obs, kind, index, end=None, **fields):
    """The master events of `kind` printed after the cli line at `index` (and before the one at
    `end`)."""
    return [e for e in obs["cli"][index + 1:end] if e.get("event") == kind and
            all(e.get(k) == v for k, v in fields.items())]


def cn_u32(data):
    return int.from_bytes(data[:4], "little")


def cn_pdo_pairs(obs, tpdo, t0, t1):
    """Pairs each TPDO1 frame on vcan_host in (t0, t1) with our master's pdo events of the same
    SYNC (sync_index = SYNCs on vcan_host before the frame): [(frame, [events])], and the events
    our master decoded for a SYNC strictly inside the window that drew no TPDO1 frame (the SYNCs
    at the window's edges may have their TPDO1 just outside it)."""
    by_index = {}
    for e in cn_events(obs, "pdo", node=CN_NODE, cob=CN_TPDO1):
        by_index.setdefault(e["sync_index"], []).append(e)
    pairs = [((t, c, d), by_index.get(cn_host_sync_index(obs, t), [])) for t, c, d in tpdo]
    framed = {cn_host_sync_index(obs, t) for t, _, _ in tpdo}
    k0, k1 = cn_host_sync_index(obs, t0), cn_host_sync_index(obs, t1)
    extra = [e for k, events in by_index.items() if k0 < k < k1 and k not in framed
             for e in events]
    return pairs, extra


def cn_host_sync_index(obs, t):
    """SYNCs on vcan_host before t: the master's sync_index of a frame received at t."""
    return sum(1 for s, _, _ in frames(obs, "host", cia301.SYNC_COB) if s < t)


def cn_sync_alignment(obs):
    """Every SYNC on vcan_host is one `sync` event of the master, in order, within 5 ms."""
    host = [t for t, _, _ in frames(obs, "host", cia301.SYNC_COB)]
    syncs = cn_events(obs, "sync")
    need([e["sync_index"] for e in syncs] == list(range(1, len(syncs) + 1)),
         "the master's sync_index does not count 1, 2, ...")
    need(len(host) == len(syncs), f"{len(host)} SYNCs on vcan_host, {len(syncs)} sync events")
    off = [abs(cn_at(e) - t) for e, t in zip(syncs, host)]
    need(not off or max(off) < 0.005, f"a sync event {max(off or [0]) * 1000:.1f} ms off its frame")
    return host


# ---- N1: NMT states and service gating ----

def cn_nmt_commands(obs):
    """[(cs, cmd index, cmd)] of the harness's NMT console lines, checked against the NMT
    frames our master put on vcan_host and the proxy forwarded to vcan_device: exactly those,
    in order, and no other. Returns [(cs, t on vcan_device, cmd index)]."""
    out = []
    for cs, n in obs["harness"]["nmt"]:
        i, cmd, _ = cn_step(obs, n)
        need(cmd["line"] == f"nmt {CN_NODE} {cs:#x}", f"line {n} is {cmd['line']!r}")
        out.append((cs, i, cmd))
    expected = [bytes([cs, CN_NODE]) for cs, _, _ in out]
    host = cn_nmt_frames(obs, "host")
    device = cn_nmt_frames(obs, "device")
    check([d for _, d in host] == expected, f"NMT frames from our master {[d.hex() for _, d in host]}"
                                             f", asked for {[d.hex() for d in expected]}")
    check([d for _, d in device] == expected, f"NMT frames on vcan_device "
                                               f"{[d.hex() for _, d in device]}")
    return [(cs, t, i) for (cs, i, _), (t, _) in zip(out, device)]


def cn_service_window(obs, t0, t1, state):
    """Service gating in [t0, t1) on vcan_device for the CiA 301 state the node is in: SYNC
    draws TPDO1 only in Operational. Returns the notes."""
    syncs = [t for t, _, _ in frames(obs, "device", cia301.SYNC_COB, t0, t1)]
    need(len(syncs) >= 3, f"{len(syncs)} SYNCs in the {state:#04x} window: no gating observed")
    tpdo = frames(obs, "device", CN_TPDO1, t0, t1)
    host_tpdo = frames(obs, "host", CN_TPDO1, t0, t1)
    if state in cia301.PDO_STATES:
        answered = [s for s, nxt in zip(syncs, syncs[1:])
                    if frames(obs, "device", CN_TPDO1, s, nxt)]
        check(len(answered) == len(syncs) - 1 and len(answered) >= CN_MIN_TPDOS,
              f"Operational: {len(answered)} of {len(syncs) - 1} SYNC periods drew TPDO1")
        bad = [d.hex() for _, _, d in tpdo if len(d) != 8]
        check(not bad, f"Operational TPDO1 with DLC other than 8: {bad[:3]}")
        pairs, extra = cn_pdo_pairs(obs, host_tpdo, t0, t1)
        missing = [hexes([f]) for f, events in pairs if len(events) != 1]
        check(not missing, f"TPDO1 our master did not decode once: {missing[:3]}")
        check(not extra, f"pdo events with no TPDO1 frame: {extra[:3]}")
        return f"{len(tpdo)} TPDO1 for {len(syncs)} SYNCs"
    check(not tpdo and not host_tpdo, f"TPDO1 in state {state:#04x}: {hexes(tpdo[:3])}")
    _, extra = cn_pdo_pairs(obs, [], t0, t1)
    check(not extra, f"pdo events with no TPDO1 frame in state {state:#04x}: {extra[:3]}")
    return f"no TPDO1 for {len(syncs)} SYNCs"


def judge_n1(obs):
    cn_complete(obs)
    cn_sync_alignment(obs)
    commands = cn_nmt_commands(obs)
    need([cs for cs, _, _ in commands] == [cia301.NMT_START, cia301.NMT_STOP,
                                           cia301.NMT_ENTER_PRE_OPERATIONAL,
                                           cia301.NMT_RESET_COMMUNICATION, cia301.NMT_RESET_NODE],
         f"the harness sent {[cs for cs, _, _ in commands]}")
    daemons = obs["canopend"]
    need(len(daemons) == 2, f"{len(daemons)} canopend processes (Reset Node needs a restart)")
    first, second = daemons
    reset_node = obs["harness"]["reset_node"]
    need(second["started"] > first["exited"] if first["exited"] else False,
         f"the restart did not follow the exit: {first['exited']} {second['started']}")
    for n in obs["harness"]["config"]:
        need(cn_sdo(obs, n)["event"] == "sdo_ok", f"TPDO1 configuration line {cn_line(obs, n)!r}")
    end = obs["device"][-1][0]
    bounds = [t for _, t, _ in commands] + [end]
    notes = []

    # Before Start: Pre-operational after boot-up, SDO served, no TPDO at SYNC.
    t_boot = [t for t, s in cn_states(obs, "device", CN_NODE) if s == cia301.STATE_BOOT_UP]
    need(t_boot and t_boot[0] < bounds[0], "no boot-up before the first NMT command")
    pre = cn_states(obs, "device", CN_NODE, t_boot[0], bounds[0])
    check(pre and {s for _, s in pre} == {cia301.STATE_PRE_OPERATIONAL},
          f"heartbeat before Start {[hex(s) for _, s in pre]}")
    windows = [(t_boot[0] + CN_SETTLE_S, bounds[0], cia301.STATE_PRE_OPERATIONAL)]

    for k, (cs, t, i) in enumerate(commands):
        t_next = bounds[k + 1]
        if cs in cia301.RESETS:
            continue
        want = cia301.STATE_AFTER[cs]
        beats = cn_states(obs, "device", CN_NODE, t + CN_SETTLE_S, t_next)
        check(beats and {s for _, s in beats} == {want},
              f"after NMT {cs:#04x} canopend's heartbeat is {[hex(s) for _, s in beats]}, "
              f"CiA 301 says {want:#04x}")
        states = cn_after(obs, "state", i, commands[k + 1][2], node=CN_NODE)
        check(states and states[0]["state"] == want,
              f"after NMT {cs:#04x} our master reports {states[:1]}, the wire {want:#04x}")
        logged = cn_log_states(first, t, t_next)
        check(logged[:1] == [CN_STATE_LOG[want]], f"after NMT {cs:#04x} canopend logged {logged}")
        windows.append((t + CN_SETTLE_S, t_next, want))
        notes.append(f"NMT {cs:#04x}: heartbeat {want:#04x} x{len(beats)}, master and log agree")

    # Reset Communication: the process stays; boot-up, then Pre-operational again.
    cs, t, i = commands[3]
    i_next = commands[4][2]
    seen = cn_states(obs, "device", CN_NODE, t, bounds[4])
    check(seen and seen[0][1] == cia301.STATE_BOOT_UP and
          {s for _, s in seen[1:]} == {cia301.STATE_PRE_OPERATIONAL},
          f"after Reset Communication {[hex(s) for _, s in seen]}")
    check(first["exited"] is None or first["exited"] > bounds[4],
          "canopend ended at Reset Communication")
    check(cn_after(obs, "boot", i, i_next, node=CN_NODE), "our master reported no boot-up after "
                                                          "Reset Communication")
    states = cn_after(obs, "state", i, i_next, node=CN_NODE)
    check(states and states[0]["state"] == cia301.STATE_PRE_OPERATIONAL,
          f"after Reset Communication our master reports {states[:1]}")
    check(cn_log_states(first, t, bounds[4]) == [cnf.LOG_NMT["initializing"],
                                                 cnf.LOG_NMT["pre_operational"]],
          f"canopend logged {cn_log_states(first, t, bounds[4])} at Reset Communication")
    windows.append((seen[0][0] + CN_SETTLE_S, bounds[4], cia301.STATE_PRE_OPERATIONAL))
    notes.append("Reset Communication: boot-up then 0x7f, process kept")

    # Reset Node: canopend exits by itself (pinned code), the harness restarts it, boot-up.
    cs, t, i = commands[4]
    check(first["exit_code"] == cnf.RESET_NODE_EXIT_CODE,
          f"canopend exit {first['exit_code']} at Reset Node, pinned {cnf.RESET_NODE_EXIT_CODE}")
    need(reset_node["exit_code"] == first["exit_code"] and first["exited"] > t,
         f"the harness's exit record {reset_node} does not follow the NMT frame")
    check(first["log"] and first["log"][-1][1] == cnf.LOG_FINISHED,
          f"canopend's last log line {first['log'][-1:]}")
    gap = frames(obs, "device", {CN_HB, CN_SDO_TX, CN_TPDO1, CN_EMCY}, t, second["started"])
    check(not gap, f"node 2 frames between Reset Node and the restart: {hexes(gap[:3])}")
    seen = cn_states(obs, "device", CN_NODE, second["started"])
    check(seen and seen[0][1] == cia301.STATE_BOOT_UP and
          {s for _, s in seen[1:]} == {cia301.STATE_PRE_OPERATIONAL} and len(seen) > 1,
          f"after the restart {[hex(s) for _, s in seen]}")
    check(cn_log_states(second)[:2] == [cnf.LOG_NMT["initializing"],
                                        cnf.LOG_NMT["pre_operational"]],
          f"the restarted canopend logged {cn_log_states(second)}")
    boots = cn_after(obs, "boot", i, node=CN_NODE)
    check(boots and cn_at(boots[0]) >= seen[0][0], "our master reported no boot-up after the "
                                                    "restart")
    states = cn_after(obs, "state", i, node=CN_NODE)
    check(states and states[0]["state"] == cia301.STATE_PRE_OPERATIONAL,
          f"after the restart our master reports {states[:1]}")
    windows.append((seen[0][0] + CN_SETTLE_S, end, cia301.STATE_PRE_OPERATIONAL))
    notes.append(f"Reset Node: exit {first['exit_code']} {(first['exited'] - t) * 1000:.0f} ms "
                 f"after the frame, restart, boot-up "
                 f"{(seen[0][0] - second['started']) * 1000:.0f} ms after it")

    for t0, t1, state in windows:
        notes.append(f"{state:#04x}: " + cn_service_window(obs, t0, t1, state))

    # SDO: answered in Pre-operational and Operational, silent in Stopped.
    for state, n in obs["harness"]["sdo"]:
        _, cmd, _ = cn_step(obs, n)
        win = [w for w in windows if w[0] < cmd["t"] < w[1]]
        need(len(win) == 1 and win[0][2] == state, f"SDO line {n} is not in a {state:#04x} window")
        reqs, resps, _ = cn_sdo_wire(obs, n, "device")
        need(len(reqs) == 1, f"SDO line {n}: {len(reqs)} requests on vcan_device")
        result = cn_sdo(obs, n)
        if state in cia301.SDO_STATES:
            check(result["event"] == "sdo_ok" and len(resps) == 1,
                  f"in {state:#04x} the SDO got {result['event']}, {len(resps)} responses")
        else:
            check(result["event"] == "sdo_timeout", f"in Stopped our master reports {result}")
            silent = frames(obs, "device", CN_SDO_TX, reqs[0][0], win[0][1]) + \
                frames(obs, "host", CN_SDO_TX, reqs[0][0], win[0][1])
            check(not silent, f"an SDO answer in Stopped: {hexes(silent)}")
            timeout = (CN_SDO_RX, cia301.sdo_abort(CN_NODE, 0x1000, 0, cia301.ABORT_TIMEOUT)[1])
            # The worker sends the abort before it forwards cmd_done, so on vcan_host it is inside
            # the line.
            _, _, host_aborts = cn_sdo_wire(obs, n, "host")
            check([(c, d) for _, c, d in host_aborts] == [timeout],
                  f"our master's abort on vcan_host before cmd_done {hexes(host_aborts)}")
            # The proxy's copy on vcan_device is not ordered against the CLI's print, so it is
            # looked for up to the end of the Stopped window.
            aborts = [f for f in frames(obs, "device", CN_SDO_RX, reqs[0][0], win[0][1])
                      if f[2][:1] and f[2][0] >> 5 == cia301.CS_ABORT]
            check([(c, d) for _, c, d in aborts] == [timeout],
                  f"our master's abort after the timeout {hexes(aborts)}")
            notes.append(f"Stopped: SDO upload of 0x1000 ended sdo_timeout, no 0x{CN_SDO_TX:03X}")
    return notes


# ---- N2: boot-up and heartbeat ----

def judge_n2(obs):
    cn_complete(obs)
    h = obs["harness"]
    daemons = obs["canopend"]
    need(len(daemons) == 1, f"{len(daemons)} canopend processes")
    d = daemons[0]
    host_syncs = cn_sync_alignment(obs)
    need(host_syncs, "no SYNC from our master: its first step is not observable")
    t0 = host_syncs[0]  # The master's first step: its consumer starts there (C7).
    early = frames(obs, "host", {CN_HB, CN_SDO_TX, CN_EMCY}, None, d["started"])
    need(not early, f"node 2 frames before canopend started: {hexes(early[:3])}")
    nmt = cn_nmt_frames(obs, "host")
    check(not nmt, f"our master sent NMT {[x.hex() for _, x in nmt]} (it never may, here)")

    # Phase A: no slave at all -> boot_timeout at 1000 ms, before canopend starts.
    timeouts = cn_events(obs, "boot_timeout", node=CN_NODE)
    check(len(timeouts) == 1, f"{len(timeouts)} boot_timeout events for node 2")
    late = cn_at(timeouts[0]) - t0
    check(CN_BOOT_TIMEOUT_S[0] <= late <= CN_BOOT_TIMEOUT_S[1],
          f"boot_timeout {late * 1000:.0f} ms after the master's first step")
    need(cn_at(timeouts[0]) < d["started"], "canopend started before the boot timeout")

    # Phase B: canopend with the seeded 0x1017, at least three heartbeats, then SIGSTOP.
    check(cn_sdo_value(obs, h["read_1017"]) == cnf.SEEDED_HEARTBEAT_MS,
          f"0x1017 reads {cn_sdo(obs, h['read_1017'])}")
    t_stop, t_cont = h["sigstop"], h["sigcont"]
    beats = cn_states(obs, "host", CN_NODE, d["started"], t_stop)
    need(len(beats) >= CN_MIN_HEARTBEATS, f"{len(beats)} heartbeats before SIGSTOP")
    silent = frames(obs, "device", {CN_HB, CN_SDO_TX, CN_EMCY}, t_stop + 0.120, t_cont)
    need(not silent, f"canopend sent while stopped (SIGSTOP did not take): {hexes(silent[:3])}")
    need(t_cont - t_stop > CN_HB_LOST_S[1] + 0.1, f"stopped only {t_cont - t_stop:.3f} s")
    lost = cn_events(obs, "hb_lost", node=CN_NODE)
    check(len(lost) == 1 and t_stop < cn_at(lost[0]) < t_cont,
          f"hb_lost events {[cn_at(e) - t_stop for e in lost]} s after SIGSTOP")
    last = [t for t, _ in cn_states(obs, "host", CN_NODE, None, cn_at(lost[0]))][-1]
    delay = cn_at(lost[0]) - last
    check(CN_HB_LOST_S[0] <= delay <= CN_HB_LOST_S[1],
          f"hb_lost {delay * 1000:.0f} ms after the last heartbeat")
    # SIGCONT: heartbeat resumes without boot-up, the master reports it, sends nothing.
    resumed = cn_states(obs, "host", CN_NODE, t_cont)
    need(resumed, "no heartbeat after SIGCONT")
    check(resumed[0][1] == cia301.STATE_PRE_OPERATIONAL, f"after SIGCONT {resumed[0]}")
    back = [e for e in cn_events(obs, "hb_resumed", node=CN_NODE) if cn_at(e) > t_cont]
    check(len(back) == 1 and resumed[0][0] <= cn_at(back[0]) <= resumed[0][0] + CN_EVENT_S,
          f"hb_resumed {[cn_at(e) - resumed[0][0] for e in back]} s after the first heartbeat")
    return [f"boot_timeout {late * 1000:.0f} ms after the first SYNC, no NMT",
            f"0x1017 = {cnf.SEEDED_HEARTBEAT_MS} ms, {len(beats)} heartbeats",
            f"hb_lost {delay * 1000:.0f} ms after the last heartbeat "
            f"({(cn_at(lost[0]) - t_stop) * 1000:.0f} ms after SIGSTOP)",
            f"hb_resumed {(cn_at(back[0]) - resumed[0][0]) * 1000:.1f} ms after the first "
            f"heartbeat after SIGCONT, no NMT sent at all"]


# ---- N3: SDO ----

# Abort cases: harness label -> (console verb, index, sub, the code canopend answers).
CN_ABORTS = {
    "unknown_object_upload": ("read", 0x2FFF, 0, cnf.ABORT_NO_OBJECT),
    "unknown_object_download": ("write", 0x2FFF, 0, cnf.ABORT_NO_OBJECT),
    "read_only_download": ("write", 0x1000, 0, cnf.ABORT_READ_ONLY),
    "unknown_subindex": ("read", 0x1018, 5, cnf.ABORT_NO_SUBINDEX),
    "too_short_download": ("write", 0x1017, 0, cnf.ABORT_LENGTH_TOO_LOW),
    "too_long_download": ("write", 0x1017, 0, cnf.ABORT_LENGTH_TOO_HIGH),
}


def cn_line_target(obs, n):
    parts = cn_line(obs, n).split()
    need(len(parts) >= 5 and parts[1] == str(CN_NODE), f"line {n}: {parts}")
    return parts[0], int(parts[2], 0), int(parts[3], 0)


def cn_upload_matches(obs, n, index, sub, value):
    """Line n uploaded (index, sub): our master's sdo_ok value and the response on vcan_host
    (expedited, size indicated, 4 bytes) both equal `value`."""
    check(cn_line_target(obs, n)[1:] == (index, sub), f"line {n} is {cn_line(obs, n)!r}")
    got = cn_sdo_value(obs, n)
    _, resps, _ = cn_sdo_wire(obs, n)
    need(len(resps) == 1, f"line {n}: {len(resps)} responses on vcan_host")
    size = 4 - ((resps[0][2][0] >> 2) & 3)
    wire_value = int.from_bytes(resps[0][2][4:4 + size], "little")
    check(resps[0][2] == cia301.sdo_upload_response(CN_NODE, index, sub, size, wire_value)[1],
          f"line {n}: response {resps[0][2].hex()} is no expedited upload response")
    check(got == wire_value == value, f"{index:#06x}:{sub} reads {got} (wire {wire_value}), "
                                      f"expected {value}")


def judge_n3(obs):
    cn_complete(obs)
    h = obs["harness"]
    cn_upload_matches(obs, h["device_type"], 0x1000, 0, cnf.DEVICE_TYPE)
    cn_upload_matches(obs, h["vendor"], 0x1018, 1, cnf.IDENTITY[1])
    notes = [f"0x1000 = {cnf.DEVICE_TYPE:#x}, 0x1018:01 = {cnf.IDENTITY[1]:#x} "
             f"(the example OD's values)"]

    # 0x1017 rewritten with node 2's consumer disabled (C7).
    for key in ("consumer_off", "consumer_on"):
        cn_step(obs, h[key])
    check(cn_line(obs, h["consumer_off"]) == f"hb {CN_NODE} 0", "the consumer was not disabled")
    _, off, _ = cn_step(obs, h["consumer_off"])
    _, on, _ = cn_step(obs, h["consumer_on"])
    for key, value in (("write_400", 400), ("write_100", 100)):
        _, cmd, _ = cn_step(obs, h[key])
        check(off["t"] < cmd["t"] < on["t"], f"0x1017 written outside the disabled window")
        check(cn_line_target(obs, h[key])[1:] == (0x1017, 0) and
              int(cn_line(obs, h[key]).split()[-1], 0) == value, f"line {cn_line(obs, h[key])!r}")
        check(cn_sdo(obs, h[key])["event"] == "sdo_ok", f"download 0x1017 = {value}: "
                                                         f"{cn_sdo(obs, h[key])}")
    cn_upload_matches(obs, h["read_400"], 0x1017, 0, 400)
    cn_upload_matches(obs, h["read_100"], 0x1017, 0, 100)
    _, w400, _ = cn_step(obs, h["write_400"])
    _, w100, _ = cn_step(obs, h["write_100"])
    beats = [t for t, _ in cn_states(obs, "device", CN_NODE, w400["t"], w100["t"])]
    gaps = [b - a for a, b in zip(beats, beats[1:])]
    need(gaps, "no heartbeat interval observed while 0x1017 was 400")
    check(any(0.3 < g < 0.6 for g in gaps), f"canopend's heartbeat at 0x1017 = 400: {gaps}")
    after = cn_states(obs, "host", CN_NODE, on["t"])
    check(len(after) >= CN_MIN_HEARTBEATS, f"{len(after)} heartbeats after re-enabling")
    lost = cn_events(obs, "hb_lost") + cn_events(obs, "boot_timeout")
    check(not lost, f"heartbeat consumer events {lost}")
    notes.append(f"consumer off, 0x1017 400 (heartbeat every {max(gaps) * 1000:.0f} ms) and back "
                 f"to 100, {len(after)} heartbeats after re-enabling, no hb_lost")

    # Aborts, decoded by our master, pinned to the codes canopend answers.
    for label, (verb, index, sub, code) in CN_ABORTS.items():
        n = h["aborts"][label]
        check(cn_line_target(obs, n) == (verb, index, sub), f"{label}: line {cn_line(obs, n)!r}")
        e = cn_sdo(obs, n)
        check(e["event"] == "sdo_abort" and (e["index"], e["sub"], e["code"]) == (index, sub, code),
              f"{label}: our master reports {e}, canopend's pinned code {code:#010x}")
        _, resps, _ = cn_sdo_wire(obs, n)
        check([d for _, _, d in resps] == [cia301.sdo_abort(CN_NODE, index, sub, code)[1]],
              f"{label}: the wire answer {hexes(resps)}")
        notes.append(f"{label} {index:#06x}:{sub} -> {code:#010x}")
    cn_upload_matches(obs, h["read_after_aborts"], 0x1017, 0, 100)
    return notes


# ---- N4, N6: PDO bytes and lengths, EMCY ----

def cn_pdo_setup(obs):
    """The N4/N6 harness steps: configuration written (all sdo_ok), the loopback values read
    back by SDO, the console line of NMT Start. Returns (v1, v2, t_start on vcan_device)."""
    h = obs["harness"]
    for n in h["config"]:
        e = cn_sdo(obs, n)
        need(e["event"] == "sdo_ok", f"configuration line {cn_line(obs, n)!r}: {e}")
    commands = cn_nmt_commands(obs)
    need([cs for cs, _, _ in commands] == [cia301.NMT_START], f"NMT commands {commands}")
    return commands[0][1]


def cn_tpdo_matches(obs, t0, t1, layout, values):
    """TPDO1 frames on vcan_host in (t0, t1): DLC = the layout's bytes, the bytes the encoding of
    `values` (little-endian, mapping order), and each one decoded by our master as a pdo event of
    the same SYNC with the same fields. Returns the count."""
    nbytes = sum(bits for _, _, bits in layout) // 8
    want = b"".join(v.to_bytes(bits // 8, "little") for v, (_, _, bits) in zip(values, layout))
    tpdo = frames(obs, "host", CN_TPDO1, t0, t1)
    need(len(tpdo) >= CN_MIN_TPDOS, f"{len(tpdo)} TPDO1 frames to judge")
    fields = [{"index": i, "sub": s, "raw": v} for (i, s, _), v in zip(layout, values)]
    pairs, extra = cn_pdo_pairs(obs, tpdo, t0, t1)
    for (t, _, d), same in pairs:
        check(d == want, f"TPDO1 {d.hex()} (DLC {len(d)}), the SDO values encode as {want.hex()} "
                         f"(DLC {nbytes})")
        k = cn_host_sync_index(obs, t)
        check(len(same) == 1, f"TPDO1 after SYNC {k}: our master decoded {len(same)} pdo events")
        check(same[0]["fields"] == fields, f"TPDO1 after SYNC {k}: our master decoded "
                                           f"{same[0]['fields']}, the bytes are {fields}")
    check(not extra, f"pdo events with no TPDO1 frame: {extra[:3]}")
    return len(tpdo)


def cn_rpdos(obs, side="device"):
    return frames(obs, side, CN_RPDO1)


def judge_n4(obs):
    cn_complete(obs)
    cn_sync_alignment(obs)
    h = obs["harness"]
    t_start = cn_pdo_setup(obs)
    layout = [tuple(f) for f in h["tpdo_map"]]
    v1, v2 = cn_sdo_value(obs, h["read_1"]), cn_sdo_value(obs, h["read_2"])
    rpdos = cn_rpdos(obs)
    need(len(rpdos) == 3, f"RPDO1 frames {hexes(rpdos)}: the harness sends three")
    check([len(d) for _, _, d in rpdos] == [2, 1, 2],
          f"RPDO1 DLCs on vcan_device {[len(d) for _, _, d in rpdos]}, sent 2, 1, 2")
    host_rpdos = cn_rpdos(obs, "host")
    check([d for _, _, d in host_rpdos] == [d for _, _, d in rpdos],
          f"RPDO1 from our master {hexes(host_rpdos)}, on vcan_device {hexes(rpdos)}")
    t_rpdo = rpdos[0][0]
    count = cn_tpdo_matches(obs, t_start, t_rpdo, layout, [v1, v2])  # Every TPDO1.
    notes = [f"{count} TPDO1, DLC 8, bytes = {v1:#010x} {v2:#010x} as read by SDO, decoded by "
             f"our master for the same SYNC"]

    # RPDO1 of the 2-byte map (DLC 2), applied on reception (type 255), read back by SDO and seen
    # in the TPDO after.
    rmap = [tuple(f) for f in h["rpdo_map"]]
    need(sum(bits for _, _, bits in rmap) == 16, f"RPDO1 map {rmap} is not 2 bytes")
    applied = int.from_bytes(rpdos[0][2], "little")
    got = cn_sdo_value(obs, h["read_rpdo"])
    check(got == applied, f"RPDO1 {rpdos[0][2].hex()} -> 0x1280:01 reads {got:#x}")
    _, read, _ = cn_step(obs, h["read_rpdo"])
    count = cn_tpdo_matches(obs, read["t"], rpdos[1][0], layout, [applied, v2])
    notes.append(f"RPDO1 DLC 2 on vcan_device applied ({applied:#x} by SDO), then {count} "
                 f"TPDO1 carry it")

    # RPDO1 of DLC 1: canopend's PDO-length EMCY (pinned code and frame).
    emcy = frames(obs, "device", CN_EMCY, rpdos[1][0], rpdos[2][0])
    want = cn_emcy_bytes(cnf.EMCY_PDO_LENGTH_SHORT, 2)
    check([d for _, _, d in emcy] == [want], f"after the DLC 1 RPDO1 canopend sent "
                                             f"{hexes(emcy)}, pinned {want.hex()}")
    beats = cn_states(obs, "device", CN_NODE, rpdos[1][0])
    check(beats and {s for _, s in beats} == {cia301.STATE_OPERATIONAL},
          f"the NMT state changed after the length error: {[hex(s) for _, s in beats]}")
    notes.append(f"RPDO1 DLC 1 -> EMCY {cnf.EMCY_PDO_LENGTH_SHORT:#06x} on 0x{CN_EMCY:03X} "
                 f"{(emcy[0][0] - rpdos[1][0]) * 1000:.1f} ms later, still Operational")
    return notes


def cn_emcy_bytes(code, info):
    """canopend's PDO-length EMCY (canopennode_facts): code, error register, error bit, info."""
    return (code.to_bytes(2, "little") + bytes([cnf.EMCY_ERROR_REGISTER,
                                                cnf.EMCY_ERROR_BIT_RPDO_LENGTH]) +
            info.to_bytes(4, "little"))


def judge_n6(obs):
    cn_complete(obs)
    cn_pdo_setup(obs)
    rpdos = cn_rpdos(obs)
    need([len(d) for _, _, d in rpdos] == [2, 1, 2], f"RPDO1 frames {hexes(rpdos)}")
    wire = frames(obs, "host", CN_EMCY)
    need(wire == [(t, c, d) for t, c, d in wire if t > rpdos[1][0]],
         f"an EMCY before the DLC 1 RPDO1: {hexes(wire)}")
    codes = [int.from_bytes(d[:2], "little") for _, _, d in wire]
    need(codes == [cnf.EMCY_PDO_LENGTH_SHORT, cnf.EMCY_RESET_CODE],
         f"EMCY codes on vcan_host {[hex(c) for c in codes]} (Task 0: short, then reset)")
    need(rpdos[1][0] < wire[0][0] < rpdos[2][0] < wire[1][0], "EMCY out of step with the RPDOs")
    decoded = cn_events(obs, "emcy", node=CN_NODE)
    check(len(decoded) == len(wire), f"{len(wire)} EMCY on vcan_host, {len(decoded)} decoded")
    for (t, _, d), e in zip(wire, decoded):
        code, register = int.from_bytes(d[:2], "little"), d[2]
        check((e["code"], e["register"]) == (code, register),
              f"EMCY {d.hex()}: our master decoded code {e['code']:#06x} register "
              f"{e['register']:#04x}")
        check(t <= cn_at(e) <= t + CN_EVENT_S, f"EMCY decoded {(cn_at(e) - t) * 1000:.1f} ms "
                                               f"after the frame")
    return [f"EMCY {cnf.EMCY_PDO_LENGTH_SHORT:#06x} (register {wire[0][2][2]:#04x}) decoded "
            f"{(cn_at(decoded[0]) - wire[0][0]) * 1000:.1f} ms after the frame",
            f"error reset {cnf.EMCY_RESET_CODE:#06x} after the next RPDO1 of the right length "
            f"decoded {(cn_at(decoded[1]) - wire[1][0]) * 1000:.1f} ms after the frame"]


# ---- N5: SYNC application and sampling order ----

def cn_sync_step(obs, n, host_syncs):
    """The SYNC a `sync step` line produced: (sync_index, its time on vcan_host)."""
    i, cmd, _ = cn_step(obs, n)
    need(cmd["line"] == "sync step", f"line {n} is {cmd['line']!r}")
    syncs = cn_after(obs, "sync", i)
    need(syncs, f"no SYNC after the step of line {n}")
    k = syncs[0]["sync_index"]
    return k, host_syncs[k - 1]


def cn_tpdo_after(obs, k, host_syncs):
    """(TPDO1 value on vcan_host, our master's decoded value) for the SYNC of index k."""
    t1 = host_syncs[k] if k < len(host_syncs) else None
    tpdo = frames(obs, "host", CN_TPDO1, host_syncs[k - 1], t1)
    need(len(tpdo) == 1 and len(tpdo[0][2]) == 4, f"TPDO1 after SYNC {k}: {hexes(tpdo)}")
    pdo = [e for e in cn_events(obs, "pdo", node=CN_NODE, cob=CN_TPDO1) if e["sync_index"] == k]
    need(len(pdo) == 1 and len(pdo[0]["fields"]) == 1 and
         (pdo[0]["fields"][0]["index"], pdo[0]["fields"][0]["sub"]) == CN_LOOP,
         f"pdo events of SYNC {k}: {pdo}")
    return cn_u32(tpdo[0][2]), pdo[0]["fields"][0]["raw"]


def judge_n5(obs):
    # Every SYNC applies the RPDO received before it (SYNC_RPDO_SEQUENCE_AFTER_EACH_SYNC is 1..8).
    need(cnf.SYNC_RPDO_SEQUENCE_AFTER_EACH_SYNC ==
         tuple(range(1, len(cnf.SYNC_RPDO_SEQUENCE_AFTER_EACH_SYNC) + 1)),
         f"the pinned build does not apply an RPDO at every SYNC "
         f"{cnf.SYNC_RPDO_SEQUENCE_AFTER_EACH_SYNC}")
    cn_complete(obs)
    h = obs["harness"]
    host_syncs = cn_sync_alignment(obs)
    daemons = obs["canopend"]
    need(len(daemons) == 1, f"{len(daemons)} canopend processes")
    boots = [t for t, s in cn_states(obs, "device", CN_NODE) if s == cia301.STATE_BOOT_UP]
    need(len(boots) == 1, f"{len(boots)} boot-ups of node 2")
    device_syncs = [t for t, _, _ in frames(obs, "device", cia301.SYNC_COB, boots[0])]
    for n in h["config"]:
        need(cn_sdo(obs, n)["event"] == "sdo_ok", f"configuration line {cn_line(obs, n)!r}")
    commands = cn_nmt_commands(obs)
    need([cs for cs, _, _ in commands] == [cia301.NMT_START], f"NMT commands {commands}")
    old = cn_sdo_value(obs, h["initial"])
    rpdos = frames(obs, "device", CN_RPDO1)
    values = h["values"]
    ks = [v["k"] for v in values]
    need(len(set(ks)) == len(ks) and all(0 < k <= 0x7FF for k in ks), f"values {ks}")
    for v in values:
        k = v["k"]
        value = cnf.LOOPBACK_DISABLED | k
        for n in v["pads"]:
            idx, _ = cn_sync_step(obs, n, host_syncs)
            check(cn_tpdo_after(obs, idx, host_syncs) == (old, old),
                  f"TPDO1 after a SYNC with no RPDO pending is not the old {old:#x}")
        _, cmd, _ = cn_step(obs, v["rpdo"])
        reqs, _, _ = cn_sdo_wire(obs, v["before"], "device")
        need(len(reqs) == 1, f"k = {k}: {len(reqs)} SDO requests of the read before the SYNC")
        # The RPDO, then the read's request: the proxy forwards in order, so a read that
        # follows the RPDO on vcan_device sees it if canopend applied it on reception.
        mine = [r for r in rpdos if cmd["t"] < r[0] < reqs[0][0]]
        need(len(mine) == 1 and mine[0][2] == value.to_bytes(4, "little"),
             f"k = {k}: RPDO1 before the read on vcan_device {hexes(mine)}")
        idx, t_sync = cn_sync_step(obs, v["step"], host_syncs)
        on_device = [t for t in device_syncs if t > mine[0][0]]
        need(on_device and on_device[0] > reqs[0][0] and abs(on_device[0] - t_sync) < 0.005,
             f"k = {k}: the step's SYNC is not the first on vcan_device after the read")
        before = cn_sdo_value(obs, v["before"])
        check(before != value, f"k = {k} visible by SDO before the SYNC after its RPDO")
        check(before == old, f"k = {k}: 0x1280:01 reads {before:#x} before the SYNC, "
                             f"the old value is {old:#x}")
        after = cn_sdo_value(obs, v["after"])
        tpdo, decoded = cn_tpdo_after(obs, idx, host_syncs)
        check(after == value, f"k = {k}: after the SYNC after its RPDO 0x1280:01 reads "
                              f"{after:#x}, not {value:#x}: every SYNC applies the RPDO before it")
        check(tpdo == value, f"k = {k}: TPDO1 after SYNC {idx} carries {tpdo:#x}, not {value:#x}")
        check(decoded == value, f"k = {k}: our master decoded {decoded:#x} after SYNC {idx}, "
                                f"not {value:#x}")
        old = value
    check(len(values) >= CN_N5_VALUES, f"{len(values)} values; at least {CN_N5_VALUES} needed")
    pads = sum(len(v["pads"]) for v in values)
    return [f"{len(values)} values (RPDO type {h['rpdo_type']}), each invisible by SDO before the "
            f"SYNC after its RPDO and visible after that SYNC, by SDO and in the TPDO our master "
            f"decoded",
            f"{pads} SYNCs with no RPDO pending left the TPDO at the old value"]


# ---- N7: capability refusal ----

def judge_n7(obs):
    # Either end is a completed run: a master that activates anyway ends with scenario_done and
    # exit 0, which is a FAIL of the row, not a missing precondition.
    end = cn_complete(obs, "activate_hal", ("scenario_failed", "scenario_done"))
    daemons = obs["canopend"]
    need(sorted(d["node"] for d in daemons) == list(CN_SESSION_NODES),
         f"canopend nodes {[d['node'] for d in daemons]}")
    for node in CN_SESSION_NODES:
        need(cn_states(obs, "host", node), f"no heartbeat from node {node}")
    check(end["event"] == "scenario_failed", f"activate_hal ended with {end['event']}: the "
                                             f"master activated")
    check(obs["harness"]["cli_rc"] == 1, f"the CLI exited {obs['harness']['cli_rc']}")
    steps = {e["line"]: e["n"] for e in cn_events(obs, "cmd")}
    need(set(steps) == {"configure", "activate"}, f"scenario steps {steps}")
    configure = cn_events(obs, "cmd_done", n=steps["configure"])
    need(configure and configure[0]["ok"], f"configure failed: {configure}")
    probes = cn_events(obs, "sdo_abort", index=0x2012)
    check(sorted(e["node"] for e in probes) == list(CN_SESSION_NODES) and
          {e["code"] for e in probes} == {cnf.ABORT_NO_OBJECT},
          f"0x2012 probes {[(e['node'], hex(e['code'])) for e in probes]}")
    for node in CN_SESSION_NODES:
        answers = frames(obs, "host", cia301.SDO_TX_BASE + node)
        check(cia301.sdo_abort(node, 0x2012, 0, cnf.ABORT_NO_OBJECT)[1] in
              [d for _, _, d in answers], f"node {node}: no 0x2012 abort on the wire")
    activate = cn_events(obs, "cmd_done", n=steps["activate"])
    check(len(activate) == 1 and not activate[0]["ok"], f"activation was not refused: {activate}")
    missing = {m["capability"]: m["support"] for m in activate[0].get("missing", [])}
    check(missing == CN_GENERIC_MISSING and len(activate[0]["missing"]) == 5,
          f"activation refused listing {missing}, generic_cia402_profile lacks "
          f"{CN_GENERIC_MISSING}")
    t_refused = activate[0]["t"]
    for side in ("host", "device"):
        starts = [d.hex() for _, d in cn_nmt_frames(obs, side) if d[:1] == bytes([cia301.NMT_START])]
        check(not starts, f"NMT Start on vcan_{side}: {starts}")
        rpdos = [f for f in frames(obs, side)
                 if any(base < f[1] <= base + 0x7F for base in cia301.RPDO_BASE.values())]
        check(not rpdos, f"RPDO on vcan_{side}: {hexes(rpdos[:3])}")
    tail = obs["host"][-1][0] - t_refused
    need(tail >= 0.2, f"only {tail * 1000:.0f} ms observed after the refusal")
    return [f"0x2012 aborted {cnf.ABORT_NO_OBJECT:#010x} on all six nodes, configure ok",
            "activation refused: " + ", ".join(f"{c.lower()} {s.lower()}"
                                               for c, s in CN_GENERIC_MISSING.items()),
            f"no NMT Start and no RPDO, {tail * 1000:.0f} ms observed after the refusal "
            f"({end['reason'][:40]}...)"]


JUDGES.update({"N1": judge_n1, "N2": judge_n2, "N3": judge_n3, "N4": judge_n4, "N5": judge_n5,
               "N6": judge_n6, "N7": judge_n7})
