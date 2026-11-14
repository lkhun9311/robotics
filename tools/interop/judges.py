# SPDX-License-Identifier: Apache-2.0
"""Verdicts of the interop matrix M1-M6 (E3b Task 6), as functions over one captured observation.

An observation is plain JSON, so a recorded one can be judged again without vcan:
  host, device   frames seen on vcan_host (what Lely saw) and vcan_device (what the drives
                 saw): [t, cob_id, hex data], t = CLOCK_MONOTONIC seconds
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
    out = []
    for t, c, h in obs[side]:
        if (cob is None or c == cob or (isinstance(cob, (set, tuple)) and c in cob)) and \
                (t0 is None or t > t0) and (t1 is None or t < t1):
            out.append((t, c, bytes.fromhex(h)))
    return out


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

def complete(obs, scenario):
    """Traces complete, processes ran, the scenario finished; Inconclusive otherwise."""
    bank = obs.get("bank") or []
    need(bank, "no vdrive_bank trace records")
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
    bad, control = h["bad"], h["control"]
    snaps = records(obs, "dispatch", node=NODE)
    before = [r for r in snaps if r["t_ns"] / 1e9 < bad["t"]]
    after = [r for r in snaps if r["t_ns"] / 1e9 > bad["t"]]
    need(before and after, "no dispatch snapshot on both sides of the DLC 7 frame")
    actuated = records(obs, "drive", node=NODE, kind="actuated")
    need(any(r["seq"] == control["seq"] and r["tag"] == tag for r in actuated),
         f"the DLC 8 control pair (seq {control['seq']}) was not actuated, so the missing "
         f"actuation of the DLC 7 frame would prove nothing")
    delta = after[0]["malformed"] - before[-1]["malformed"]
    check(delta == 1, f"node 3 malformed count rose by {delta} across the DLC 7 RPDO1")
    check(not any(r["seq"] == bad["seq"] for r in actuated),
          f"the DLC 7 RPDO1 (seq {bad['seq']}) was actuated")
    notes.append(f"DLC 7 RPDO1 seq {bad['seq']}: malformed {before[-1]['malformed']} -> "
                 f"{after[0]['malformed']} in the next snapshot, no kActuated; control pair seq "
                 f"{control['seq']} (DLC 8) actuated")
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
