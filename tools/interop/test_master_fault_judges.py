# SPDX-License-Identifier: Apache-2.0
"""The master-side fault judges (X1-X7) on recorded observations, without vcan.

tools/interop/recorded/x1..x7.json.gz are observations of passing runs of test_master_faults.py
(written with INTEROP_RECORD_DIR). Each recording passes its judge unchanged (the control). Each
edit below breaks one thing a row asserts, and the judge must say so: a broken pass condition is
FAIL, a missing precondition INCONCLUSIVE, never PASS.

x7_edge.json.gz is a kept observation of X7 that the judge once got wrong (#38): its last delayed
RPDO2 left vcan_host 1.847 ms before the window's end, inside the EDGE_S slack.
"""
import copy
import gzip
import json
import pathlib

import pytest

import judges
import judges_master as jm

RECORDED = pathlib.Path(__file__).parent / "recorded"
ROWS = ("X1", "X2", "X3", "X4", "X5", "X6", "X7")
STREAM_ROWS = ("X1", "X2", "X3", "X4", "X6", "X7")
_CACHE = {}


def load(row):
    if row not in _CACHE:
        with gzip.open(RECORDED / f"{row.lower()}.json.gz", "rt", encoding="utf-8") as f:
            _CACHE[row] = json.load(f)
    return copy.deepcopy(_CACHE[row])


def expect(obs, status, text):
    got, why = jm.verdict(obs)
    assert got == status and text in why, f"{got}: {why}"


def cli_events(obs, kind):
    return [e for e in obs["cli"] if e["event"] == kind]


def onset(obs):
    rules = jm.parse_rules(obs["harness"]["rules"])
    return min(f[0] for m in jm.replay(obs, rules, jm.EDGE_S) for f in m)


def bank(obs, kind, node):
    return [r for r in obs["bank"] if r.get("event") == "drive" and r["node"] == node and
            r["kind"] == kind]


def record(kind, t, node, **fields):
    r = {"event": "drive", "t_ns": int(t * 1e9), "node": node, "kind": kind, "seq": 0, "tag": 0,
         "drop": 0, "count": 0, "reason": 0, "fault_code": 0}
    r.update(fields)
    return r


def add_bank(obs, r):
    obs["bank"].append(r)
    obs["bank"].sort(key=lambda x: x.get("t_ns", 0))


@pytest.mark.parametrize("row", ROWS)
def test_recorded_observation_passes(row):
    status, text = jm.verdict(load(row))
    assert status == "PASS", f"{row} {status}: {text}"


@pytest.mark.parametrize("row", ROWS)
def test_rule_match_count_off_by_one_is_inconclusive(row):
    obs = load(row)
    obs["proxy_exit"]["rules"][0]["matched"] += 1
    expect(obs, "INCONCLUSIVE", "match counts")


@pytest.mark.parametrize("row", ROWS)
def test_window_that_matched_nothing_is_inconclusive(row):
    obs = load(row)
    rules = jm.parse_rules(obs["harness"]["rules"])
    obs["harness"]["rules"] = obs["harness"]["rules"].replace(
        f"window {int(rules[0]['from_s'] * 1000)} {int(rules[0]['to_s'] * 1000)}",
        "window 9000 9100")
    for r in obs["proxy_exit"]["rules"]:
        r["matched"] = 0
    expect(obs, "INCONCLUSIVE", "the window matched nothing")


@pytest.mark.parametrize("row", ROWS)
def test_trace_overflow_is_inconclusive(row):
    obs = load(row)
    next(r for r in reversed(obs["bank"]) if "trace_overflow" in r)["trace_overflow"] = 1
    expect(obs, "INCONCLUSIVE", "trace_overflow")


@pytest.mark.parametrize("row", ROWS)
def test_bank_overload_is_inconclusive(row):
    # The record vdrive_bank writes when a wake finds three releases skipped (spec 10.2).
    obs = load(row)
    t = [r["t_ns"] for r in obs["bank"] if "t_ns" in r]
    add_bank(obs, {"event": "overload", "t_ns": t[len(t) // 2],
                   "consecutive_skipped": 3, "error_code": 0x6100})
    expect(obs, "INCONCLUSIVE", "vdrive_bank overload")


@pytest.mark.parametrize("row", ROWS)
def test_bank_overload_is_named_over_the_proxy_match_counts(row):
    # The rule checks run before complete(), and a stall that overloads the bank also delays the
    # proxy, so the overload is named first.
    obs = load(row)
    t = [r["t_ns"] for r in obs["bank"] if "t_ns" in r]
    add_bank(obs, {"event": "overload", "t_ns": t[len(t) // 2],
                   "consecutive_skipped": 3, "error_code": 0x6100})
    obs["proxy_exit"]["rules"][0]["matched"] += 1
    expect(obs, "INCONCLUSIVE", "vdrive_bank overload")


@pytest.mark.parametrize("row", ROWS)
def test_cli_that_lost_events_is_inconclusive(row):
    obs = load(row)
    cli_events(obs, "exit")[0]["event_overflows"] = 1
    expect(obs, "INCONCLUSIVE", "lost events")


@pytest.mark.parametrize("row", STREAM_ROWS)
def test_onset_before_the_arm_completed_is_inconclusive(row):
    obs = load(row)
    done = next(e for e in cli_events(obs, "cmd_done") if e["n"] == 3)
    done["t"] = onset(obs) + 0.01
    expect(obs, "INCONCLUSIVE", "precedes the arm's completion")


@pytest.mark.parametrize("row", STREAM_ROWS)
def test_short_baseline_is_inconclusive(row):
    obs = load(row)
    t = onset(obs)
    before = [e for e in obs["cli"] if e["event"] == "dispatch" and e["approved"] and
              e["at_ns"] / 1e9 < t - 0.02]
    assert len(before) > 60, "the recording has too short a baseline to cut"
    drop = {id(e) for e in before[:-40]}
    obs["cli"] = [e for e in obs["cli"] if id(e) not in drop]
    expect(obs, "INCONCLUSIVE", "moving baseline")


@pytest.mark.parametrize("row", STREAM_ROWS)
def test_baseline_command_not_actuated_on_all_six_is_inconclusive(row):
    obs = load(row)
    r = bank(obs, "actuated", 5)[20]
    obs["bank"].remove(r)
    expect(obs, "INCONCLUSIVE", "moving baseline")


@pytest.mark.parametrize("row", STREAM_ROWS)
def test_stop_before_the_onset_is_inconclusive(row):
    obs = load(row)
    obs["cli"].append({"t": onset(obs) - 0.1, "event": "stop", "stop": ["STOPPING"] * 6})
    expect(obs, "INCONCLUSIVE", "before the onset")


@pytest.mark.parametrize("row", ("X1", "X2", "X3", "X4", "X6"))
def test_dispatch_after_the_inhibit_fails(row):
    obs = load(row)
    t = next(e for e in obs["cli"] if e["event"] == "stop" and "STOPPING" in e["stop"])["t"]
    obs["cli"].append({"t": t + 0.02, "event": "dispatch", "at_ns": int((t + 0.02) * 1e9),
                       "seq": 9999, "approved": True, "source_seq": 9999})
    expect(obs, "FAIL", "dispatched after the inhibit")


@pytest.mark.parametrize("row", ("X1", "X2", "X3", "X4", "X6"))
def test_actuation_after_a_stop_fails(row):
    obs = load(row)
    last = bank(obs, "actuated", 2)[-1]
    add_bank(obs, record("actuated", jm.stop_times(obs, 2, onset(obs)) + 0.05, 2, seq=last["seq"] + 1,
                         tag=last["tag"]))
    expect(obs, "FAIL", "after it stopped")


@pytest.mark.parametrize("row", ("X1", "X2", "X3", "X4", "X6"))
def test_unconfirmed_stop_fails(row):
    obs = load(row)
    last = [e for e in obs["cli"] if e["event"] == "stop"][-1]
    last["stop"][4] = "UNCONFIRMED"
    expect(obs, "FAIL", "not all six confirmed")


@pytest.mark.parametrize("row", ("X1", "X2", "X3", "X4", "X6"))
def test_inhibit_never_latched_fails(row):
    obs = load(row)
    for e in cli_events(obs, "faults"):
        e["faults"] = [f for f in e["faults"] if f != "INHIBITED"]
    expect(obs, "FAIL", "kInhibited never raised")


@pytest.mark.parametrize("row", ("X1", "X2"))
def test_feedback_never_stale_fails(row):
    obs = load(row)
    obs["cli"] = [e for e in obs["cli"] if not (e["event"] == "feedback" and not e["fresh"] and
                                                 e["t"] > onset(obs))]
    expect(obs, "FAIL", "never went stale")


@pytest.mark.parametrize("row", ("X1", "X2"))
def test_feedback_that_never_returned_is_inconclusive(row):
    obs = load(row)
    t = onset(obs)
    obs["cli"] = [e for e in obs["cli"] if not (e["event"] == "feedback" and e["fresh"] and
                                                 e["t"] > t)]
    expect(obs, "INCONCLUSIVE", "never returned")


@pytest.mark.parametrize("row", ("X1", "X2"))
def test_other_event_before_the_latch_fails(row):
    obs = load(row)
    t = [e for e in obs["cli"] if e["event"] == "faults" and "INHIBITED" in e["faults"]][0]["t"]
    obs["cli"].append({"t": t - 0.001, "event": "state", "at_ns": 1, "node": 3, "state": 127})
    expect(obs, "FAIL", "events besides the feedback age")


def test_x1_outage_longer_than_the_lease_is_inconclusive():
    obs = load("X1")
    obs["harness"]["rules"] = obs["harness"]["rules"].replace("2335", "2349")
    rules = jm.parse_rules(obs["harness"]["rules"])
    for r, m in zip(obs["proxy_exit"]["rules"], jm.replay(obs, rules, 0)):
        r["matched"] = len(m)
    expect(obs, "INCONCLUSIVE", "X1: the outage is")


def test_x2_dispatch_after_the_feedback_returned_fails():
    obs = load("X2")
    fresh = [e for e in obs["cli"] if e["event"] == "feedback" and e["fresh"]][-1]
    obs["cli"].append({"t": fresh["t"] + 0.01, "event": "dispatch",
                       "at_ns": int((fresh["t"] + 0.01) * 1e9), "seq": 9999, "approved": True,
                       "source_seq": 9999})
    status, why = jm.verdict(obs)
    assert status == "FAIL" and "dispatched after" in why, why


def test_x2_inhibit_latched_late_fails():
    obs = load("X2")
    for e in obs["cli"]:
        if e["event"] in ("faults", "verdict", "stop") and e["t"] > onset(obs) + 0.015:
            e["t"] += 0.05
    expect(obs, "FAIL", "after the onset (bound")


@pytest.mark.parametrize("row", STREAM_ROWS)
def test_refused_write_before_the_onset_fails(row):
    obs = load(row)
    first = [e for e in obs["cli"] if e["event"] == "verdict"][0]
    obs["cli"].append({"t": onset(obs) - 0.1, "event": "verdict", "seq": first["seq"] + 30,
                       "verdict": "FEEDBACK_STALE", "age_ns": 12_000_000})
    expect(obs, "FAIL", "before the onset")


def test_x2_inhibit_cleared_after_recovery_fails():
    obs = load("X2")
    last = [e for e in obs["cli"] if e["event"] == "feedback"][-1]
    obs["cli"].append({"t": last["t"] + 0.05, "event": "faults", "bits": 0, "faults": []})
    expect(obs, "FAIL", "inhibit was cleared")


def test_x2_hardware_fault_fails():
    obs = load("X2")
    e = [e for e in obs["cli"] if e["event"] == "faults"][-1]
    e["faults"] = e["faults"] + ["NODE_LOST"]
    expect(obs, "FAIL", "hardware-level faults")


def test_x2_drive_leaving_quick_stop_fails():
    obs = load("X2")
    t = jm.parse_rules(obs["harness"]["rules"])[0]["to_s"] + jm.epoch(obs) + 0.05
    for i, (ts, c, h) in enumerate(obs["host"]):
        if c == 0x183 and ts > t:
            obs["host"][i][2] = "2700" + h[4:]  # Operation Enabled
    expect(obs, "FAIL", "left Quick Stop Active")


def test_x2_resumed_samples_missing_fails():
    obs = load("X2")
    t = jm.parse_rules(obs["harness"]["rules"])[0]["to_s"] + jm.epoch(obs) + jm.EDGE_S
    obs["host"] = [f for f in obs["host"] if not (f[1] == 0x183 and f[0] > t)]
    expect(obs, "FAIL", "TPDO1 of node 3 reached vcan_host")


def test_x3_node3_actuating_a_dropped_seq_fails():
    obs = load("X3")
    last = bank(obs, "actuated", 3)[-1]
    add_bank(obs, record("actuated", onset(obs) + 0.001, 3, seq=last["seq"] + 1,
                         tag=last["tag"]))
    expect(obs, "FAIL", "whose RPDO2 was dropped")


def test_x3_node3_without_its_own_stop_fails():
    obs = load("X3")
    for r in bank(obs, "quick_stop", 3):
        obs["bank"].remove(r)
    expect(obs, "FAIL", "node 3's first stop")


def test_x3_stop_for_the_wrong_reason_fails():
    obs = load("X3")
    bank(obs, "quick_stop", 3)[0]["reason"] = judges.QS_INTEGRITY
    expect(obs, "FAIL", "expected timeout or expiry")


def test_x3_no_missing_half_fails():
    obs = load("X3")
    for r in bank(obs, "dropped", 3):
        if r["drop"] == judges.DROP_MISSING_HALF:
            r["count"] = 0
    expect(obs, "FAIL", "no missing half")


def test_x4_hb_lost_too_early_fails():
    obs = load("X4")
    e = cli_events(obs, "hb_lost")[0]
    e["at_ns"] -= 100_000_000
    expect(obs, "FAIL", "after the last heartbeat")


def test_x4_hb_lost_too_late_fails():
    obs = load("X4")
    cli_events(obs, "hb_lost")[0]["at_ns"] += 200_000_000
    expect(obs, "FAIL", "after the last heartbeat")


def test_x4_nmt_frame_after_the_arm_fails():
    obs = load("X4")
    t = onset(obs) + 0.2
    obs["host"].append([t, 0, "8104"])
    obs["host"].sort(key=lambda f: f[0])
    expect(obs, "FAIL", "NMT frame")


def test_x4_hb_lost_of_another_node_fails():
    obs = load("X4")
    cli_events(obs, "hb_lost")[0]["node"] = 2
    expect(obs, "FAIL", "hb_lost events for nodes")


def test_x4_node_lost_not_raised_fails():
    obs = load("X4")
    for e in cli_events(obs, "faults"):
        e["faults"] = [f for f in e["faults"] if f != "NODE_LOST"]
    expect(obs, "FAIL", "kNodeLost not raised")


def test_x5_enable_operation_sent_fails():
    obs = load("X5")
    t = onset(obs) + 0.1
    obs["host"].append([t, 0x403, "0f00"])
    obs["host"].sort(key=lambda f: f[0])
    expect(obs, "FAIL", "Enable Operation sent")


def test_x5_arm_that_completed_fails():
    obs = load("X5")
    done = next(e for e in cli_events(obs, "cmd_done") if e["n"] == 3)
    done["ok"] = True
    expect(obs, "FAIL", "the arm completed")


def test_x5_no_sdo_timeout_fails():
    obs = load("X5")
    obs["cli"] = [e for e in obs["cli"] if e["event"] != "sdo_timeout"]
    expect(obs, "FAIL", "no SDO timeout")


def test_x5_dropped_answer_before_the_arm_is_inconclusive():
    obs = load("X5")
    arm = next(e for e in cli_events(obs, "cmd") if e["line"] == "arm")
    arm["t"] = onset(obs) + 0.1
    expect(obs, "INCONCLUSIVE", "precedes the start of the arm")


def test_x5_actuation_fails():
    obs = load("X5")
    add_bank(obs, record("actuated", onset(obs) + 0.1, 5, seq=1, tag=1))
    expect(obs, "FAIL", "a drive actuated")


def test_x6_not_valid_drop_instead_of_other_tag_fails():
    obs = load("X6")
    bank(obs, "dropped", 4)[-1]["drop"] = judges.DROP_NOT_VALID
    status, why = jm.verdict(obs)
    assert status == "FAIL" and ("not-valid" in why or "other-tag" in why), why


def test_x6_missing_other_tag_drop_fails():
    obs = load("X6")
    other = [r for r in bank(obs, "dropped", 2) if r["drop"] == judges.DROP_OTHER_TAG]
    other[-1]["count"] -= 1
    expect(obs, "FAIL", "other-tag drops at its SYNC")


def test_x6_drive_without_its_own_stop_fails():
    obs = load("X6")
    obs["bank"].remove(bank(obs, "quick_stop", 6)[0])
    expect(obs, "FAIL", "node 6: first stop")


def test_x6_master_stopping_before_the_drive_fails():
    obs = load("X6")
    r = bank(obs, "quick_stop", 1)[0]
    r["t_ns"] += 100_000_000
    expect(obs, "FAIL", "drive's own stop must come first")


def test_x7_delayed_seq_actuated_fails():
    obs = load("X7")
    matched = jm.replay(obs, jm.parse_rules(obs["harness"]["rules"]), jm.EDGE_S)[0][0]
    seq = judges.decode("RPDO2", matched[2])["command_seq"]
    tag = bank(obs, "actuated", 3)[0]["tag"]
    add_bank(obs, record("actuated", matched[0] + 0.07, 3, seq=seq, tag=tag))
    expect(obs, "FAIL", "delayed seqs actuated")


def test_x7_missing_not_valid_drop_fails():
    obs = load("X7")
    for r in bank(obs, "dropped", 3):
        if r["drop"] == judges.DROP_NOT_VALID:
            r["count"] -= 1
            break
    expect(obs, "FAIL", "not-valid drops")


def test_x7_actuation_with_an_expired_deadline_fails():
    obs = load("X7")
    r = bank(obs, "actuated", 3)[30]
    r["t_ns"] += 100_000_000  # Now past the 48 ms lease of its RPDO2.
    expect(obs, "FAIL", "expired deadline")


def test_x7_too_few_live_arrivals_is_inconclusive():
    obs = load("X7")
    obs["harness"]["rules"] = obs["harness"]["rules"].replace("window 2300 3000",
                                                              "window 2300 2480")
    rules = jm.parse_rules(obs["harness"]["rules"])
    obs["proxy_exit"]["rules"][0]["matched"] = len(jm.replay(obs, rules, 0)[0])
    status, why = jm.verdict(obs)
    assert status == "INCONCLUSIVE" and ("only" in why or "delay" in why), why


def test_x7_stop_while_the_stream_ran_fails():
    obs = load("X7")
    t = onset(obs) + 0.2
    add_bank(obs, record("quick_stop", t, 5, reason=judges.QS_TIMEOUT))
    expect(obs, "FAIL", "a drive stopped while the stream ran")


def test_x7_group_stop_while_the_stream_ran_fails():
    obs = load("X7")
    obs["cli"].append({"t": onset(obs) + 0.2, "event": "stop", "stop": ["STOPPING"] * 6})
    expect(obs, "FAIL", "stopped or faulted while the stream ran")


# ---- frames inside the EDGE_S slack of a window edge (#38) ----

# The rule window takes whole milliseconds and the proxy's clock is not the capture's, so a frame
# this close to an edge is resolved from the wire, not from the window.
INSIDE_S = 0.0015


def place_at_edge(obs, edge, inside=INSIDE_S):
    """Moves one edge of the rules' window so the first ("start") or last ("end") frame the rules
    acted on lies `inside` s inside it, the way a different SYNC phase would.
    The window text takes whole milliseconds, so the epoch moves by the rest (under 0.5 ms).
    The captures, the bank and the proxy's counts stay as recorded, and so does the set of frames
    the rules act on (asserted).
    Returns that frame (t, cob, data)."""
    rules = jm.parse_rules(obs["harness"]["rules"])
    acted = jm.replay(obs, rules, 0)
    frame = sorted((f for m in acted for f in m), key=lambda f: f[0])[0 if edge == "start" else -1]
    since = frame[0] - jm.epoch(obs)
    target = since - inside if edge == "start" else since + inside
    ms = round(target * 1000)
    obs["harness"]["epoch_ns"] += round((target - ms / 1000) * 1e9)
    for r in rules:
        old = f"window {round(r['from_s'] * 1000)} {round(r['to_s'] * 1000)}"
        new = (f"window {ms} {round(r['to_s'] * 1000)}" if edge == "start" else
               f"window {round(r['from_s'] * 1000)} {ms}")
        obs["harness"]["rules"] = obs["harness"]["rules"].replace(old, new)
    moved = jm.replay(obs, jm.parse_rules(obs["harness"]["rules"]), 0)
    assert [len(m) for m in moved] == [len(m) for m in acted], "the move changed the matches"
    since = frame[0] - jm.epoch(obs)
    edge_s = jm.parse_rules(obs["harness"]["rules"])[0]["from_s" if edge == "start" else "to_s"]
    assert abs(abs(since - edge_s) - inside) < 1e-6, f"{since} is not {inside} s from {edge_s}"
    return frame


def destination(obs, frame):
    """The capture a frame's rule forwards it to: vcan_device for h2d, vcan_host for d2h."""
    return "host" if [f for f in judges.source_frames(obs, "d2h") if f[0] == frame[0]] else "device"


def copies(obs, side, frame):
    """The raw capture entries on `side` with the frame's identifier and bytes, after it."""
    return [f for f in obs[side]
            if f[1] == frame[1] and f[2] == frame[2].hex() and f[0] >= frame[0]]


def move(obs, side, entry, t):
    entry[0] = t
    obs[side].sort(key=lambda f: f[0])


def total_matched(obs):
    return sum(r["matched"] for r in obs["proxy_exit"]["rules"])


@pytest.mark.parametrize("edge", ("start", "end"))
@pytest.mark.parametrize("row", ROWS)
def test_frame_inside_the_edge_slack_is_resolved_from_the_wire(row, edge):
    # The proxy acted on the edge frame (its count says so) and the destination capture agrees.
    # The verdict is PASS, and the row's note counts every frame the proxy acted on.
    obs = load(row)
    place_at_edge(obs, edge)
    status, why = jm.verdict(obs)
    assert status == "PASS", f"{status}: {why}"
    assert why.startswith(f"{total_matched(obs)} "), why


def test_x7_delayed_frame_inside_the_edge_slack_passes():
    obs = load("X7")
    place_at_edge(obs, "end")
    expect(obs, "PASS", "18 reached the drive with the tag current = 18 not-valid drops")


def test_x7_delayed_frame_inside_the_start_slack_passes():
    obs = load("X7")
    place_at_edge(obs, "start")
    expect(obs, "PASS", "18 reached the drive with the tag current = 18 not-valid drops")


def on_time_x7_edge_frame():
    """X7 with the last frame forwarded on time: its device copy is 20 us after the host one, its
    not-valid drop is gone, and the proxy counts 17."""
    obs = load("X7")
    frame = place_at_edge(obs, "end")
    held = copies(obs, "device", frame)
    assert len(held) == 1 and held[0][0] - frame[0] > 0.05, held
    t_arrival = held[0][0]
    move(obs, "device", held[0], frame[0] + 20e-6)
    drop = next(r for r in bank(obs, "dropped", 3) if r["drop"] == judges.DROP_NOT_VALID and
                0 <= r["t_ns"] / 1e9 - t_arrival < 0.001)
    assert drop["count"] == 1, drop
    obs["bank"].remove(drop)
    obs["proxy_exit"]["rules"][0]["matched"] = 17
    return obs


def test_x7_on_time_frame_inside_the_edge_slack_passes():
    # The control: the same placement, but the proxy forwarded the last frame on time.
    expect(on_time_x7_edge_frame(), "PASS",
           "17 reached the drive with the tag current = 17 not-valid drops")


def test_x7_on_time_frame_with_the_device_capture_30_us_early_passes():
    # The two captures estimate their clock offsets apart, so a correct copy may stamp earlier.
    obs = on_time_x7_edge_frame()
    for entry in obs["device"]:
        entry[0] -= 30e-6
    expect(obs, "PASS", "17 reached the drive with the tag current = 17 not-valid drops")


def test_x7_proxy_count_disagreeing_with_the_wire_is_inconclusive():
    # The edge frame was held one delay on the wire, but the proxy counts only 17.
    obs = load("X7")
    place_at_edge(obs, "end")
    obs["proxy_exit"]["rules"][0]["matched"] = 17
    expect(obs, "INCONCLUSIVE", "match counts")


def test_x7_edge_frame_neither_on_time_nor_held_is_inconclusive():
    # A copy 30 ms after the edge frame is neither forwarded on time nor held one delay.
    obs = load("X7")
    frame = place_at_edge(obs, "end")
    move(obs, "device", copies(obs, "device", frame)[0], frame[0] + 0.030)
    expect(obs, "INCONCLUSIVE", "edge")


def test_x7_edge_regression_recording_passes():
    # The kept observation's last delayed frame lies inside the slack, the proxy delayed it, and
    # the bank dropped it not-valid: all 18 are counted.
    obs = load("X7_EDGE")
    rule = jm.parse_rules(obs["harness"]["rules"])[0]
    last = jm.replay(obs, [rule], 0)[0][-1][0] - jm.epoch(obs)
    assert 0 < rule["to_s"] - last < jm.EDGE_S, f"{(rule['to_s'] - last) * 1000:.3f} ms"
    expect(obs, "PASS", "18 reached the drive with the tag current = 18 not-valid drops")


@pytest.mark.parametrize("row", ("X1", "X2", "X3", "X4", "X5"))
def test_dropped_edge_frame_found_on_the_destination_is_inconclusive(row):
    # The proxy counts the edge frame as dropped, but its destination capture has it on time.
    # vcan_device is the destination of h2d rules (X3), vcan_host of d2h rules (X1, X2, X4, X5).
    obs = load(row)
    frame = place_at_edge(obs, "end")
    side = destination(obs, frame)
    assert not [f for f in copies(obs, side, frame) if f[0] - frame[0] < 0.004]
    obs[side].append([frame[0] + 20e-6, frame[1], frame[2].hex()])
    obs[side].sort(key=lambda f: f[0])
    expect(obs, "INCONCLUSIVE", "match counts")



# ---- identical frames at a window edge (#38) ----
# The master resends the same RPDO2 bytes, so a destination frame with the edge frame's bytes may
# be the copy of another transmission.

EDGE_T = 10.0


def synthetic(sent, forwarded):
    """An observation whose captures hold identical RPDO2 frames: `sent` are the times on
    vcan_host, `forwarded` the times on vcan_device."""
    data = bytes(range(8)).hex()
    return {"harness": {"epoch_ns": 0},
            "host": [[t, 0x303, data] for t in sent],
            "device": [[t, 0x303, data] for t in forwarded]}


DELAY_60 = {"dir": "h2d", "action": "delay", "args": ["60"]}
EDGE_FRAME = (EDGE_T, 0x303, bytes(range(8)))


def test_edge_frame_delayed_beside_an_identical_frame_forwarded_at_once_is_inconclusive():
    # One transmission 1.5 ms before the edge frame was forwarded at once.
    # The edge frame was delayed 60 ms.
    # The wire fits as well with the two copies swapped.
    # So it does not say which transmission the proxy delayed.
    obs = synthetic([EDGE_T - 0.0015, EDGE_T], [EDGE_T - 0.0015 + 20e-6, EDGE_T + 0.060])
    with pytest.raises(judges.Inconclusive, match="edge frame"):
        jm.acted_at_edge(obs, DELAY_60, EDGE_FRAME)


def test_edge_frame_delayed_before_an_identical_frame_forwarded_at_once_is_inconclusive():
    # The same, the identical transmission 3 ms after the edge frame.
    obs = synthetic([EDGE_T, EDGE_T + 0.003], [EDGE_T + 0.003 + 20e-6, EDGE_T + 0.060])
    with pytest.raises(judges.Inconclusive, match="edge frame"):
        jm.acted_at_edge(obs, DELAY_60, EDGE_FRAME)


def test_dropped_edge_frame_beside_an_identical_frame_forwarded_at_once_is_inconclusive():
    # One copy on the wire for two transmissions 1.5 ms apart: either was the one dropped.
    obs = synthetic([EDGE_T - 0.0015, EDGE_T], [EDGE_T - 0.0015 + 20e-6])
    with pytest.raises(judges.Inconclusive, match="edge frame"):
        jm.acted_at_edge(obs, {"dir": "h2d", "action": "drop", "args": []}, EDGE_FRAME)


def test_edge_frame_delayed_among_identical_frames_far_apart_is_resolved():
    # The control: resends 10 ms apart, each copy has one transmission it can belong to.
    sent = [EDGE_T - 0.010, EDGE_T, EDGE_T + 0.010]
    obs = synthetic(sent, [sent[0] + 20e-6, EDGE_T + 0.060, sent[2] + 20e-6])
    assert jm.acted_at_edge(obs, DELAY_60, EDGE_FRAME) is True
    obs = synthetic(sent, [t + 20e-6 for t in sent])
    assert jm.acted_at_edge(obs, DELAY_60, EDGE_FRAME) is False


def test_on_time_edge_frame_beside_an_identical_frame_sent_one_delay_later_is_resolved():
    # The frame 58 ms later is forwarded at once.
    # Its copy lies in the edge frame's delayed window too.
    # Only the edge frame can own the copy right after it.
    # So the edge frame was not held, and the later one owns the other.
    obs = synthetic([EDGE_T, EDGE_T + 0.058], [EDGE_T + 20e-6, EDGE_T + 0.058 + 20e-6])
    assert jm.acted_at_edge(obs, DELAY_60, EDGE_FRAME) is False


def test_dropped_edge_frame_is_refuted_when_the_other_copies_need_the_other_transmission():
    # The later transmission has its own copy 3.5 ms after it, and each transmission has one copy
    # at most, so the copy right after the edge frame is the edge frame's.
    obs = synthetic([EDGE_T, EDGE_T + 0.001], [EDGE_T + 0.0002, EDGE_T + 0.0045])
    assert jm.acted_at_edge(obs, {"dir": "h2d", "action": "drop", "args": []}, EDGE_FRAME) is False
