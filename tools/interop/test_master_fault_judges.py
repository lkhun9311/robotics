# SPDX-License-Identifier: Apache-2.0
"""The master-side fault judges (X1-X7) on recorded observations, without vcan.

tools/interop/recorded/x1..x7.json.gz are observations of passing runs of test_master_faults.py
(written with INTEROP_RECORD_DIR). Each recording passes its judge unchanged (the control). Each
edit below breaks one thing a row asserts, and the judge must say so: a broken pass condition is
FAIL, a missing precondition INCONCLUSIVE, never PASS.
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
