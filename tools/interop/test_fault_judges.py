# SPDX-License-Identifier: Apache-2.0
"""The proxy-fault judges (F1-F6, R1) on recorded observations, without vcan.

tools/interop/recorded/f1..f6, r1.json.gz are observations of passing runs of
test_proxy_faults.py (written with INTEROP_RECORD_DIR). Each recording passes its judge unchanged
(the control). Each edit below breaks one thing the row asserts, and the judge must say so: a
broken pass condition is FAIL, a missing precondition INCONCLUSIVE, never PASS.
"""
import copy
import gzip
import json
import pathlib

import pytest

import judges

RECORDED = pathlib.Path(__file__).parent / "recorded"
FAULT_ROWS = ("F1", "F2", "F3", "F4", "F5", "F6", "F7")
ROWS = FAULT_ROWS + ("R1",)
_CACHE = {}


def load(row):
    if row not in _CACHE:
        with gzip.open(RECORDED / f"{row.lower()}.json.gz", "rt", encoding="utf-8") as f:
            _CACHE[row] = json.load(f)
    return copy.deepcopy(_CACHE[row])


def verdict(obs):
    return judges.verdict(obs)


def drive(obs, kind, bank="bank"):
    return [r for r in obs[bank] if r.get("event") == "drive" and r.get("node") == judges.NODE
            and r.get("kind") == kind]


def stream_tag(obs):
    return next(e["tag"] for e in obs["peer"] if e.get("event") == "scenario_done")


def add_record(obs, record, bank="bank"):
    obs[bank].append(record)
    obs[bank].sort(key=lambda r: r.get("t_ns", 0))


def record_like(obs, kind, t_ns, **fields):
    r = {"event": "drive", "t_ns": t_ns, "node": judges.NODE, "kind": kind, "seq": 0, "tag": 0,
         "drop": 0, "count": 0, "reason": 0, "fault_code": 0}
    r.update(fields)
    return r


def mid_time(obs):
    t = [r["t_ns"] for r in obs["bank"] if r.get("event") == "drive"]
    return t[len(t) // 2]


@pytest.mark.parametrize("row", ROWS)
def test_recorded_observation_passes(row):
    status, text = verdict(load(row))
    assert status == "PASS", f"{row} {status}: {text}"


@pytest.mark.parametrize("row", FAULT_ROWS)
def test_rule_match_count_off_by_one_is_inconclusive(row):
    obs = load(row)
    obs["proxy_exit"]["rules"][0]["matched"] += 1
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "match counts" in text, f"{status}: {text}"


@pytest.mark.parametrize("row", ROWS)
def test_trace_overflow_is_inconclusive(row):
    obs = load(row)
    banks = obs["banks"] if row == "R1" else [obs["bank"]]
    last = next(r for r in reversed(banks[-1]) if "trace_overflow" in r)
    last["trace_overflow"] = 1
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "trace_overflow" in text, f"{status}: {text}"


def test_stream_seq_missing_on_host_is_inconclusive():
    obs = load("F1")  # Its rule acts on RPDO2: removing an RPDO1 leaves the match count.
    tag = stream_tag(obs)
    i = next(i for i, (_, c, h) in enumerate(obs["host"])
             if c == 0x203 and bytes.fromhex(h)[6:8] == tag.to_bytes(2, "little") and
             int.from_bytes(bytes.fromhex(h)[4:6], "little") == 30)
    del obs["host"][i]
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "not on vcan_host" in text, f"{status}: {text}"


def test_f1_delayed_seq_actuated_fails():
    obs = load("F1")
    add_record(obs, record_like(obs, "actuated", mid_time(obs), seq=4, tag=stream_tag(obs)))
    status, text = verdict(obs)
    assert status == "FAIL" and "delayed seqs actuated" in text, f"{status}: {text}"


def test_f1_missing_other_tag_drop_fails():
    obs = load("F1")
    other = [r for r in drive(obs, "dropped") if r["drop"] == judges.DROP_OTHER_TAG]
    other[-3]["count"] -= 1
    status, text = verdict(obs)
    assert status == "FAIL" and "other-tag drops at its SYNC" in text, f"{status}: {text}"


def test_f1_stop_reason_integrity_fails():
    obs = load("F1")
    drive(obs, "quick_stop")[0]["reason"] = judges.QS_INTEGRITY
    status, text = verdict(obs)
    assert status == "FAIL" and "timeout or expiry" in text, f"{status}: {text}"


@pytest.mark.parametrize("row", ("F2", "F3", "F4"))
def test_integrity_where_none_is_allowed_fails(row):
    obs = load(row)
    add_record(obs, record_like(obs, "integrity", mid_time(obs)))
    status, text = verdict(obs)
    assert status == "FAIL" and "kIntegrity" in text, f"{status}: {text}"


def test_f2_lost_seq_actuated_fails():
    obs = load("F2")
    add_record(obs, record_like(obs, "actuated", mid_time(obs), seq=4, tag=stream_tag(obs)))
    status, text = verdict(obs)
    assert status == "FAIL" and "actuated minus expected [4]" in text, f"{status}: {text}"


@pytest.mark.parametrize("row", ("F2", "F7"))
def test_gap_row_streamed_with_the_default_validity_is_inconclusive(row):
    obs = load(row)
    for e in obs["peer"]:
        if e.get("event") == "stream":
            e["valid_until"] = (e["valid_until"] - 8000) % 2**32  # now + 40 ms
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "lands on the expiry" in text, f"{status}: {text}"


@pytest.mark.parametrize("row", ("F2", "F7"))
def test_gap_row_enabled_before_the_first_update_is_inconclusive(row):
    obs = load(row)
    tag = stream_tag(obs).to_bytes(2, "little")
    first = next(f[0] for f in obs["host"] if f[1] == 0x203 and
                 bytes.fromhex(f[2])[4:8] == (1).to_bytes(2, "little") + tag)
    for f in obs["host"]:
        if f[1] == 0x403 and f[2].startswith("0f00"):
            f[0] = first - 0.001  # 0x0F sent with the first update, not after it
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "after the first update" in text, f"{status}: {text}"


@pytest.mark.parametrize("row", ("F2", "F7"))
def test_gap_row_validity_just_under_the_bound_is_inconclusive(row):
    obs = load(row)
    for e in obs["peer"]:
        if e.get("event") == "stream":
            e["valid_until"] = (e["valid_until"] - 2500) % 2**32  # now + 45.5 ms
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "lands on the expiry" in text, f"{status}: {text}"


@pytest.mark.parametrize("shift_us", (-50_000, 2_000))  # behind the write; past the 48 ms lease
def test_gap_row_validity_outside_the_lease_is_inconclusive(shift_us):
    obs = load("F2")
    for e in obs["peer"]:
        if e.get("event") == "stream":
            e["valid_until"] = (e["valid_until"] + shift_us) % 2**32
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "lands on the expiry" in text, f"{status}: {text}"


def test_gap_row_first_enable_before_the_first_update_is_inconclusive():
    obs = load("F2")
    tag = stream_tag(obs).to_bytes(2, "little")
    first = next(f[0] for f in obs["host"] if f[1] == 0x203 and
                 bytes.fromhex(f[2])[4:8] == (1).to_bytes(2, "little") + tag)
    enable = next(f for f in obs["host"] if f[1] == 0x403 and f[2].startswith("0f00"))
    obs["host"].append([first - 0.001, enable[1], enable[2]])  # an earlier 0x0F; a later one too
    obs["host"].sort(key=lambda f: f[0])
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "after the first update" in text, f"{status}: {text}"


def held_rpdo2(obs):
    """[(t on vcan_host, data hex)] of the stream-tag RPDO2 frames the delay rule held."""
    tag = stream_tag(obs)
    matched = judges.expected_matches(obs, judges.parse_rules(obs["harness"]["rules"]))[0]
    return [(t, d.hex()) for t, _, d in matched if d[6:8] == tag.to_bytes(2, "little")]


def test_f7_held_frame_forwarded_at_once_is_inconclusive():
    obs = load("F7")
    t_held, h = held_rpdo2(obs)[5]
    copy = next(f for f in obs["device"] if f[1] == 0x303 and f[2] == h and f[0] > t_held + 0.05)
    copy[0] = t_held + 0.0003  # the proxy forwarded it instead of holding it
    obs["device"].sort(key=lambda f: f[0])
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "one delay later" in text, f"{status}: {text}"


def test_f7_held_repeat_of_the_last_seq_judges_by_its_late_copy():
    """Lely repeats seq 60's RPDO2 at every SYNC after the stream; when the rule holds one of
    those repeats, its bytes equal the repeats forwarded at once, one of which arrives within the
    held copy's window. Only the one late copy is the held frame (the y6 run)."""
    obs = load("F7")
    last = [f for f in obs["host"] if f[1] == 0x303 and
            bytes.fromhex(f[2])[4:6] == (60).to_bytes(2, "little") and
            bytes.fromhex(f[2])[6:8] == stream_tag(obs).to_bytes(2, "little")]
    h, t0 = last[0][2], last[0][0]
    on_device = [f for f in obs["device"] if f[1] == 0x303 and f[2] == h]
    lag = on_device[0][0] - t0
    # Repeats 4..11 of seq 60, 10 ms apart, after the 3 recorded: the 4th and 8th are held.
    for i in range(len(last) + 1, 12):
        t = t0 + 0.010 * (i - 1)
        obs["host"].append([t, 0x303, h])
        obs["device"].append([t + (0.060 if i % 4 == 0 else 0) + lag, 0x303, h])
    obs["host"].sort(key=lambda f: f[0])
    obs["device"].sort(key=lambda f: f[0])
    obs["proxy_exit"]["rules"][0]["matched"] += 2
    status, text = verdict(obs)
    assert status == "PASS", f"{status}: {text}"


def test_f7_held_copy_missing_on_device_is_inconclusive():
    obs = load("F7")
    _, i = delayed_stream_rpdo2(obs)[len(delayed_stream_rpdo2(obs)) // 2]
    del obs["device"][i]
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "one delay later" in text, f"{status}: {text}"


def overload_record(t_ns):
    """The record vdrive_bank writes when a wake finds three releases skipped (spec 10.2)."""
    return {"event": "overload", "t_ns": t_ns, "consecutive_skipped": 3, "error_code": 0x6100}


def test_f7_bank_overload_is_inconclusive():
    # CI run 44722382439 attempt 1: the bank overloaded before the peer's 0x06.
    obs = load("F7")
    add_record(obs, overload_record(mid_time(obs)))
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "vdrive_bank overload" in text, f"{status}: {text}"


def test_f7_bank_overload_is_named_over_the_failed_scenario():
    # The peer's failure is the overload's symptom, so the reason names the overload.
    obs = load("F7")
    add_record(obs, overload_record(mid_time(obs)))
    done = next(e for e in obs["peer"] if e.get("event") == "scenario_done")
    obs["peer"][obs["peer"].index(done)] = {
        "t": done["t"], "event": "scenario_failed", "scenario": done["scenario"],
        "reason": "node 3 did not report Ready to Switch On after 0x06"}
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "vdrive_bank overload" in text, f"{status}: {text}"


@pytest.mark.parametrize("bank", (0, -1))
def test_r1_bank_overload_is_inconclusive(bank):
    # R1 judges two bank runs without complete(), so each one is checked on its own.
    obs = load("R1")
    records = obs["banks"][bank]
    t = [r["t_ns"] for r in records if "t_ns" in r]
    records.append(overload_record(t[len(t) // 2]))
    records.sort(key=lambda r: r.get("t_ns", 0))
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "vdrive_bank overload" in text, f"{status}: {text}"


def test_f3_actuated_set_differs_from_no_rule_run_fails():
    obs = load("F3")
    obs["bank"].remove(next(r for r in drive(obs, "actuated") if r["seq"] == 30))
    status, text = verdict(obs)
    assert status == "FAIL" and "no-rule run" in text, f"{status}: {text}"


def test_f3_missing_duplicate_fails():
    obs = load("F3")
    i = next(i for i, (_, c, _) in enumerate(obs["device"]) if c == 0x303)
    del obs["device"][i]
    status, text = verdict(obs)
    assert status in ("FAIL", "INCONCLUSIVE") and "twice" in text, f"{status}: {text}"


def test_f4_unactuated_seq_fails():
    obs = load("F4")
    obs["bank"].remove(next(r for r in drive(obs, "actuated") if r["seq"] == 30))
    status, text = verdict(obs)
    assert status == "FAIL" and "not actuated: [30]" in text, f"{status}: {text}"


def test_f5_stop_reason_other_than_integrity_fails():
    obs = load("F5")
    drive(obs, "quick_stop")[0]["reason"] = judges.QS_TIMEOUT
    status, text = verdict(obs)
    assert status == "FAIL" and "expected integrity first" in text, f"{status}: {text}"


def test_f5_latch_not_set_fails():
    obs = load("F5")
    t_flags = obs["harness"]["t_flags"]
    for f in obs["host"]:
        if f[0] > t_flags and f[1] == 0x583 and f[2][2:8] == "302005":
            f[2] = f[2][:8] + "04" + f[2][10:]  # drive_flags 0x0005 -> 0x0004
    status, text = verdict(obs)
    assert status == "FAIL" and "drive_flags 0x0004" in text, f"{status}: {text}"


def test_f5_seq_after_the_conflict_actuated_fails():
    obs = load("F5")
    add_record(obs, record_like(obs, "actuated", obs["bank"][-1]["t_ns"] - 1, seq=50,
                                tag=stream_tag(obs)))
    status, text = verdict(obs)
    assert status == "FAIL" and "later one was actuated" in text, f"{status}: {text}"


def test_f6_tpdo1_on_host_fails():
    obs = load("F6")
    t = obs["host"][len(obs["host"]) // 2][0]
    obs["host"].append([t, 0x183, "2702000000000100"])
    obs["host"].sort(key=lambda f: f[0])
    status, text = verdict(obs)
    assert status == "FAIL" and "TPDO1 of node 3 on vcan_host" in text, f"{status}: {text}"


def test_r1_old_tag_actuated_fails():
    obs = load("R1")
    old_tag = next(e["tag"] for e in obs["peers"][0] if e.get("event") == "scenario_done")
    t = obs["harness"]["t_second_stream"]
    obs["banks"][-1].append(record_like(obs, "actuated", int(t * 1e9) + 5_000_000, seq=3,
                                        tag=old_tag))
    obs["banks"][-1].sort(key=lambda r: r.get("t_ns", 0))
    status, text = verdict(obs)
    assert status == "FAIL" and "old-tag seqs actuated" in text, f"{status}: {text}"


def test_r1_replayed_frame_not_counted_fails():
    obs = load("R1")
    t_sync, t_next = obs["harness"]["t_sync"], obs["harness"]["t_second_stream"]
    drop = next(r for r in obs["banks"][-1] if r.get("event") == "drive" and
                r.get("kind") == "dropped" and t_sync < r["t_ns"] / 1e9 < t_next)
    drop["count"] -= 1
    status, text = verdict(obs)
    assert status == "FAIL" and "other-tag drops (expected" in text, f"{status}: {text}"


def test_r1_same_tag_after_restart_fails():
    obs = load("R1")
    old_tag = next(e["tag"] for e in obs["peers"][0] if e.get("event") == "scenario_done")
    obs["harness"]["new_tag"] = old_tag
    for e in obs["peers"][-2]:
        if e.get("event") == "sdo_ok" and e.get("index") == 0x2012:
            e["value"] = old_tag
    status, text = verdict(obs)
    assert status == "FAIL" and "committed the old tag" in text, f"{status}: {text}"


def delayed_stream_rpdo2(obs):
    """(seq, index in obs['device']) of the stream-tag RPDO2 frames the delay rule matched, in
    device order."""
    tag = stream_tag(obs)
    matched = judges.expected_matches(obs, judges.parse_rules(obs["harness"]["rules"]))[0]
    wanted = {d.hex() for _, _, d in matched}
    return [(int.from_bytes(bytes.fromhex(h)[4:6], "little"), i)
            for i, (_, c, h) in enumerate(obs["device"])
            if c == 0x303 and h in wanted and bytes.fromhex(h)[6:8] == tag.to_bytes(2, "little")]


def test_f7_delayed_seq_actuated_fails():
    obs = load("F7")
    seq = delayed_stream_rpdo2(obs)[0][0]
    add_record(obs, record_like(obs, "actuated", mid_time(obs), seq=seq, tag=stream_tag(obs)))
    status, text = verdict(obs)
    assert status == "FAIL" and "delayed seqs actuated" in text, f"{status}: {text}"


def test_f7_missing_not_valid_drop_fails():
    obs = load("F7")
    nv = [r for r in drive(obs, "dropped") if r["drop"] == judges.DROP_NOT_VALID]
    obs["bank"].remove(nv[len(nv) // 2])
    status, text = verdict(obs)
    assert status == "FAIL" and "not-valid" in text, f"{status}: {text}"


def test_f7_non_delayed_seq_not_actuated_fails():
    obs = load("F7")
    delayed = {s for s, _ in delayed_stream_rpdo2(obs)}
    seq = next(s for s in range(10, 50) if s not in delayed)
    obs["bank"].remove(next(r for r in drive(obs, "actuated") if r["seq"] == seq))
    status, text = verdict(obs)
    assert status == "FAIL" and f"[{seq}]" in text, f"{status}: {text}"


def test_f1_delayed_frame_outside_every_judged_span_is_inconclusive():
    obs = load("F1")
    t_stop = drive(obs, "quick_stop")[0]["t_ns"] / 1e9
    _, i = delayed_stream_rpdo2(obs)[0]
    obs["device"][i][0] = t_stop + 0.002  # Between the stop and the first judged SYNC.
    obs["device"].sort(key=lambda f: f[0])
    status, text = verdict(obs)
    assert status == "INCONCLUSIVE" and "not accounted" in text, f"{status}: {text}"


def test_r1_control_counted_other_tag_fails():
    """The tag revoked before the replay SYNC: the control is one more other-tag drop and no
    lone half is dropped."""
    obs = load("R1")
    t_sync, t_next = obs["harness"]["t_sync"], obs["harness"]["t_second_stream"]
    at_sync = [r for r in obs["banks"][-1] if r.get("event") == "drive" and
               r.get("kind") == "dropped" and t_sync < r["t_ns"] / 1e9 < t_next]
    other = next(r for r in at_sync if r["drop"] == judges.DROP_OTHER_TAG)
    other["count"] += 1
    for r in at_sync:
        if r["drop"] == judges.DROP_MISSING_HALF:
            obs["banks"][-1].remove(r)
    status, text = verdict(obs)
    assert status in ("FAIL", "INCONCLUSIVE") and "control" in text, f"{status}: {text}"
