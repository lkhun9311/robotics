# SPDX-License-Identifier: Apache-2.0
"""The matrix judges on recorded observations (no vcan): a verdict is never reached on an
incomplete observation.

tools/interop/recorded/m1..m6.json.gz are observations of passing runs of test_matrix.py
(written with INTEROP_RECORD_DIR). Each recording passes its judge unchanged (the control);
with one required kActuated removed the M5 judge is INCONCLUSIVE; with trace_overflow = 1 in the
bank trace every judge is INCONCLUSIVE.
"""
import copy
import gzip
import json
import pathlib

import pytest

import judges

RECORDED = pathlib.Path(__file__).parent / "recorded"
ROWS = ("M1", "M2", "M3", "M4", "M5", "M6")


def load(row):
    with gzip.open(RECORDED / f"{row.lower()}.json.gz", "rt", encoding="utf-8") as f:
        return json.load(f)


@pytest.mark.parametrize("row", ROWS)
def test_recorded_observation_passes(row):
    status, text = judges.verdict(load(row))
    assert status == "PASS", f"{row} {status}: {text}"


def test_m5_missing_actuation_is_inconclusive():
    obs = load("M5")
    actuated = [r for r in obs["bank"] if r.get("event") == "drive" and
                r.get("node") == judges.NODE and r.get("kind") == "actuated"]
    assert len(actuated) >= 20, "the recording has too few kActuated records to cut one"
    removed = actuated[len(actuated) // 2]
    obs["bank"].remove(removed)
    status, text = judges.verdict(obs)
    assert status == "INCONCLUSIVE", f"{status}: {text}"
    assert "kActuated missing" in text and str(removed["seq"]) in text, text


@pytest.mark.parametrize("row", ROWS)
def test_trace_overflow_is_inconclusive(row):
    obs = copy.deepcopy(load(row))
    # The record that closes the trace: the exit record, or the last snapshot before a kill.
    last = next(r for r in reversed(obs["bank"]) if "trace_overflow" in r)
    last["trace_overflow"] = 1
    status, text = judges.verdict(obs)
    assert status == "INCONCLUSIVE", f"{row} {status}: {text}"
    assert "trace_overflow = 1" in text, text


# M1 resets: a heartbeat the bank sent just before it handled the reset can cross the reset
# command in the proxy and be stamped on vcan_host after the NMT frame.
# Only such a frame (the previous state, before the boot-up, within GRACE_S of the command) is
# tolerated.

def m1_reset(obs, cs):
    """(t of the reset NMT frame to node 3 on vcan_host, index of its boot-up in obs['host'])."""
    t0 = next(t for t, c, h in obs["host"] if c == 0 and h == f"{cs:02x}{judges.NODE:02x}")
    i = next(i for i, (t, c, h) in enumerate(obs["host"])
             if c == 0x700 + judges.NODE and t > t0 and h == "00")
    return t0, i


def with_frame(obs, t, cob, data):
    obs["host"].append([t, cob, data])
    obs["host"].sort(key=lambda f: f[0])
    return obs


@pytest.mark.parametrize("cs", [0x82, 0x81])
def test_m1_stale_heartbeat_within_grace_passes(cs):
    obs = load("M1")
    t0, i = m1_reset(obs, cs)
    t_boot = obs["host"][i][0]
    obs["host"][i][0] = t0 + 0.6 * judges.GRACE_S  # boot-up after the crossing heartbeat
    with_frame(obs, t0 + 0.3 * judges.GRACE_S, 0x700 + judges.NODE, "7f")
    assert t_boot - t0 < judges.GRACE_S, "the recorded boot-up is not right after the reset"
    status, text = judges.verdict(obs)
    assert status == "PASS", f"{status}: {text}"


@pytest.mark.parametrize("cs", [0x82, 0x81])
def test_m1_missing_boot_up_fails(cs):
    obs = load("M1")
    _, i = m1_reset(obs, cs)
    del obs["host"][i]
    status, text = judges.verdict(obs)
    assert status == "FAIL" and "boot-up" in text, f"{status}: {text}"


@pytest.mark.parametrize("cs", [0x82, 0x81])
def test_m1_stale_heartbeat_beyond_grace_fails(cs):
    obs = load("M1")
    t0, i = m1_reset(obs, cs)
    obs["host"][i][0] = t0 + 3 * judges.GRACE_S
    with_frame(obs, t0 + 2 * judges.GRACE_S, 0x700 + judges.NODE, "7f")
    status, text = judges.verdict(obs)
    assert status == "FAIL" and "boot-up" in text, f"{status}: {text}"


@pytest.mark.parametrize("cs", [0x82, 0x81])
def test_m1_heartbeat_right_after_boot_up_fails(cs):
    obs = load("M1")
    _, i = m1_reset(obs, cs)
    t_boot = obs["host"][i][0]
    with_frame(obs, t_boot + 0.5 * judges.GRACE_S, 0x700 + judges.NODE, "7f")
    status, text = judges.verdict(obs)
    assert status == "FAIL", f"{status}: {text}"


# M1 Stopped/Pre-op: Lely applies a synchronous TPDO it received at the next SYNC (CiA 301
# synchronous reception), so the TPDO the node sent right before the command is reported by
# Lely at its first SYNC after the command.
# Only that one SYNC of lag is tolerated.

def m1_lely_pdo_group(obs, cs, nth_sync, after_s=0.0):
    """Adds Lely rpdo lines for node 3 after Lely's nth SYNC line after the NMT command cs
    (after_s later than that line)."""
    sent = next(e["t"] for e in obs["peer"] if e.get("event") == "nmt_sent" and e["cs"] == cs)
    t = [e["t"] for e in obs["peer"] if e.get("event") == "sync" and e["t"] > sent][nth_sync - 1]
    group = [{"t": t + after_s + 1e-6 * (i + 1), "event": "rpdo", "node": judges.NODE, "index": 0x6041,
              "sub": 0, "value": 0x0260} for i in range(1)]
    obs["peer"] = sorted(obs["peer"] + group, key=lambda e: e["t"])
    return obs


@pytest.mark.parametrize("cs", [0x02, 0x80])
def test_m1_lely_pdo_at_first_sync_after_command_passes(cs):
    status, text = judges.verdict(m1_lely_pdo_group(load("M1"), cs, 1))
    assert status == "PASS", f"{status}: {text}"


@pytest.mark.parametrize("cs", [0x02, 0x80])
def test_m1_lely_pdo_at_second_sync_after_command_fails(cs):
    status, text = judges.verdict(m1_lely_pdo_group(load("M1"), cs, 2))
    assert status == "FAIL" and "Lely decoded PDO values" in text, f"{status}: {text}"


# M1 Stopped/Pre-op judges Lely's lines from its second SYNC line after the command, by order
# of Lely's own lines and not by a time window around the first one: a peer preempted between
# its sync line and the rpdo lines of that SYNC still prints them before its next sync line.

@pytest.mark.parametrize("cs", [0x02, 0x80])
def test_m1_lely_pdo_late_within_first_sync_passes(cs):
    # 6 ms after the first SYNC line (before the second): failed under the earlier 1 ms window.
    status, text = judges.verdict(m1_lely_pdo_group(load("M1"), cs, 1, after_s=0.006))
    assert status == "PASS", f"{status}: {text}"


@pytest.mark.parametrize("cs", [0x02, 0x80])
def test_m1_lely_pdo_late_within_second_sync_fails(cs):
    status, text = judges.verdict(m1_lely_pdo_group(load("M1"), cs, 2, after_s=0.006))
    assert status == "FAIL" and "Lely decoded PDO values" in text, f"{status}: {text}"


# M4: the malformed count is read in the first dispatch snapshot after the bank actuated the
# control pair sent behind the DLC 7 frame (the bank's clock), not in the first snapshot after
# the harness's send time: a snapshot between the send and the dispatch shows the old count.

def m4_with_snapshot(obs, t_ns, malformed):
    """Adds a node 3 dispatch snapshot at t_ns with the given malformed count."""
    snap = next(r for r in obs["bank"] if r.get("event") == "dispatch" and
                r.get("node") == judges.NODE)
    i = next(i for i, r in enumerate(obs["bank"]) if r["t_ns"] > t_ns)
    obs["bank"].insert(i, dict(snap, t_ns=t_ns, malformed=malformed))
    return obs


def m4_snapshots(obs):
    bad = obs["harness"]["bad"]["t"]
    snaps = [r for r in obs["bank"] if r.get("event") == "dispatch" and
             r.get("node") == judges.NODE]
    control = next(r for r in obs["bank"] if r.get("event") == "drive" and
                   r.get("kind") == "actuated" and r.get("node") == judges.NODE and
                   r.get("seq") == obs["harness"]["control"]["seq"])
    before = [r for r in snaps if r["t_ns"] / 1e9 < bad][-1]
    after = next(r for r in snaps if r["t_ns"] > control["t_ns"])
    return before, after


def test_m4_snapshot_between_send_and_dispatch_passes():
    obs = load("M4")
    before, _ = m4_snapshots(obs)
    t_ns = int(obs["harness"]["bad"]["t"] * 1e9) + 100_000  # 0.1 ms after the send
    status, text = judges.verdict(m4_with_snapshot(obs, t_ns, before["malformed"]))
    assert status == "PASS", f"{status}: {text}"


def test_m4_count_unchanged_after_the_control_actuation_fails():
    obs = load("M4")
    before, after = m4_snapshots(obs)
    after["malformed"] = before["malformed"]
    status, text = judges.verdict(obs)
    assert status == "FAIL" and "malformed count rose by 0" in text, f"{status}: {text}"
