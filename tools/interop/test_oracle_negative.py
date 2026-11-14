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
# command in the proxy and be stamped on vcan_host after the NMT frame. Only such a frame (the
# previous state, before the boot-up, within GRACE_S of the command) is tolerated.

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
