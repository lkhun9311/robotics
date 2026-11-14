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
