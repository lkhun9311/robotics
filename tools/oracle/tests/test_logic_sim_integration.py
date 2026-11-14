# SPDX-License-Identifier: Apache-2.0
"""Each clean scenario must PASS (which includes its scenario evidence); each paired defect
must be caught for the intended reason. A missing binary fails the test: a skip is never a pass."""
import os
import subprocess

import pytest

from oracle.rules import FAIL, INCONCLUSIVE, PASS, judge_file

CASES = [
    ("normal", "none", PASS, None),
    ("manager_stall", "none", PASS, None),
    ("generation_switch", "none", PASS, None),
    ("fault_reset_reenable", "none", PASS, None),
    ("manager_stall", "expiry_off_by_one", FAIL, "expired"),
    ("generation_switch", "skip_generation_check", FAIL, "!= permit"),
    ("fault_reset_reenable", "auto_resume", FAIL, "without explicit reset and enable"),
    ("fault_reset_reenable", "reuse_identity", FAIL, "reuses a closed session/generation"),
    ("fault_reset_reenable", "replay_bypass", FAIL, "redelivered command applied"),
    ("normal", "drop_apply_event", INCONCLUSIVE, "without drive outcome"),
]

# Reasons that would mean the trace itself is broken, so a FAIL must not rest on them.
STRUCTURAL = ("must be int", "must be str", "clock", "scenario_begin", "scenario_end")


def _binary() -> str:
    path = os.environ.get("LOGIC_SIM_BIN")
    if not path:
        pytest.fail("LOGIC_SIM_BIN is not set; this test must run, not skip")
    return path


def _trace_dir(tmp_path):
    # ORACLE_TRACE_DIR keeps every trace (including failing ones) for the CI artifact.
    keep = os.environ.get("ORACLE_TRACE_DIR")
    if keep:
        os.makedirs(keep, exist_ok=True)
        return keep
    return str(tmp_path)


@pytest.mark.parametrize("scenario,defect,expected,reason", CASES)
def test_oracle_verdicts(tmp_path, scenario, defect, expected, reason):
    out = os.path.join(_trace_dir(tmp_path), f"{scenario}-{defect}.jsonl")
    subprocess.run([_binary(), "--scenario", scenario, "--defect", defect, "--out", out], check=True)
    verdict = judge_file(out)
    assert verdict.result == expected, verdict.reasons
    assert not any(s in r for r in verdict.reasons for s in STRUCTURAL), verdict.reasons
    if reason:
        assert any(reason in r for r in verdict.reasons), verdict.reasons


def test_bad_arguments_exit_2(tmp_path):
    result = subprocess.run([_binary(), "--scenario", "nope", "--defect", "none", "--out", str(tmp_path / "x")])
    assert result.returncode == 2
