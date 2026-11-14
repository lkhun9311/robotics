# SPDX-License-Identifier: Apache-2.0
"""The committed E3b evidence is what the committed scripts make from the committed inputs."""
import json

import pytest

import common
import fault_timeline
import frame_trace
import matrix_table
import proxy_cost_histogram

MEDIA = common.MEDIA


def test_frame_trace_is_regenerated_from_the_m5_recording():
    text = frame_trace.build(common.load_recording("m5"), 21, "m5")
    assert text == (MEDIA / "e3b-frame-trace.txt").read_text(encoding="utf-8")


def test_frame_trace_refuses_a_seq_without_one_actuation():
    obs = common.load_recording("m5")
    obs["bank"] = [r for r in obs["bank"] if not (r.get("kind") == "actuated" and r["seq"] == 21)]
    with pytest.raises(SystemExit, match="0 actuation records"):
        frame_trace.build(obs, 21, "m5")


def test_fault_timeline_is_regenerated_from_the_f1_and_f7_recordings(tmp_path):
    out = tmp_path / "timeline.svg"
    fault_timeline.draw(out)
    assert out.read_bytes() == (MEDIA / "e3b-fault-timeline.svg").read_bytes()


def test_proxy_cost_histogram_is_regenerated_from_the_committed_measurement(tmp_path):
    out = tmp_path / "cost.svg"
    cost = json.loads((MEDIA / "e3b-proxy-cost.json").read_text())
    assert cost["host"] and cost["commit"] and sum(n for _, n in cost["dist"]) == cost["samples"]
    proxy_cost_histogram.draw(cost, out, 200)
    assert out.read_bytes() == (MEDIA / "e3b-proxy-cost.svg").read_bytes()


def test_matrix_table_never_reports_a_missing_or_failed_row_as_pass(tmp_path):
    junit = tmp_path / "junit.xml"
    junit.write_text(
        '<testsuites><testsuite name="pytest" tests="2">'
        '<testcase classname="tools.interop.test_matrix" name="test_nmt_states_and_service_gating"/>'
        '<testcase classname="tools.interop.test_matrix" name="test_sdo_expedited_and_aborts">'
        '<failure message="M3 FAIL"/></testcase></testsuite></testsuites>')
    rows = {line.split("|")[1].strip(): line.split("|")[4].strip()
            for line in matrix_table.table(junit).splitlines()[2:]}
    assert rows["M1"] == "PASS"
    assert rows["M3"] == "FAIL"
    assert rows["M2"] == "MISSING" and rows["R1"] == "MISSING"
