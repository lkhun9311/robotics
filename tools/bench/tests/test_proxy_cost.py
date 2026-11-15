# SPDX-License-Identifier: Apache-2.0
import json

import pytest

import proxy_cost

EXIT = {"event": "exit", "t_ns": 5, "trace_overflow": 0, "signal": 15,
        "cost_ns": {"count": 6000, "samples": 6000, "p50": 9000, "p99": 30000, "max": 80000,
                    "dist": [[9000, 5900], [30000, 99], [80000, 1]]}}


def trace(tmp_path, *records):
    path = tmp_path / "proxy.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in records))
    return path


def test_summary_is_the_exit_records_cost(tmp_path):
    s = proxy_cost.summarize(trace(tmp_path, {"event": "start", "t_ns": 1}, EXIT))
    assert (s["count"], s["samples"], s["p50"], s["p99"], s["max"]) == (6000, 6000, 9000, 30000,
                                                                       80000)
    assert s["unit"] == "ns"


def test_a_trace_without_a_clean_exit_is_refused(tmp_path):
    with pytest.raises(ValueError, match="exit"):
        proxy_cost.summarize(trace(tmp_path, {"event": "start", "t_ns": 1}))
    with pytest.raises(ValueError, match="trace_overflow"):
        proxy_cost.summarize(trace(tmp_path, dict(EXIT, trace_overflow=2)))


def test_cli_writes_the_json_with_commit_and_host(tmp_path):
    out = tmp_path / "out" / "proxy-cost.json"
    rc = proxy_cost.main([str(trace(tmp_path, EXIT)), "--out", str(out), "--commit", "abc123",
                          "--host", "bench-1"])
    assert rc == 0
    written = json.loads(out.read_text())
    assert written["count"] == 6000 and written["p99"] == 30000
    assert written["commit"] == "abc123" and written["host"] == "bench-1"


def test_the_distribution_is_kept_for_the_histogram(tmp_path):
    s = proxy_cost.summarize(trace(tmp_path, EXIT))
    assert s["dist"] == [[9000, 5900], [30000, 99], [80000, 1]]


def test_a_distribution_that_does_not_sum_to_the_samples_is_refused(tmp_path):
    cost = dict(EXIT["cost_ns"], dist=[[9000, 5900], [30000, 99]])
    with pytest.raises(ValueError, match="dist"):
        proxy_cost.summarize(trace(tmp_path, dict(EXIT, cost_ns=cost)))
    unsorted = dict(EXIT["cost_ns"], dist=[[30000, 99], [9000, 5900], [80000, 1]])
    with pytest.raises(ValueError, match="dist"):
        proxy_cost.summarize(trace(tmp_path, dict(EXIT, cost_ns=unsorted)))
