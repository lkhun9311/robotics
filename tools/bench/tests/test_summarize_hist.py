# SPDX-License-Identifier: Apache-2.0
import pytest

import summarize_hist

HIST = """\
000000 000000
000001 000090
000002 000009
000003 000001
# Total: 000000100
# Min Latencies: 00001
# Avg Latencies: 00001
# Max Latencies: 00003
# Histogram Overflows: 00000
"""


def test_percentiles_come_from_histogram_counts():
    s = summarize_hist.summarize(HIST)
    assert s["samples_in_histogram"] == 100
    assert s["samples_total"] == 100
    assert s["p50_us"] == 1
    assert s["p99_us"] == 2
    assert s["p999_us"] == 3
    assert s["max_us"] == 3
    assert s["overflows"] == 0
    assert s["percentiles_are_lower_bounds"] is False


def test_overflow_marks_percentiles_as_lower_bounds():
    text = HIST.replace("Histogram Overflows: 00000", "Histogram Overflows: 00004")
    s = summarize_hist.summarize(text)
    assert s["overflows"] == 4
    assert s["samples_total"] == 104
    assert s["percentiles_are_lower_bounds"] is True


def test_empty_histogram_is_an_error():
    with pytest.raises(ValueError):
        summarize_hist.summarize("# Max Latencies: 00000\n")
