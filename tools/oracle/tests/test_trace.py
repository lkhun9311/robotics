# SPDX-License-Identifier: Apache-2.0
import json

from oracle.trace import load_trace


def _write(path, lines):
    path.write_text("".join(line + "\n" for line in lines), encoding="utf-8")


def test_missing_file(tmp_path):
    assert load_trace(tmp_path / "nope.jsonl").status == "missing"


def test_empty_file(tmp_path):
    p = tmp_path / "t.jsonl"
    p.write_text("\n\n", encoding="utf-8")
    assert load_trace(p).status == "empty"


def test_time_going_backwards_is_invalid(tmp_path):
    p = tmp_path / "t.jsonl"
    _write(p, [json.dumps({"t_ns": 5, "clock": "c", "kind": "a"}),
               json.dumps({"t_ns": 4, "clock": "c", "kind": "b"})])
    result = load_trace(p)
    assert result.status == "invalid"
    assert "backwards" in result.error


def test_non_integer_time_is_invalid(tmp_path):
    p = tmp_path / "t.jsonl"
    _write(p, [json.dumps({"t_ns": 1.5, "clock": "c", "kind": "a"})])
    assert load_trace(p).status == "invalid"


def test_non_string_clock_is_invalid(tmp_path):
    p = tmp_path / "t.jsonl"
    _write(p, [json.dumps({"t_ns": 1, "clock": 3, "kind": "a"})])
    assert load_trace(p).status == "invalid"


def test_bad_json_is_invalid(tmp_path):
    p = tmp_path / "t.jsonl"
    _write(p, ["{not json"])
    assert load_trace(p).status == "invalid"


def test_directory_is_invalid(tmp_path):
    assert load_trace(tmp_path).status == "invalid"


def test_non_utf8_file_is_invalid(tmp_path):
    p = tmp_path / "t.jsonl"
    p.write_bytes(b"\xff\xfe\x00")
    assert load_trace(p).status == "invalid"
