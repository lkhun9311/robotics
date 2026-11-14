# SPDX-License-Identifier: Apache-2.0
import hashlib
import pathlib
import subprocess
import sys

import pytest

import menagerie

SCRIPT = pathlib.Path(menagerie.__file__)


def _tree(tmp_path):
    model = tmp_path / "model"
    (model / "assets").mkdir(parents=True)
    (model / "ur5e.xml").write_text("<mujoco/>\n")
    (model / "assets" / "base.obj").write_text("v 0 0 0\n")
    sums = tmp_path / "SHA256SUMS"
    lines = []
    for rel in ("assets/base.obj", "ur5e.xml"):
        digest = hashlib.sha256((model / rel).read_bytes()).hexdigest()
        lines.append(f"{digest}  {rel}\n")
    sums.write_text("".join(lines))
    return model, sums


def test_verify_accepts_matching_tree(tmp_path):
    model, sums = _tree(tmp_path)
    assert menagerie.verify(model, sums) == []


def test_verify_reports_a_modified_file(tmp_path):
    model, sums = _tree(tmp_path)
    (model / "ur5e.xml").write_text("<mujoco model='x'/>\n")
    problems = menagerie.verify(model, sums)
    assert len(problems) == 1
    assert "ur5e.xml" in problems[0] and "mismatch" in problems[0]


def test_verify_reports_a_missing_file(tmp_path):
    model, sums = _tree(tmp_path)
    (model / "assets" / "base.obj").unlink()
    assert menagerie.verify(model, sums) == ["missing: assets/base.obj"]


def test_verify_reports_an_extra_file(tmp_path):
    model, sums = _tree(tmp_path)
    (model / "extra.txt").write_text("x")
    problems = menagerie.verify(model, sums)
    assert len(problems) == 1
    assert "unexpected" in problems[0]


def _cli(*args):
    return subprocess.run([sys.executable, str(SCRIPT), *map(str, args)], capture_output=True, text=True)


def test_cli_verify_exit_codes(tmp_path):
    model, sums = _tree(tmp_path)
    assert _cli("verify", model, sums).returncode == 0
    (model / "ur5e.xml").write_text("changed")
    result = _cli("verify", model, sums)
    assert result.returncode == 1
    assert "mismatch" in result.stdout + result.stderr


def test_cli_usage_exits_2():
    for args in ((), ("verify", "only-one"), ("frobnicate", "a", "b")):
        result = _cli(*args)
        assert result.returncode == 2, args
        assert "menagerie.py verify <dir> <sums>" in result.stderr, args


def test_cli_unreadable_sums_file_exits_3(tmp_path):
    model, _ = _tree(tmp_path)
    result = _cli("verify", model, tmp_path / "absent")
    assert result.returncode == 3
    assert result.stderr.startswith("menagerie.py: cannot verify: "), result.stderr
    assert "absent" in result.stderr and "Traceback" not in result.stderr, result.stderr


def test_cli_missing_model_dir_exits_3(tmp_path):
    _, sums = _tree(tmp_path)
    result = _cli("verify", tmp_path / "nowhere", sums)
    assert result.returncode == 3
    assert "nowhere" in result.stderr and "is not a directory" in result.stderr, result.stderr


@pytest.mark.parametrize("line", [
    "garbage",
    "abc  ur5e.xml",
    f"{'0' * 64} ur5e.xml",
    f"{'0' * 64}  ",
])
def test_cli_malformed_sums_line_exits_3(tmp_path, line):
    model, sums = _tree(tmp_path)
    sums.write_text(sums.read_text() + line + "\n")
    result = _cli("verify", model, sums)
    assert result.returncode == 3, result.stderr
    assert f"{sums}:3: malformed line" in result.stderr, result.stderr


def test_cli_duplicate_sums_entry_exits_3(tmp_path):
    model, sums = _tree(tmp_path)
    first = sums.read_text().splitlines()[0]
    sums.write_text(sums.read_text() + first + "\n")
    result = _cli("verify", model, sums)
    assert result.returncode == 3
    assert f"{sums}:3: duplicate entry for assets/base.obj" in result.stderr, result.stderr
