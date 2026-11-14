# SPDX-License-Identifier: Apache-2.0
import hashlib
import pathlib
import subprocess
import sys

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
    problems = menagerie.verify(model, sums)
    assert len(problems) == 1
    assert "missing" in problems[0]


def test_verify_reports_an_extra_file(tmp_path):
    model, sums = _tree(tmp_path)
    (model / "extra.txt").write_text("x")
    problems = menagerie.verify(model, sums)
    assert len(problems) == 1
    assert "unexpected" in problems[0]


def test_cli_verify_exit_codes(tmp_path):
    model, sums = _tree(tmp_path)
    run = lambda: subprocess.run(
        [sys.executable, str(SCRIPT), "verify", str(model), str(sums)],
        capture_output=True, text=True,
    )
    assert run().returncode == 0
    (model / "ur5e.xml").write_text("changed")
    result = run()
    assert result.returncode == 1
    assert "mismatch" in result.stdout + result.stderr
