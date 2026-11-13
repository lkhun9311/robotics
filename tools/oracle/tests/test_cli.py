# SPDX-License-Identifier: Apache-2.0
from oracle.cli import main


def test_cli_exit_code_for_missing_trace(tmp_path, capsys):
    assert main([str(tmp_path / "none.jsonl")]) == 3
    assert '"NOT_RUN"' in capsys.readouterr().out
