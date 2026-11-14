# SPDX-License-Identifier: Apache-2.0
from oracle.cli import main


def test_cli_exit_code_for_missing_trace(tmp_path, capsys):
    assert main([str(tmp_path / "none.jsonl")]) == 3
    assert '"NOT_RUN"' in capsys.readouterr().out


def test_cli_maps_an_unreadable_trace_to_inconclusive(tmp_path, capsys):
    assert main([str(tmp_path)]) == 2
    assert '"INCONCLUSIVE"' in capsys.readouterr().out
