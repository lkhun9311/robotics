# SPDX-License-Identifier: Apache-2.0
"""Checks of the `Sim` instrument itself. Nothing here starts the simulator: a stand-in process
group or hand-fed messages take its place, so each check isolates one property of `Sim`."""
import subprocess
import time

import pytest

import sim as sim_module
from sim import Sim


def _stand_in(sim, command):
    """Gives `sim` a launch process group running `command`, as `Sim.start()` would."""
    sim._log = open(sim.log_path, "w")
    sim._proc = subprocess.Popen(["bash", "-c", command], start_new_session=True,
                                 stdout=sim._log, stderr=subprocess.STDOUT)
    time.sleep(0.2)  # let bash install its traps before a signal arrives
    return sim._proc.pid


def test_stop_kills_the_launch_group_even_when_node_teardown_fails(monkeypatch, tmp_path):
    monkeypatch.setattr(sim_module, "LOG_DIR", tmp_path)
    s = Sim("fixture-teardown-fails")
    pgid = _stand_in(s, "sleep 60")

    def broken_teardown():
        raise RuntimeError("node teardown failed")

    monkeypatch.setattr(s, "_stop_node", broken_teardown)
    with pytest.raises(RuntimeError, match="node teardown failed"):
        s.stop()
    alive = sim_module._group_alive(pgid)
    if alive:
        sim_module.os.killpg(pgid, sim_module.signal.SIGKILL)
    assert not alive, "the launch process group outlived a failed node teardown"


def test_stop_waits_ten_seconds_in_total_before_sigkill(monkeypatch, tmp_path):
    monkeypatch.setattr(sim_module, "LOG_DIR", tmp_path)
    s = Sim("fixture-ignores-sigint")
    # Both bash and its child ignore SIGINT, so only SIGKILL ends the group.
    pgid = _stand_in(s, "trap '' INT; sleep 60")
    t0 = time.monotonic()
    s.stop()
    elapsed = time.monotonic() - t0
    assert not sim_module._group_alive(pgid)
    assert 9.5 <= elapsed <= 11.5, f"stop() took {elapsed:.1f} s; SIGKILL is due 10 s after SIGINT"
