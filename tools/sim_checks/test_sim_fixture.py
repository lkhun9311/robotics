# SPDX-License-Identifier: Apache-2.0
"""Checks of the `Sim` instrument itself. Nothing here starts the simulator: a stand-in process
group or hand-fed messages take its place, so each check isolates one property of `Sim`."""
import subprocess
import threading
import time

import pytest

import sim as sim_module
from sim import Sim, SimError


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


def _feed(s, topic, stamp_s):
    s._on_message(topic, lambda m: m, sim_module.Sample(stamp_s, {}, {}), {"source_timestamp": time.time_ns()})


def test_paired_positions_timeout_is_an_overall_bound():
    s = Sim("fixture-unpaired")
    stop = threading.Event()

    def publish():
        # Pairs that never match for 0.9 s, then /mujoco_actuators_states falls silent.
        t0 = time.monotonic()
        while not stop.is_set():
            now = time.monotonic() - t0
            _feed(s, "/joint_states", 100.0 + now)
            if now < 0.9:
                _feed(s, sim_module.MUJOCO_STATES, 200.0 + now)
            time.sleep(0.01)

    feeder = threading.Thread(target=publish, daemon=True)
    feeder.start()
    try:
        t0 = time.monotonic()
        with pytest.raises(SimError):
            s.paired_positions(within_s=0.02, timeout_s=1.0)
        elapsed = time.monotonic() - t0
    finally:
        stop.set()
        feeder.join()
    assert elapsed <= 1.2, f"paired_positions(timeout_s=1.0) gave up after {elapsed:.2f} s"


def test_stop_keeps_the_launch_error_and_notes_the_teardown_error(monkeypatch, tmp_path):
    monkeypatch.setattr(sim_module, "LOG_DIR", tmp_path)
    s = Sim("fixture-both-fail")

    def launch_fails():
        raise AssertionError("process group survived SIGKILL")

    def node_fails():
        raise RuntimeError("node teardown failed")

    monkeypatch.setattr(s, "_stop_launch", launch_fails)
    monkeypatch.setattr(s, "_stop_node", node_fails)
    with pytest.raises(AssertionError, match="process group survived SIGKILL") as e:
        s.stop()
    assert any("node teardown failed" in n for n in getattr(e.value, "__notes__", [])), \
        "the teardown error was dropped instead of attached to the launch error"


def test_paired_positions_reports_the_last_gap_when_the_deadline_runs_out():
    s = Sim("fixture-pair-error")
    stop = threading.Event()

    def publish():
        # Pairs 100 s apart for 0.5 s, then both topics fall silent: the deadline ends inside
        # a sample().
        t0 = time.monotonic()
        while not stop.is_set() and time.monotonic() - t0 < 0.5:
            _feed(s, "/joint_states", 100.0)
            _feed(s, sim_module.MUJOCO_STATES, 200.0)
            time.sleep(0.01)

    feeder = threading.Thread(target=publish, daemon=True)
    feeder.start()
    try:
        with pytest.raises(SimError, match=r"no sample pair within 0\.02 s of each other \(last gap 100\.0 s\)"):
            s.paired_positions(within_s=0.02, timeout_s=1.0)
    finally:
        stop.set()
        feeder.join()
