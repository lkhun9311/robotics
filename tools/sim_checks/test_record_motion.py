# SPDX-License-Identifier: Apache-2.0
"""Failure handling of record_motion.py, with a stand-in for the simulator and the real ffmpeg.
A recording run itself is evidence, not a check, so no test here starts the simulator."""
import types

import pytest

import record_motion
from sim import FollowResult, SimError

ABORTED = 6  # action_msgs/msg/GoalStatus STATUS_ABORTED
FRAME = types.SimpleNamespace(encoding="rgb8", width=640, height=480, step=640 * 3,
                              data=bytes(640 * 480 * 3))


class FakeSim:
    """Stands in for `Sim`: `start` sends one valid frame, then each hook does what a test sets."""

    start_error = None
    stop_error = None
    follow_result = FollowResult(True, 0, "", 4)  # STATUS_SUCCEEDED

    def __init__(self, name, camera=False, on_frame=None):
        self._on_frame = on_frame

    def start(self):
        if self.start_error:
            raise self.start_error
        self._on_frame(FRAME)

    def follow(self, points):
        return self.follow_result

    def stop(self):
        if self.stop_error:
            raise self.stop_error


@pytest.fixture
def out(tmp_path, monkeypatch):
    monkeypatch.setattr(record_motion, "Sim", FakeSim)
    path = tmp_path / "baseline.mp4"
    path.write_bytes(b"the committed video")
    return path


def test_a_failed_run_leaves_the_existing_output_untouched(out, monkeypatch):
    monkeypatch.setattr(FakeSim, "start_error", SimError("controllers not active"))
    with pytest.raises(SimError, match="controllers not active"):
        record_motion.main(["--out", str(out)])
    assert out.read_bytes() == b"the committed video"
    assert sorted(p.name for p in out.parent.iterdir()) == [out.name], "a partial file was left behind"


def test_a_failed_goal_is_an_error_not_an_assert(out, monkeypatch):
    # `assert` disappears under `python -O`; a failed goal must fail the run regardless.
    monkeypatch.setattr(FakeSim, "follow_result", FollowResult(True, -5, "tolerance", ABORTED))
    with pytest.raises(SimError, match="HOME -> TARGET failed"):
        record_motion.main(["--out", str(out)])
    assert out.read_bytes() == b"the committed video"


def test_a_cleanup_error_does_not_mask_the_first_error(out, monkeypatch):
    monkeypatch.setattr(FakeSim, "start_error", SimError("first failure"))
    monkeypatch.setattr(FakeSim, "stop_error", AssertionError("process group survived SIGKILL"))
    with pytest.raises(SimError, match="first failure") as e:
        record_motion.main(["--out", str(out)])
    assert any("process group survived SIGKILL" in note for note in getattr(e.value, "__notes__", [])), \
        "the cleanup error was dropped instead of reported alongside the first error"


def test_ctrl_c_stops_the_sim_removes_the_partial_file_and_ends_ffmpeg(out, monkeypatch):
    started = []
    real_start = record_motion._start_ffmpeg
    monkeypatch.setattr(record_motion, "_start_ffmpeg", lambda fps, path: started.append(real_start(fps, path)) or started[-1])
    stopped = []
    monkeypatch.setattr(FakeSim, "start_error", KeyboardInterrupt())
    monkeypatch.setattr(FakeSim, "stop", lambda self: stopped.append(True))
    with pytest.raises(KeyboardInterrupt):
        record_motion.main(["--out", str(out)])
    assert stopped, "Sim.stop() was not called after Ctrl-C"
    assert started[0].poll() is not None, "ffmpeg was left running after Ctrl-C"
    assert out.read_bytes() == b"the committed video"
    assert sorted(p.name for p in out.parent.iterdir()) == [out.name], "a partial file was left behind"
