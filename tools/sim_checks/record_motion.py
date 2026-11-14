# SPDX-License-Identifier: Apache-2.0
"""Records the E2 baseline motion from the simulated `overview` camera into an MP4.

    python3 tools/sim_checks/record_motion.py --out docs/media/e2-baseline.mp4 [--fps 15]

Starts the bringup with `camera:=true`, waits for the first frame, then records a fixed 14 s:
hold 2 s, HOME -> TARGET 4 s, hold 2 s, TARGET -> HOME 4 s, hold 2 s. This is evidence, not a
check, so it is not part of run_checks.sh. It fails (never skips) when the camera sends anything
but rgb8 640x480, when too few frames arrive, or when ffmpeg fails, and then leaves an existing
--out file as it was: the video is written to a partial file and renamed over --out on success.
"""
import argparse
import os
import pathlib
import subprocess
import sys
import threading
import time

from expected import HOME, TARGET
from sim import Sim, SimError

WIDTH, HEIGHT = 640, 480
DURATION_S = 14.0
MIN_FRACTION = 0.85
FIRST_FRAME_TIMEOUT_S = 10.0
FFMPEG_EXIT_TIMEOUT_S = 60.0


class Recorder:
    """Pipes frames into ffmpeg while `enabled`; any bad frame is kept as the failure."""

    def __init__(self, ffmpeg):
        self._ffmpeg = ffmpeg
        self._lock = threading.Lock()
        self.first = threading.Event()
        self.enabled = False
        self.frames = 0
        self.error = None

    def on_frame(self, msg):
        with self._lock:
            if msg.encoding != "rgb8" or (msg.width, msg.height) != (WIDTH, HEIGHT) or msg.step != WIDTH * 3:
                self.error = self.error or (
                    f"unexpected frame: encoding={msg.encoding} {msg.width}x{msg.height} step={msg.step}"
                )
            elif self.enabled and self.error is None:
                try:
                    self._ffmpeg.stdin.write(bytes(msg.data))
                    self.frames += 1
                except OSError as e:
                    self.error = f"ffmpeg pipe closed: {e}"
            self.first.set()


def _start_ffmpeg(fps, path):
    return subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}",
         "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "28", str(path)],
        stdin=subprocess.PIPE,
    )


def _finish_ffmpeg(ffmpeg):
    """Ends ffmpeg's input and waits for it; a non-zero exit is an error."""
    try:
        ffmpeg.stdin.close()
    finally:
        try:
            code = ffmpeg.wait(timeout=FFMPEG_EXIT_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            ffmpeg.kill()
            ffmpeg.wait()
            raise SimError(f"ffmpeg did not exit within {FFMPEG_EXIT_TIMEOUT_S} s of its input ending") from None
    if code != 0:
        raise SimError(f"ffmpeg exited {code}")


def _record(sim, rec):
    """Runs the fixed schedule; returns the wall time it took."""
    sim.start()
    if not rec.first.wait(FIRST_FRAME_TIMEOUT_S):
        raise SimError(f"no frame on /overview/image_raw within {FIRST_FRAME_TIMEOUT_S} s")
    rec.enabled = True
    t0 = time.monotonic()

    def hold(until_s):
        time.sleep(max(0.0, t0 + until_s - time.monotonic()))

    hold(2.0)
    out = sim.follow([(4.0, TARGET)])
    if not out.succeeded:
        raise SimError(f"HOME -> TARGET failed: {out}")
    hold(8.0)
    back = sim.follow([(4.0, HOME)])
    if not back.succeeded:
        raise SimError(f"TARGET -> HOME failed: {back}")
    hold(DURATION_S)
    rec.enabled = False
    return time.monotonic() - t0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=15.0, help="camera rate counted from received frames (the recorder logs it), not `ros2 topic hz`")
    args = ap.parse_args(argv)

    out = pathlib.Path(args.out)
    # Written next to --out and renamed over it only on success, so a failed run leaves the
    # previous video in place.
    partial = out.with_name(f".{out.stem}.partial{out.suffix}")
    ffmpeg = _start_ffmpeg(args.fps, partial)
    rec = Recorder(ffmpeg)
    sim = Sim("record", camera=True, on_frame=rec.on_frame)
    # The first error is the one raised; errors met while cleaning up after it are attached to
    # it as notes instead of replacing it.
    failure = None
    try:
        elapsed = _record(sim, rec)
    except BaseException as e:  # BaseException: Ctrl-C must still stop the sim and ffmpeg
        failure = e
    if failure is None and rec.error:
        failure = SimError(rec.error)
    for cleanup in (sim.stop, lambda: _finish_ffmpeg(ffmpeg)):
        try:
            cleanup()
        except BaseException as e:  # a second Ctrl-C must not skip the remaining cleanup
            if failure is None:
                failure = e
            else:
                failure.add_note(f"also, while cleaning up: {type(e).__name__}: {e}")
    if failure is None:
        need = MIN_FRACTION * DURATION_S * args.fps
        print(f"recorded {rec.frames} frames in {elapsed:.1f} s wall (need at least {need:.0f}) -> {out}")
        if rec.frames < need:
            failure = SimError(f"only {rec.frames} frames, need at least {need:.0f}")
    if failure is not None:
        partial.unlink(missing_ok=True)
        raise failure
    os.replace(partial, out)


if __name__ == "__main__":
    try:
        main()
    except (SimError, AssertionError) as e:
        # AssertionError: Sim.stop() asserts that no process survived.
        print(f"record_motion: FAILED: {e}", file=sys.stderr)
        for note in getattr(e, "__notes__", []):
            print(f"record_motion: {note}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("record_motion: interrupted", file=sys.stderr)
        sys.exit(130)
