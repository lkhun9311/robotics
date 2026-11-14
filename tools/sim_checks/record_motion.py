# SPDX-License-Identifier: Apache-2.0
"""Records the E2 baseline motion from the simulated `overview` camera into an MP4.

    python3 tools/sim_checks/record_motion.py --out docs/media/e2-baseline.mp4 [--fps 15]

Starts the bringup with `camera:=true`, waits for the first frame, then records a fixed 14 s:
hold 2 s, HOME -> TARGET 4 s, hold 2 s, TARGET -> HOME 4 s, hold 2 s. This is evidence, not a
check, so it is not part of run_checks.sh. It fails (never skips) when the camera sends anything
but rgb8 640x480, when too few frames arrive, or when ffmpeg fails.
"""
import argparse
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=15.0, help="camera rate counted from received frames (the recorder logs it), not `ros2 topic hz`")
    args = ap.parse_args()

    ffmpeg = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}",
         "-r", str(args.fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "28", args.out],
        stdin=subprocess.PIPE,
    )
    rec = Recorder(ffmpeg)
    sim = Sim("record", camera=True, on_frame=rec.on_frame)
    try:
        sim.start()
        if not rec.first.wait(FIRST_FRAME_TIMEOUT_S):
            raise SimError(f"no frame on /overview/image_raw within {FIRST_FRAME_TIMEOUT_S} s")
        rec.enabled = True
        t0 = time.monotonic()

        def hold(until_s):
            time.sleep(max(0.0, t0 + until_s - time.monotonic()))

        hold(2.0)
        out = sim.follow([(4.0, TARGET)])
        assert out.succeeded, f"HOME -> TARGET failed: {out}"
        hold(8.0)
        back = sim.follow([(4.0, HOME)])
        assert back.succeeded, f"TARGET -> HOME failed: {back}"
        hold(DURATION_S)
        rec.enabled = False
        elapsed = time.monotonic() - t0
    finally:
        sim.stop()
        ffmpeg.stdin.close()
        code = ffmpeg.wait()
    if rec.error:
        raise SimError(rec.error)
    if code != 0:
        raise SimError(f"ffmpeg exited {code}")
    need = MIN_FRACTION * DURATION_S * args.fps
    print(f"recorded {rec.frames} frames in {elapsed:.1f} s wall (need at least {need:.0f}) -> {args.out}")
    if rec.frames < need:
        raise SimError(f"only {rec.frames} frames, need at least {need:.0f}")


if __name__ == "__main__":
    try:
        main()
    except (SimError, AssertionError) as e:
        print(f"record_motion: FAILED: {e}", file=sys.stderr)
        sys.exit(1)
