# SPDX-License-Identifier: Apache-2.0
"""Records an E2 motion: the baseline from the simulated `overview` camera into an MP4, or the
pick-and-place-shaped demo as the joint trajectory MuJoCo reports.

    python3 tools/sim_checks/record_motion.py --out docs/media/e2-baseline.mp4 [--fps 15]
    python3 tools/sim_checks/record_motion.py --demo pick_place \
        --out docs/media/e2-pick-place-recorded-joint-trajectory.json.gz

Baseline: starts the bringup with `camera:=true`, waits for the first frame, then records a fixed
14 s: hold 2 s, HOME -> TARGET 4 s, hold 2 s, TARGET -> HOME 4 s, hold 2 s. It fails (never
skips) when the camera sends anything but rgb8 640x480, when too few frames arrive, or when ffmpeg
fails.

pick_place: starts the bringup headless, moves to HOME, then sends pick_place.DEMO_SCHEDULE as one
goal and records /mujoco_actuators_states from the send until 1 s after the result. The recording
is judged as the simulation check judges it (every waypoint reached, the object out of the table,
the carry high enough), and a recording that fails is not written. It is kept at 100 Hz (every
fifth 500 Hz sample). tools/evidence/pick_place_replay.py renders it.

Both are evidence, not checks, so they are not part of run_checks.sh. On failure an existing --out
file is left as it was: the output is written to a partial file and renamed over --out on success.
"""
import argparse
import gzip
import json
import os
import pathlib
import subprocess
import sys
import threading
import time

import pick_place
from expected import HOME, JOINTS, TARGET
from sim import MUJOCO_STATES, Sim, SimError

WIDTH, HEIGHT = 640, 480
DURATION_S = 14.0
MIN_FRACTION = 0.85
FIRST_FRAME_TIMEOUT_S = 10.0
FFMPEG_EXIT_TIMEOUT_S = 60.0
KEEP_EVERY = 5  # 500 Hz -> 100 Hz in the pick_place recording
AFTER_RESULT_S = 1.0


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


def _record_pick_place(sim, site_z):
    """Runs the demo; returns the recording as a JSON-ready dict."""
    sim.start()
    home = sim.follow([(3.0, HOME)])
    if not home.succeeded:
        raise SimError(f"move to HOME failed: {home}")
    sim.start_recording()
    sent = sim.stamp_now()
    r = sim.follow(pick_place.points(pick_place.DEMO_SCHEDULE))
    result = sim.stamp_now()
    rec = sim.stop_recording(until_stamp=result + AFTER_RESULT_S)
    if not r.succeeded:
        raise SimError(f"the pick_place goal failed: {r}")
    samples = [(s.stamp_s, s.position) for _, s in rec.get(MUJOCO_STATES, [])]
    stamps = [t for t, _ in samples]
    gap = max((b - a for a, b in zip(stamps, stamps[1:])), default=float("inf"))
    if not samples or stamps[0] > sent + 0.1 or stamps[-1] < result + AFTER_RESULT_S or gap > 0.1:
        raise SimError(f"recording does not cover the goal: {len(samples)} samples, largest stamp gap {gap} s")
    reaches, heights = pick_place.judge(samples, sent, site_z, pick_place.DEMO_SCHEDULE)
    problems = pick_place.failures(reaches, heights)
    if problems:
        raise SimError("the recorded demo fails the check's conditions: " + "; ".join(problems))
    for x in reaches:
        print(f"{x.name}: best largest-joint error {x.best_error:.4f} rad, lowest site z {x.lowest_z:.4f} m")
    kept = samples[::KEEP_EVERY]
    return {
        "what": "MuJoCo joint positions from /mujoco_actuators_states during the pick_place demo goal",
        "joints": list(JOINTS),
        "schedule": [list(x) for x in pick_place.DEMO_SCHEDULE],
        "waypoints": {k: list(v) for k, v in pick_place.WAYPOINTS.items()},
        "result_s": round(result - sent, 6),
        "sample_rate_hz": 500 / KEEP_EVERY,
        # [stamp s after the send, positions in `joints` order]
        "samples": [[round(t - sent, 6)] + [round(p[j], 6) for j in JOINTS] for t, p in kept],
    }


def _main_pick_place(out):
    from kinematics import Kinematics

    kin = Kinematics()
    partial = out.with_name(f".{out.name}.partial")
    sim = Sim("record-pick-place")
    failure = None
    try:
        data = _record_pick_place(sim, lambda p: kin.site(p)[0][2])
    except BaseException as e:  # BaseException: Ctrl-C must still stop the sim
        failure = e
    try:
        sim.stop()
    except BaseException as e:
        if failure is None:
            failure = e
        else:
            failure.add_note(f"also, while cleaning up: {type(e).__name__}: {e}")
    if failure is not None:
        raise failure
    # mtime 0: the bytes depend on the data only.
    with open(partial, "wb") as raw, gzip.GzipFile(fileobj=raw, mode="wb", mtime=0, filename="") as f:
        f.write((json.dumps(data, separators=(",", ":")) + "\n").encode())
    os.replace(partial, out)
    print(f"recorded {len(data['samples'])} samples -> {out}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--demo", choices=("baseline", "pick_place"), default="baseline")
    ap.add_argument("--fps", type=float, default=15.0, help="camera rate counted from received frames (the recorder logs it), not `ros2 topic hz`")
    args = ap.parse_args(argv)

    out = pathlib.Path(args.out)
    if args.demo == "pick_place":
        return _main_pick_place(out)
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
