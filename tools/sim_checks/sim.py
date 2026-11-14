# SPDX-License-Identifier: Apache-2.0
"""Starts the UR5e bringup headless and talks to it over ROS 2.

`Sim.start()` launches `ros2 launch robot_bringup ur5e_baseline.launch.py headless:=true` in its
own session and returns once both controllers are active. `Sim.stop()` ends that session and
asserts nothing in it survived. All timeouts are wall time: the test node keeps
`use_sim_time` false.
"""
import collections
import dataclasses
import os
import pathlib
import resource
import signal
import subprocess
import threading
import time

from expected import JOINTS

REPO = pathlib.Path(__file__).resolve().parents[2]
LOG_DIR = REPO / "artifacts" / "sim"
CONTROLLERS = ("joint_state_broadcaster", "arm_controller")
LAUNCH = ["ros2", "launch", "robot_bringup", "ur5e_baseline.launch.py", "headless:=true"]


class SimError(Exception):
    """The simulator could not be started, or did not answer in time."""


@dataclasses.dataclass(frozen=True)
class Sample:
    """One JointState message: header stamp in seconds, and name -> value maps."""

    stamp_s: float
    position: dict
    velocity: dict


PGREP = ["pgrep", "-f", "ros2_control_node"]


def ros2_control_pids():
    """Pids of running ros2_control_node processes. pgrep exits 1 for "none found"; any
    other non-zero exit is pgrep's own error and must not read as "none found"."""
    p = subprocess.run(PGREP, capture_output=True, text=True)
    if p.returncode not in (0, 1):
        raise SimError(f"{' '.join(PGREP)} exited {p.returncode}: {p.stderr.strip()}")
    return p.stdout.split()


def _no_core_dumps():
    # A node that aborts during shutdown would otherwise drop a ~200 MB core file into the
    # working directory, which is the repository root.
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


@dataclasses.dataclass(frozen=True)
class FollowResult:
    accepted: bool
    error_code: int | None
    error_string: str
    # action_msgs/msg/GoalStatus value of the finished goal; None when the goal was rejected.
    # A cancelled goal still reports error_code 0, so error_code alone cannot mean success.
    status: int | None

    @property
    def succeeded(self):
        from action_msgs.msg import GoalStatus

        return self.accepted and self.status == GoalStatus.STATUS_SUCCEEDED and self.error_code == 0


def _wait(future, timeout_s, what):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout_s):
        future.cancel()
        raise SimError(f"{what}: no answer within {timeout_s} s")
    return future.result()


class Sim:
    def __init__(self, name):
        self.log_path = LOG_DIR / f"launch-{name}.log"
        self._proc = None
        self._log = None
        self._ctx = None
        self._node = None
        self._executor = None
        self._spin = None
        self._cond = threading.Condition()
        # topic -> (count of messages received, latest Sample)
        self._latest = collections.defaultdict(lambda: (0, None))

    # --- lifecycle ---

    def start(self, timeout_s=60.0):
        running = ros2_control_pids()
        if running:
            raise SimError(f"a simulator is already running: ros2_control_node pid {', '.join(running)}")
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self._log = open(self.log_path, "w")
        self._proc = subprocess.Popen(
            LAUNCH, stdin=subprocess.DEVNULL, stdout=self._log, stderr=subprocess.STDOUT,
            start_new_session=True, preexec_fn=_no_core_dumps,
        )
        self._start_node()
        deadline = time.monotonic() + timeout_s
        states = {}
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                raise SimError(f"launch exited with {self._proc.returncode} before the controllers were active")
            try:
                states = self.active_controllers(timeout_s=min(2.0, max(0.1, deadline - time.monotonic())))
            except SimError:
                states = {}
            if all(states.get(c) == "active" for c in CONTROLLERS):
                return
            time.sleep(0.2)
        raise SimError(f"controllers not active within {timeout_s} s: {states}")

    def stop(self):
        self._stop_node()
        if self._proc is None:
            return
        pgid = self._proc.pid  # start_new_session: the launch process leads its own group
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            self._proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        # Children may outlive the launch process; give them the rest of the 10 s, then kill.
        deadline = time.monotonic() + 10.0
        while _group_alive(pgid) and time.monotonic() < deadline:
            time.sleep(0.1)
        if _group_alive(pgid):
            os.killpg(pgid, signal.SIGKILL)
            self._proc.wait(timeout=5)
            deadline = time.monotonic() + 5.0
            while _group_alive(pgid) and time.monotonic() < deadline:
                time.sleep(0.1)
        self._proc = None
        self._log.close()
        assert not _group_alive(pgid), f"process group {pgid} survived SIGKILL"

    def log_tail(self, lines=40):
        try:
            text = self.log_path.read_text(errors="replace")
        except OSError as e:
            return f"(cannot read {self.log_path}: {e})"
        return "\n".join(text.splitlines()[-lines:])

    def failure_report(self, error):
        """`error`, the launch log's ERROR/FATAL lines, then its tail. A failed hardware load
        is logged once at startup and then buried under retry warnings, so the tail alone
        does not show it."""
        try:
            text = self.log_path.read_text(errors="replace")
        except OSError as e:
            return f"{error}\n(cannot read {self.log_path}: {e})"
        errors = [line for line in text.splitlines() if "[ERROR]" in line or "[FATAL]" in line]
        return (
            f"{error}\n--- launch log errors ({self.log_path}) ---\n" + "\n".join(errors[:20])
            + f"\n--- launch log tail ---\n{self.log_tail()}"
        )

    # --- queries ---

    def sample(self, topic, timeout_s=5.0):
        """The first message on `topic` received after this call."""
        with self._cond:
            seen = self._latest[topic][0]
            if not self._cond.wait_for(lambda: self._latest[topic][0] > seen, timeout_s):
                raise SimError(f"no message on {topic} within {timeout_s} s")
            return self._latest[topic][1]

    def mujoco_positions(self, timeout_s=5.0):
        return self.sample("/mujoco_actuators_states", timeout_s).position

    def mujoco_velocities(self, timeout_s=5.0):
        return self.sample("/mujoco_actuators_states", timeout_s).velocity

    def joint_states(self, timeout_s=5.0):
        return self.sample("/joint_states", timeout_s).position

    def paired_positions(self, within_s=0.02, timeout_s=5.0):
        """Fresh /joint_states and /mujoco_actuators_states positions whose stamps differ by at
        most `within_s`, retried until `timeout_s`. Returns (ros, mujoco, stamp gap in s)."""
        deadline = time.monotonic() + timeout_s
        gap = None
        while time.monotonic() < deadline:
            ros = self.sample("/joint_states", timeout_s)
            mujoco = self.sample("/mujoco_actuators_states", timeout_s)
            # Unstamped messages would pair with a gap of 0 and make the comparison vacuous.
            if ros.stamp_s == 0.0 or mujoco.stamp_s == 0.0:
                raise SimError(f"unstamped message: /joint_states {ros.stamp_s}, "
                               f"/mujoco_actuators_states {mujoco.stamp_s}")
            gap = abs(ros.stamp_s - mujoco.stamp_s)
            if gap <= within_s:
                return ros.position, mujoco.position, gap
        raise SimError(f"no sample pair within {within_s} s of each other (last gap {gap} s)")

    def active_controllers(self, timeout_s=5.0):
        from controller_manager_msgs.srv import ListControllers

        if not self._list.wait_for_service(timeout_sec=timeout_s):
            raise SimError(f"/controller_manager/list_controllers not available within {timeout_s} s")
        res = _wait(self._list.call_async(ListControllers.Request()), timeout_s, "list_controllers")
        return {c.name: c.state for c in res.controller}

    def follow(self, points, joints=JOINTS, timeout_s=30.0):
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint

        if not self._follow.wait_for_server(timeout_sec=timeout_s):
            raise SimError(f"/arm_controller/follow_joint_trajectory not available within {timeout_s} s")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        for t, positions in points:
            p = JointTrajectoryPoint(positions=[float(v) for v in positions])
            sec = int(t)
            p.time_from_start = Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))
            goal.trajectory.points.append(p)
        handle = _wait(self._follow.send_goal_async(goal), timeout_s, "send_goal")
        if not handle.accepted:
            return FollowResult(False, None, "", None)
        response = _wait(handle.get_result_async(), timeout_s, "follow_joint_trajectory result")
        return FollowResult(True, response.result.error_code, response.result.error_string, response.status)

    # --- ROS node ---

    def _start_node(self):
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from controller_manager_msgs.srv import ListControllers
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from sensor_msgs.msg import JointState

        self._ctx = rclpy.Context()
        rclpy.init(context=self._ctx)
        self._node = rclpy.create_node("sim_checks", context=self._ctx)
        for topic in ("/joint_states", "/mujoco_actuators_states"):
            self._node.create_subscription(JointState, topic, lambda m, t=topic: self._on_state(t, m), 10)
        self._list = self._node.create_client(ListControllers, "/controller_manager/list_controllers")
        self._follow = ActionClient(self._node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self._executor = MultiThreadedExecutor(context=self._ctx)
        self._executor.add_node(self._node)
        self._stopping = threading.Event()
        self._spin = threading.Thread(target=self._spin_until_stopped, daemon=True)
        self._spin.start()

    def _spin_until_stopped(self):
        while not self._stopping.is_set():
            self._executor.spin_once(timeout_sec=0.05)

    def _stop_node(self):
        if self._ctx is None:
            return
        # The spin thread must be gone before the executor shuts down: Executor.shutdown()
        # destroys the guard condition a concurrent spin() may still be waiting on, which
        # surfaces as "exception was never retrieved: cannot use Destroyable".
        self._stopping.set()
        self._spin.join(timeout=5)
        assert not self._spin.is_alive(), "executor thread did not stop"
        self._executor.shutdown()
        self._node.destroy_node()
        self._ctx.try_shutdown()
        self._ctx = None

    def _on_state(self, topic, msg):
        s = Sample(
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            dict(zip(msg.name, msg.position)),
            dict(zip(msg.name, msg.velocity)),
        )
        with self._cond:
            self._latest[topic] = (self._latest[topic][0] + 1, s)
            self._cond.notify_all()


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    return True
