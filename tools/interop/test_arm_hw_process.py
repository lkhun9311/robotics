# SPDX-License-Identifier: Apache-2.0
"""CanopenArmHw (arm_hw) under ros2_control on vcan, against vdrive_bank (E3c Task 9).

First our master as a process (canopen_master_cli, stream scenario with --target) moves the six
joints to distinct non-zero positions and exits with the drives stopped. Then ros2_control_node
loads the arm_hw/CanopenArmHw plugin (robot_bringup/config/arm_hw.urdf.xacro, robot_description
from robot_state_publisher) and joint_state_broadcaster; the plugin configures and activates, and
/joint_states must report each named joint within 1 mrad of the position the bank itself reports
for that node: its TPDO1 position actual value (0x6064, urad) captured on vcan_device, the bank's
side of the proxy. The bank's trace carries no joint position (vdrive_bank is consumed unchanged),
so its own TPDOs are the reference. No permit is present while the plugin runs, so the bank must
record no `actuated` drive event in that time.

The comparison is a pure function; two negative fixtures (feedback replaced by zeros, joints 2 and
3 swapped) show it fails, and they run without vcan. Preconditions fail with INCONCLUSIVE.
"""
import json
import os
import pathlib
import resource
import shutil
import signal
import struct
import subprocess
import threading
import time

from conftest import DEVICE_IF, HOST_IF, NODES, RELEASE, inconclusive
from test_master_processes import run_to_end, start_cli, step_done

ROOT = pathlib.Path(__file__).resolve().parents[2]
XACRO = RELEASE / "robot_bringup" / "share" / "robot_bringup" / "config" / "arm_hw.urdf.xacro"
JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
          "wrist_2_joint", "wrist_3_joint")
TARGET = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6)  # rad, joint order = node order (node_ids 1-6)
TOLERANCE = 1e-3  # rad
TPDO1 = 0x180
STREAM_MS = 2500  # The ramp to TARGET takes about 1.45 s.
ACTIVE_TIMEOUT_S = 20.0


# ---- the judge (pure) ----

def bank_positions(frames, since=None):
    """node -> the newest TPDO1 position actual value (rad) in `frames` ((t, cob, data) from the
    device side), counting only frames at or after `since`."""
    out = {}
    for t, cob, data in frames:
        node = cob - TPDO1
        if node in NODES and len(data) >= 6 and (since is None or t >= since):
            out[node] = struct.unpack_from("<i", data, 2)[0] * 1e-6
    return out


def compare(joint_states, bank, joint_nodes=None):
    """Errors (empty = pass) between a JointState (`name`, `position` lists) and the bank's own
    positions per node. Joint i of JOINTS is node NODES[i] unless `joint_nodes` maps it."""
    joint_nodes = joint_nodes or dict(zip(JOINTS, NODES))
    errors = []
    names = list(joint_states["name"])
    if sorted(names) != sorted(JOINTS):
        return [f"joint names {names} are not {list(JOINTS)}"]
    for name, position in zip(names, joint_states["position"]):
        node = joint_nodes[name]
        if node not in bank:
            errors.append(f"{name}: the bank reported no position for node {node}")
        elif abs(position - bank[node]) > TOLERANCE:
            errors.append(f"{name}: /joint_states {position:.6f} rad, bank node {node} "
                          f"{bank[node]:.6f} rad")
    return errors


def _bank_at_target():
    return {node: q for node, q in zip(NODES, TARGET)}


def _states(positions):
    return {"name": list(JOINTS), "position": list(positions)}


def test_arm_hw_comparison_passes_matching_feedback():
    assert compare(_states(TARGET), _bank_at_target()) == []


def test_arm_hw_comparison_rejects_zero_feedback():
    errors = compare(_states([0.0] * 6), _bank_at_target())
    assert len(errors) == 6, errors


def test_arm_hw_comparison_rejects_swapped_joints():
    swapped = list(TARGET)
    swapped[1], swapped[2] = swapped[2], swapped[1]
    errors = compare(_states(swapped), _bank_at_target())
    assert len(errors) == 2 and "shoulder_lift_joint" in errors[0] and "elbow_joint" in errors[1], \
        errors


# ---- the processes ----

def _no_core_dumps():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


class RosProc:
    """A ROS process whose output goes to a log file; stopped with SIGINT, then SIGKILL."""

    def __init__(self, name, argv, env, log_dir):
        self.name = name
        self.log_path = log_dir / f"{name}.log"
        self._log = open(self.log_path, "w")
        self.proc = subprocess.Popen([str(a) for a in argv], stdin=subprocess.DEVNULL,
                                     stdout=self._log, stderr=subprocess.STDOUT, env=env,
                                     start_new_session=True, preexec_fn=_no_core_dumps)

    def text(self):
        return self.log_path.read_text(errors="replace")

    def wait_for_line(self, needle, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if needle in self.text():
                return True
            if self.proc.poll() is not None:
                return needle in self.text()
            time.sleep(0.05)
        return False

    def stop(self, timeout=10):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self._log.close()
        return self.proc.returncode


def _ros_env(tmp_path):
    env = dict(os.environ)
    # Its own DDS domain on this host only: nothing else in the run sees these topics.
    env["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env["ROS_LOG_DIR"] = str(tmp_path / "ros_log")
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"
    return env


def _one_joint_state(env, timeout):
    """The first /joint_states message received by a fresh subscriber, as a dict (a separate
    process, so this test's own process never joins the DDS domain)."""
    script = (
        "import json, rclpy, sys\n"
        "from sensor_msgs.msg import JointState\n"
        "rclpy.init()\n"
        "node = rclpy.create_node('arm_hw_probe')\n"
        "got = []\n"
        "node.create_subscription(JointState, '/joint_states', lambda m: got.append(m), 10)\n"
        f"end = node.get_clock().now().nanoseconds + {int(timeout * 1e9)}\n"
        "while not got and node.get_clock().now().nanoseconds < end:\n"
        "    rclpy.spin_once(node, timeout_sec=0.1)\n"
        "if not got: sys.exit(3)\n"
        "m = got[-1]\n"
        "print(json.dumps({'name': list(m.name), 'position': list(m.position),\n"
        "                  'velocity': list(m.velocity)}))\n")
    p = subprocess.run(["python3", "-c", script], env=env, capture_output=True, text=True,
                       timeout=timeout + 10, preexec_fn=_no_core_dumps)
    if p.returncode != 0:
        inconclusive(f"no /joint_states within {timeout} s (exit {p.returncode}): {p.stderr[-500:]}")
    return json.loads(p.stdout.strip().splitlines()[-1])


def test_controller_manager_loads_canopen_arm_hw(start_bank, proxy, procs, capture, tmp_path):
    assert shutil.which("xacro"), "NOT_RUN: xacro not on PATH"
    assert XACRO.exists(), f"NOT_RUN: {XACRO} not installed"
    bank = start_bank()
    bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "bank start record")

    # 1. Our master moves the joints to TARGET and exits with the drives stopped.
    cli = start_cli(procs, "stream", "--duration-ms", str(STREAM_MS),
                          "--target", ",".join(str(q) for q in TARGET))
    rc, end = run_to_end(cli, 30)
    for step in ("configure", "activate", "arm", "deactivate"):
        done = step_done(cli, step)
        if done is None or not done["ok"]:
            inconclusive(f"the master's step {step} did not complete: {done}; {end}")
    if rc != 0 or end["event"] != "scenario_done":
        inconclusive(f"the master's stream did not end cleanly: {rc} {end}")

    # 2. CanopenArmHw under ros2_control_node, the bank's TPDOs captured on its own side.
    device = capture(DEVICE_IF)
    env = _ros_env(tmp_path)
    log_dir = tmp_path / "ros"
    log_dir.mkdir()
    urdf = subprocess.run(["xacro", str(XACRO), "plugin:=arm_hw/CanopenArmHw",
                           f"can_interface:={HOST_IF}", "node_ids:=1,2,3,4,5,6",
                           "required_capabilities:=all"],
                          capture_output=True, text=True, env=env)
    assert urdf.returncode == 0, urdf.stderr
    rsp_params = log_dir / "rsp.yaml"
    rsp_params.write_text(json.dumps(
        {"robot_state_publisher": {"ros__parameters": {"robot_description": urdf.stdout}}}))
    cm_params = log_dir / "controllers.yaml"
    cm_params.write_text(json.dumps({"controller_manager": {"ros__parameters": {
        "update_rate": 100,  # the controller rate is T_bus
        "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
    }}}))
    started = []
    t_plugin = time.monotonic()
    try:
        rsp = RosProc("robot_state_publisher", ["ros2", "run", "robot_state_publisher",
                                                "robot_state_publisher", "--ros-args",
                                                "--params-file", rsp_params], env, log_dir)
        started.append(rsp)
        cm = RosProc("ros2_control_node", ["ros2", "run", "controller_manager",
                                           "ros2_control_node", "--ros-args", "--params-file",
                                           cm_params], env, log_dir)
        started.append(cm)
        if not cm.wait_for_line("arm: active", ACTIVE_TIMEOUT_S):
            inconclusive(f"CanopenArmHw did not become active; log tail: {cm.text()[-2000:]}")
        spawner = subprocess.run(["ros2", "run", "controller_manager", "spawner",
                                  "joint_state_broadcaster", "--controller-manager",
                                  "/controller_manager", "--controller-manager-timeout", "20"],
                                 env=env, capture_output=True, text=True, timeout=60,
                                 preexec_fn=_no_core_dumps)
        if spawner.returncode != 0:
            inconclusive(f"joint_state_broadcaster did not start: {spawner.stdout[-500:]} "
                         f"{spawner.stderr[-500:]}")
        time.sleep(0.5)  # a few cycles of fresh feedback
        t_sample = time.monotonic()
        states = _one_joint_state(env, 10)
        t_states = time.monotonic()
        loaded = "Loaded hardware 'arm' from plugin 'arm_hw/CanopenArmHw'" in cm.text() or \
            "arm_hw/CanopenArmHw" in cm.text()
    finally:
        for p in reversed(started):
            p.stop()
    t_plugin_end = time.monotonic()
    frames = device.stop()
    bank.stop()
    assert loaded, f"the log does not name the plugin: {cm.text()[-1000:]}"
    print(f"/joint_states: {states}")

    # Preconditions: the bank reported a position for every node while the plugin ran, and the
    # master had placed each joint at its distinct non-zero target.
    reported = bank_positions(frames, since=t_sample - 0.5)
    if sorted(reported) != list(NODES):
        inconclusive(f"the bank sent TPDO1 for nodes {sorted(reported)} only")
    print(f"bank TPDO1 positions (rad): {reported}")
    placed = compare(_states(TARGET), reported)
    if placed:
        inconclusive(f"the master did not place the joints at {TARGET}: {placed}")
    newest = bank_positions([f for f in frames if f[0] <= t_states])
    errors = compare(states, newest)
    assert errors == [], errors

    # No permit was present: nothing may have been actuated while the plugin ran.
    drive = [r for r in bank.trace.records if r.get("event") == "drive"]
    if not [r for r in drive if r.get("kind") == "actuated" and r["t_ns"] / 1e9 < t_plugin]:
        inconclusive("the bank trace shows no actuation by the master's stream at all")
    actuated = [r for r in drive if r.get("kind") == "actuated"
                and t_plugin <= r["t_ns"] / 1e9 <= t_plugin_end]
    assert not actuated, f"actuated while the plugin ran without a permit: {actuated[:3]}"
