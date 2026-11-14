# SPDX-License-Identifier: Apache-2.0
"""Checks of the running bringup: controllers, joint states, and the start pose.

One simulator serves the whole module (the module-scoped `sim` fixture in conftest.py).
"""
import time

from expected import GOAL_TOLERANCE, HOME, JOINTS

# Largest |velocity| (rad/s) that counts as standing still; the controller's stopped tolerance.
STOPPED_VELOCITY = 0.05


def test_controllers_are_active(sim):
    states = sim.active_controllers()
    for name in ("joint_state_broadcaster", "arm_controller"):
        assert states.get(name) == "active", f"{name}: {states.get(name)!r} in {states}"


def test_joint_states_cover_exactly_the_six_joints(sim):
    assert set(sim.joint_states()) == set(JOINTS)


def test_ros2_control_positions_are_mujoco_positions(sim):
    ros, mujoco, gap_s = sim.paired_positions(within_s=0.02)
    assert gap_s <= 0.02, f"samples {gap_s * 1e3:.1f} ms apart"
    for j in JOINTS:
        assert abs(ros[j] - mujoco[j]) < 1e-3, f"{j}: joint_states {ros[j]} vs MuJoCo {mujoco[j]}"


def test_arm_starts_at_the_home_keyframe(sim):
    time.sleep(1.0)  # settle
    # Position and velocity of the same instant: both from one message.
    state = sim.mujoco_state()
    pos, vel = state.position, state.velocity
    for j, home in zip(JOINTS, HOME):
        assert abs(pos[j] - home) <= GOAL_TOLERANCE, f"{j}: {pos[j]} vs home {home}"
        assert abs(vel[j]) < STOPPED_VELOCITY, f"{j}: velocity {vel[j]}"
