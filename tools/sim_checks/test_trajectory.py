# SPDX-License-Identifier: Apache-2.0
"""Trajectory goals through the stock JointTrajectoryController, judged on MuJoCo's joints."""
import time

import pytest

from expected import GOAL_TOLERANCE, HOME, JOINTS, RANGE, TARGET
from sim import CONTROLLER_STATE, MUJOCO_STATES, SimError

STEP = 0.2  # rad, single-joint move in the mapping check

# The out-of-range goal moves wrist_3: from HOME it reaches within GOAL_TOLERANCE of its +6.28319
# limit. The elbow does not: under the plugin it stopped at about 2.89 rad, short of +3.1415, and
# a standalone MuJoCo 4.0.1 run of the same scene showed the elbow actuator at its force limit
# and upper_arm_link touching wrist_3_link (measured 2026-11-14).
OUT_OF_RANGE_JOINT = "wrist_3_joint"
OUT_OF_RANGE_GOAL = 6.5  # rad, beyond RANGE (6.28319) and the actuator ctrlrange (6.2831)
# Largest allowed gap between the header stamps of consecutive recorded /mujoco_actuators_states
# messages. It bounds the recorded data's coverage in simulation time, independent of when the
# test process happened to receive the messages.
MAX_STAMP_GAP_S = 0.1
# How long after the action result the recording runs, in simulation time.
AFTER_RESULT_S = 2.0
# Observed with JTC 4.42.1: the goal is accepted, its value is passed through unchanged as the
# controller output, and the goal ends ABORTED with GOAL_TOLERANCE_VIOLATED once goal_time
# (1.0 s) runs out with the joint held short of the goal.
OUT_OF_RANGE_ERROR_CODE = -5  # control_msgs FollowJointTrajectory GOAL_TOLERANCE_VIOLATED


def _go_home(sim):
    r = sim.follow([(3.0, HOME)])
    assert r.succeeded, f"return to HOME failed: {r}"


def _require_coverage(topic, samples, sent, result, lag_s):
    """`samples` start no later than 0.1 s after `sent`, end at least 1.5 s after `result`, and
    no two consecutive header stamps are more than MAX_STAMP_GAP_S apart."""
    stamps = [s.stamp_s for _, s in samples]
    gap = max((b - a for a, b in zip(stamps, stamps[1:])), default=float("inf"))
    received = [t for t, _ in samples]
    receive_gap = max((b - a for a, b in zip(received, received[1:])), default=float("inf"))
    # Captured by pytest: shown only with -rP or when the test fails.
    print(f"{topic}: first stamp {stamps[0] - sent:+.4f} s after send, last {stamps[-1] - result:+.4f} s "
          f"after result, largest stamp gap {gap:.4f} s, largest receive gap {receive_gap:.4f} s, "
          f"largest executor lag {lag_s:.4f} s, {len(stamps)} samples")
    assert stamps[0] <= sent + 0.1 and stamps[-1] >= result + 1.5 and gap <= MAX_STAMP_GAP_S, (
        f"{topic}: samples do not span the interval in simulation time: first stamp "
        f"{stamps[0] - sent:+.3f} s after send, last {stamps[-1] - result:+.3f} s after result, largest "
        f"stamp gap {gap:.3f} s (diagnostic: largest receive gap {receive_gap:.3f} s wall time, largest "
        f"executor lag {lag_s:.3f} s, {len(stamps)} samples)")


def _errors(positions, goal):
    return {j: abs(positions[j] - g) for j, g in zip(JOINTS, goal)}


def test_normal_trajectory_reaches_its_goal(sim):
    _go_home(sim)
    before = sim.mujoco_positions()
    # The goal is far from the start, so reaching it within tolerance means the arm moved.
    assert all(e > 2 * GOAL_TOLERANCE for e in _errors(before, TARGET).values()), before
    midpoint = tuple((h + t) / 2 for h, t in zip(HOME, TARGET))
    r = sim.follow([(2.0, midpoint), (4.0, TARGET)])
    assert r.succeeded, f"goal did not succeed: {r}"
    errors = _errors(sim.mujoco_positions(), TARGET)
    assert all(e <= GOAL_TOLERANCE for e in errors.values()), errors


def test_each_joint_moves_only_its_own_mujoco_joint(sim):
    for i, joint in enumerate(JOINTS):
        _go_home(sim)
        before = sim.mujoco_positions()
        goal = list(HOME)
        goal[i] += STEP
        r = sim.follow([(2.0, goal)])
        assert r.succeeded, f"{joint}: goal did not succeed: {r}"
        after = sim.mujoco_positions()
        moved = {j: after[j] - before[j] for j in JOINTS}
        assert abs(moved[joint] - STEP) <= GOAL_TOLERANCE, f"{joint} moved {moved[joint]:+.4f}, not {STEP:+}: {moved}"
        others = {j: d for j, d in moved.items() if j != joint and abs(d) >= 0.05}
        assert not others, f"moving {joint} also moved {others}"
    _go_home(sim)


def test_out_of_range_goal_never_leaves_the_joint_range(sim):
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory

    assert FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED == OUT_OF_RANGE_ERROR_CODE
    joint = OUT_OF_RANGE_JOINT
    _go_home(sim)
    goal = list(HOME)
    goal[JOINTS.index(joint)] = OUT_OF_RANGE_GOAL
    sim.start_recording()
    # Send and result moments in simulation time: the stamp of a /mujoco_actuators_states
    # message published after that wall-time moment, so a backlog in this process's queue
    # cannot move them earlier.
    sent = sim.stamp_now()
    r = sim.follow([(3.0, goal)])
    result = sim.stamp_now()
    rec = sim.stop_recording(until_stamp=result + AFTER_RESULT_S)
    mujoco = rec.get(MUJOCO_STATES, [])
    state = rec.get(CONTROLLER_STATE, [])
    assert mujoco and state, f"no samples: mujoco {len(mujoco)}, controller_state {len(state)}"
    # Both recordings must span the interval in simulation time: the position checks below read
    # /mujoco_actuators_states, the output check reads controller_state, and a recording with a
    # hole could pass either vacuously.
    for topic, samples in ((MUJOCO_STATES, mujoco), (CONTROLLER_STATE, state)):
        _require_coverage(topic, samples, sent, result, sim.recording_lag_s)
    positions = [s.position[joint] for _, s in mujoco]
    upper = RANGE[joint][1]
    # An accepted goal that never executes must not pass: the joint has to reach the boundary.
    assert max(positions) >= upper - GOAL_TOLERANCE, f"{joint} reached only {max(positions):.4f}, result {r}"
    assert max(positions) <= upper + 1e-3, f"{joint} reached {max(positions):.4f} > {upper}"
    # The goal names wrist_3 only; every other joint holds HOME throughout.
    drift = {j: max(abs(s.position[j] - h) for _, s in mujoco) for j, h in zip(JOINTS, HOME) if j != joint}
    print(f"largest drift from HOME of the other joints: {max(drift.values()):.4f} rad")
    moved = {j: round(d, 4) for j, d in drift.items() if d > GOAL_TOLERANCE}
    assert not moved, f"joints other than {joint} left HOME by more than {GOAL_TOLERANCE} rad: {moved}"
    assert (r.accepted, r.status, r.error_code) == (True, GoalStatus.STATUS_ABORTED, OUT_OF_RANGE_ERROR_CODE), r
    output = max(s.joints[joint][1] for _, s in state)
    # Exact equality on purpose: the controller copies the goal value into its output; it is not
    # computed. 6.5 survives the double round trip unchanged, so any difference means the
    # controller altered the goal (clamped or interpolated it), which is what this check is for.
    assert output == OUT_OF_RANGE_GOAL, f"controller output max {output}, expected the goal passed through"


def test_controller_joint_order_does_not_change_named_motion(sim):
    _go_home(sim)
    before = sim.mujoco_positions()
    r = sim.follow([(4.0, tuple(reversed(TARGET)))], joints=tuple(reversed(JOINTS)))
    assert r.succeeded, f"goal did not succeed: {r}"
    after = sim.mujoco_positions()
    errors = _errors(after, TARGET)
    assert all(e <= GOAL_TOLERANCE for e in errors.values()), errors
    assert all(abs(after[j] - before[j]) > 2 * GOAL_TOLERANCE for j in JOINTS), (before, after)


def test_follow_timeout_cancels_the_goal(sim):
    _go_home(sim)
    duration_s = 4.0
    sent = time.monotonic()
    with pytest.raises(SimError, match="result: no answer .*; the goal was cancelled"):
        sim.follow([(duration_s, TARGET)], timeout_s=1.0)
    # Past the end of the trajectory: an uncancelled goal has reached TARGET by now.
    time.sleep(max(0.0, sent + duration_s + 0.5 - time.monotonic()))
    at = sim.mujoco_positions()
    # Cancelled about a quarter of the way in time, the arm holds a pose nearer HOME than TARGET.
    nearer_target = [j for j, h, t in zip(JOINTS, HOME, TARGET) if abs(at[j] - t) <= abs(at[j] - h)]
    assert not nearer_target, f"the timed-out goal kept running: {nearer_target} reached toward TARGET: {at}"
    _go_home(sim)
