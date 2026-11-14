# robotics

![ci](https://github.com/lkhun9311/robotics/actions/workflows/ci.yml/badge.svg?branch=dev)

A simulation-based robot system runtime, built in stages.

- **Done (E0, E1, E2):** a pinned ROS 2 Jazzy toolchain with CI, a cyclictest baseline, a
  ROS-free C++ core of command and permit contracts, checked by an independent judge in
  logic time, and the UR5e in MuJoCo under the stock JointTrajectoryController (no permit or
  generation guarantee).
- **Planned (E3 onward):** a C++ hardware abstraction layer, a partial CANopen / CiA 402
  stack over virtual CAN, fault injection and real-time loop measurement, on ros2_control
  and MuJoCo.

## Status

| Stage | State |
|---|---|
| E0 environment, dev and runtime images, CI | partial: toolchain, CI, environment manifest, cyclictest baseline, and the MuJoCo plugin and Menagerie model done; CANopen peers, supervisor, DDS, Zephyr and PREEMPT_RT checks are open and run in the stage that needs them |
| E1 command and permit contracts, independent judge | done |
| E2 official plugin baseline | done: stock JTC baseline; no permit/generation guarantee |
| E3 onward | not started |

## E2 baseline

The Menagerie UR5e in MuJoCo under ros2_control, driven by the stock JointTrajectoryController.
After `docker compose run --rm dev tools/ci/run_checks.sh` has built the workspace:

    docker compose run --rm dev bash -c 'source install/release/setup.bash && ros2 launch robot_bringup ur5e_baseline.launch.py'

The launch is headless by default; `headless:=false` opens the MuJoCo viewer (needs a display).
Pass `camera:=true` to publish the `overview` camera on `/overview/image_raw`. The recording [docs/media/e2-baseline.mp4](docs/media/e2-baseline.mp4)
(640x480, from that camera; `tools/sim_checks/record_motion.py`) shows the arm leaving the home pose
and returning to it.

The simulation checks (`tools/sim_checks`) show that:

- the joints, actuators and controller joints map to each other by name;
- the arm starts at the home pose;
- a normal goal is accepted and reached within 0.05 rad at its end;
- out-of-range goals behave as measured below.

Out-of-range behaviour, measured with the stock JointTrajectoryController 4.42.1 on the MuJoCo UR5e:
A goal of 6.5 rad for `wrist_3_joint` was accepted, passed through unchanged as the controller
output, and aborted with GOAL_TOLERANCE_VIOLATED (-5), while the MuJoCo joint stayed at or below
6.2815 rad. The actuator `ctrlrange` (6.2831) is below the joint `range` (6.28319), so this run
does not show which of the two held the joint; the project HAL (E3) owns limit enforcement. A
goal of 3.5 rad for `elbow_joint` from the home pose was also passed through and aborted with
GOAL_TOLERANCE_VIOLATED (-5); the elbow stopped at about 2.89 rad, short of its 3.1415 rad limit.
A standalone MuJoCo 4.0.1 run of the same scene showed the elbow actuator at its force limit and
`upper_arm_link` touching `wrist_3_link`.

Pinned versions: ROS snapshot 2026-10-05, `mujoco_ros2_control` 0.1.2,
`joint_trajectory_controller` 4.42.1, MuJoCo Menagerie commit
`370c43965323c0944b433b202aac5058b5f907a9` (files verified by SHA-256).

This path uses the stock JointTrajectoryController. It accepts goals on its own action and topic interfaces, and nothing outside it can prove which goal produced a given output, so E2 claims no permit or generation guarantee. Those claims start with the project's own controller in E3.

## What E1 proves today (logic time, no robot, no ROS)

- A command is applied only while `now < valid_until` as it was sent, and never outlives its
  motion permit.
- A permit is renewed only on observed controller progress and fresh feedback, only by the
  manager that issued it; an expired permit is never renewed.
- Output from an old trajectory generation is blocked after a new execution starts.
- After an injected fault, motion resumes only after an explicit Reset and then an explicit
  Enable, with a new session and generation.
- Each backend profile states, per guarantee, whether the device supports it, the host
  provides it, it does not apply, or it is unknown or unsupported; `missing_capabilities()`
  lists what a backend lacks for a given set of requirements. Nothing refuses activation on
  that list yet: the activation path arrives with the project's own HAL (E3).

The judge (`tools/oracle`) is written in Python and does not import the C++ code. It judges
the command as sent, the faults the scenario injected, and evidence that each scenario reached
the path it claims to test. Each rule has a deliberate defect in the harness that it must catch:

| Scenario | Defect | Judge result |
|---|---|---|
| manager stall | expiry off by one | FAIL |
| generation switch | generation check skipped | FAIL |
| fault, reset, re-enable | enable without request | FAIL |
| fault, reset, re-enable | old session and generation reused | FAIL |
| fault, reset, re-enable | pre-fault frame redelivered and applied | FAIL |
| normal | apply events dropped | INCONCLUSIVE |

## Run everything

    docker compose build dev
    docker compose run --rm dev tools/ci/run_checks.sh

## Layout

- `src/runtime_core` ROS-free C++ contracts, GoogleTest, logic-time harness
- `tools/oracle` independent trace judge
- `src/ur5e_description` UR5e model, scene and ros2_control description
- `src/robot_bringup` launch file and controller and plugin configuration
- `models/` checksums of the pinned Menagerie model; `tools/models` fetch and verify
- `tools/sim_checks` simulation checks and the motion recorder
- `tools/env`, `tools/bench` environment manifest and cyclictest baseline
- `docker`, `compose.yaml`, `.github/workflows` toolchain, images and CI

## Limits

E1 runs in logic time only: no OS jitter, no real IPC, no CAN. E2 adds MuJoCo physics, but its
timing is the simulator's, not a real-time measurement. The numeric limits are project-declared,
not industry values. Ubuntu archive packages in the image are not pinned to a snapshot.
