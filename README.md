# robotics

![ci](https://github.com/lkhun9311/robotics/actions/workflows/ci.yml/badge.svg?branch=dev)

A simulation-based robot system runtime, built in stages.

- **Done (E0, E1):** a pinned ROS 2 Jazzy toolchain with CI, a cyclictest baseline, and a
  ROS-free C++ core of command and permit contracts, checked by an independent judge in
  logic time.
- **Planned (E2 onward):** a C++ hardware abstraction layer, a partial CANopen / CiA 402
  stack over virtual CAN, fault injection and real-time loop measurement, on ros2_control
  and MuJoCo.

## Status

| Stage | State |
|---|---|
| E0 environment, dev and runtime images, CI | partial: toolchain, CI, environment manifest and cyclictest baseline done; MuJoCo plugin, CANopen peers, supervisor, DDS, Zephyr and PREEMPT_RT checks are open and run in the stage that needs them |
| E1 command and permit contracts, independent judge | done |
| E2 onward | not started |

E2 (in progress), measured with the stock JointTrajectoryController 4.42.1 on the MuJoCo UR5e: a
goal of 6.5 rad for `wrist_3_joint` was accepted, passed through unchanged as the controller
output, and aborted with GOAL_TOLERANCE_VIOLATED (-5), while MuJoCo held the joint at or below
its 6.28319 rad limit. From the home pose the elbow cannot reach its 3.1415 rad limit: the arm
contacts itself at about 2.89 rad, and JTC then aborts the goal with GOAL_TOLERANCE_VIOLATED (-5).

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
- `tools/env`, `tools/bench` environment manifest and cyclictest baseline
- `docker`, `compose.yaml`, `.github/workflows` toolchain, images and CI

## Limits

Logic time only: no OS jitter, no real IPC, no CAN, no physics yet. The numeric limits are
project-declared, not industry values.
