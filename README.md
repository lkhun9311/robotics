# robotics

![ci](https://github.com/lkhun9311/robotics/actions/workflows/ci.yml/badge.svg?branch=dev)

A simulation-based robot system runtime, built in stages.

- **Done (E0, E1, E2):** a pinned ROS 2 Jazzy toolchain with CI, a cyclictest baseline, a
  ROS-free C++ core of command and permit contracts, checked by an independent judge in
  logic time, and the UR5e in MuJoCo under the stock JointTrajectoryController (no permit or
  generation guarantee).
- **In progress (E3):** E3a, the CANopen wire contract and a ROS-free drive core, is done (see
  below).
- **Planned (E3b onward):** a C++ hardware abstraction layer, a partial CANopen / CiA 402
  stack over virtual CAN, fault injection and real-time loop measurement, on ros2_control
  and MuJoCo.

## Status

| Stage | State |
|---|---|
| E0 environment, dev and runtime images, CI | partial: toolchain, CI, environment manifest, cyclictest baseline, and the MuJoCo plugin and Menagerie model done; CANopen peers, supervisor, DDS, Zephyr and PREEMPT_RT checks are open and run in the stage that needs them |
| E1 command and permit contracts, independent judge | done |
| E2 official plugin baseline | done: stock JTC baseline; no permit/generation guarantee |
| E3 | in progress: E3a wire contract and drive core done; E3b-E3d planned |

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

## E3a wire contract and drive core

E3a fixes the CAN wire contract of the virtual drive and implements the drive side in ROS-free
C++ (`src/canopen_codec`, `src/drive_core`). It runs in logic time; no bus is involved yet.

What the contract fixes:

- One EDS (`config/od/vdrive_arm.eds`) is the single source of the object dictionary; the C++
  header is generated from it.
- Every executed command carries a tag issued by the drive. A master obtains the tag through an
  SDO commit that is bound by a CRC; each command is two PDO fragments bound to that tag and a
  sequence number, and the drive pairs fragments only on equal tag and sequence.
- A command's validity is bounded by a lease of at most 48 ms. The drive never actuates an
  expired command, and a tag lives at most 5 minutes.
- SYNC timeline: RPDOs received between SYNC k and k+1 are actuated at SYNC k+1; TPDOs are
  sampled at SYNC k. Loss of SYNC or of commands, an integrity violation or leaving NMT
  Operational starts a quick stop.
- Quick stop is an achieved deceleration ramp, not a switch of state.
- A policy stop sets a latch that survives reset node and reset communication and clears only
  through an explicit policy reset.
- Fault layers separate controlled faults from torque loss; after torque loss the drive stays in
  Fault until the process restarts.
- Commands reach the drive by PDO only. 0x6040, 0x607A, 0x2020 and 0x2021 are `rww` in the EDS
  so that RPDOs can map them, but an SDO write to them is refused with abort 0x06010000.
  Parameters the drive never changes at run time (0x1005, 0x1017, the PDO COB-IDs and
  transmission types, 0x605A, 0x605E, 0x6085) are `const`, so a write is refused as read-only.
- The SDO server serves expedited transfers only. A segmented or block request, or any other
  client command specifier it does not serve, is answered with abort 0x05040001. An expedited
  download with the size indicator clear (command byte 0x22) is accepted: the object's own size
  applies, and data bytes beyond it must be zero.

Which tool checks what:

| Check | Tool | What it covers |
|---|---|---|
| Wire bytes | Python wire encoder (`tools/od/wire.py`), independent of the C++ codec and of the shared EDS loader | Golden vectors (`tools/od/make_golden.py`) that the C++ codec must reproduce byte for byte |
| EDS syntax | Lely `dcfchk` 2.3.5 (`tools/od/tests/test_dcfchk.py`) | Catches sections, keys, access types, value syntax and ranges, sub-object counts. Misses PDO mappings that are 72 bits long, map an unmappable object, or map a read-only object into an RPDO |
| EDS contract | project validator and `tools/od/tests/test_od.py` | The PDO mapping rules `dcfchk` misses, and every object, type, default and PDO layout of the contract, restated by hand in the test |
| CiA 402 transitions | table written from the standard text, in the drive core tests | The transitions of the state machine |
| Bandwidth | `tools/od/bandwidth.py` | Frame count and occupancy computed from the EDS |

No independent CiA 402 implementation was available, so the CiA 402 state machine is checked
only against the project's table from the standard text. Interop with Lely and with CANopenNode
is E3b and E3c and covers the standard part only.

Measured and computed:

- Total travel after a command stops being valid, with the validity deadline swept over 20
  phases of one control tick W (`TotalTravelWithinBoundAcrossPhases` in
  `src/drive_core/test/test_drive_loop.cpp`, which pins the value): maximum 0.097 rad (24 ticks
  at v_max, 0.048 rad, plus the braking ramp from v_max, 0.049 rad) against an allowance of 0.100 rad
  (lease 48 ms at v_max 1 rad/s, a_stop 10 rad/s^2). These values are project-declared, not
  industry values.
- Bandwidth (`python3 tools/od/bandwidth.py config/od/vdrive_arm.eds`): per 10 ms bus cycle
  1 SYNC + 6 x (2 RPDO + 3 TPDO) = 31 frames; RPDO3 is event-driven and not counted. At the
  worst-case 135 bits per 8-byte frame (47 + 8n + floor((34 + 8n - 1)/4) with stuffing) the
  cyclic load is 4185 bits per cycle, 41.85 % of 1 Mbit/s (41.05 % with the real DLC of SYNC).
  With 6 heartbeats per 100 ms and one SDO request and response per cycle it is 44.94 %. This
  is a computed bound: vcan does not show physical arbitration delay.

Residual risks and deviations:

1. Across a drive reboot, a fragment that survived in some queue through the 100 ms quarantine
   could carry a tag equal to one issued by the new boot (the tag starts pseudo-randomly per
   boot within a process, about 2^-16). If its sequence also matches and the fresh RPDO1 of
   that sequence is lost, an old target could pair with a fresh RPDO2. In the project harness
   the bus proxy bounds delay so this cannot occur there.
2. The tag start of a boot is spread uniformly over 1..65535 (`exec_tag_start`), and a boot can
   issue the tags from its start up to 65535, that is 65536 - start tags. Fewer than 100 tags
   are left exactly when the start is above 65436: 99 of 65535 values, about 0.151 % of boots.
   Such a boot runs out of tags sooner and then needs a reset node (availability, not safety).
3. With the policy latch set, Enable Operation in Ready to Switch On is ignored instead of
   taking only transition 3. This is intended and conservative.
4. Quick-stop option 6 with transition 16 disabled while latched is a documented project
   restriction.
5. SDO writes to command objects are refused; commands travel by PDO only.
6. A tag lives at most 5 minutes of drive time, even while commands stream, so trajectories
   longer than 5 minutes are out of scope. When the tag expires mid-stream, every later fragment
   is of a dead tag, nothing actuates, and the command timeout starts a policy stop
   (`Lifetime.StreamPastTagLifetimeEndsInTimeoutStop`). A commit is refused in Operation Enabled
   while a command is active, so renewing the tag takes Disable Operation, a new commit and
   Enable Operation.
7. drive_core produces no heartbeat and no EMCY. 0x1017 states the 100 ms heartbeat period that
   the process hosting the drive is to produce (E3b).

Decision index (the `D<n>` references in code comments):

| # | Decision |
|---|---|
| D1 | The EDS is the single source of the object dictionary; Lely `dcfchk` lints it, a generator writes the C++ header |
| D2 | Units: int32 µrad for positions, int32 µrad/s for velocity |
| D3 | The drive issues a 16-bit exec_tag per commit from a per-boot counter; a tag lives at most 5 min; commits are refused for 100 ms after boot |
| D4 | RPDO1 (target, seq, tag) and RPDO2 (valid_until_us, seq, tag) are 8 bytes; RPDO3 is the controlword alone, 2 bytes; a frame whose DLC differs from its mapping is rejected |
| D5 | valid_until_us is host monotonic µs mod 2^32; accepted only if 0 < (until - now) mod 2^32 <= 48 000 |
| D6 | Replay safety comes from D3 and D5; RPDO1 actuates only paired with an open RPDO2 of the same tag and seq |
| D7 | RPDOs received between SYNC k and k+1 actuate at SYNC k+1; TPDOs are sampled at SYNC k; no SYNC for 3 T_bus stops; a SYNC sooner than T_bus/2 is ignored |
| D8 | Fragments pair on equal tag and seq; the newest complete pair actuates; two payloads for one seq are an integrity violation, detected at ingestion, stopping at the next W tick |
| D9 | Validity is checked at actuation and every W tick; no actuated command for 30 ms in Operation Enabled stops |
| D10 | Quick stop: option 6, a_stop 10 rad/s^2 achieved exactly; stop confirmed after 5 quiet ticks, unconfirmed after 0.112 s |
| D11 | Policy stops latch until an explicit policy reset after a confirmed stop; normal disable needs no reset; torque loss is Fault without a ramp |
| D12 | NMT service matrix; reset communication keeps boot_id and the tag allocator; leaving Operational while Operation Enabled is a policy stop |
| D13 | TPDO1-3 carry statusword, position, velocity, sample time and the applied seq and tag, joined by sample_seq |
| D14 | Manufacturer objects 0x2010-0x2030 hold every wire field |
| D15 | Bandwidth: 31 frames per 10 ms bus cycle |
| D16 | One CMake include sets warnings, sanitizers and coverage for every pure C++ package |
| D17 | Commit transaction: attempt_id, staged values, then a CRC-32 commit; idempotent repeat; 0x2016 confirms which attempt holds the tag |
| D18 | SYNC on 0x80, EMCY on 0x80 + node-ID, read from the EDS |

## Run everything

    docker compose build dev
    docker compose run --rm dev tools/ci/run_checks.sh

## Layout

- `src/runtime_core` ROS-free C++ contracts, GoogleTest, logic-time harness
- `tools/oracle` independent trace judge
- `config/od` the drive EDS, the one source of the object dictionary
- `tools/od` EDS loader and validator, header generator, independent wire encoder, golden vectors, bandwidth
- `src/canopen_codec` PDO and SDO codec, checked against the golden vectors
- `src/drive_core` ROS-free drive: CiA 402 state machine, SYNC timeline, guards, quick stop, SDO server
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
