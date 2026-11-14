# SPDX-License-Identifier: Apache-2.0
"""Values the simulation checks compare against.

Written by hand from the Menagerie universal_robots_ur5e model at the pinned commit (ur5e.xml:
joint and actuator defaults, worldbody order, keyframe "home"), never computed from the files
under test, so a change to the installed model or the URDF shows up as a mismatch here.
"""

JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# Keyframe "home" qpos, in JOINTS order.
HOME = (-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0)

# Joint range from class "ur5e"; elbow_joint uses class "size3_limited".
RANGE = {j: (-6.28319, 6.28319) for j in JOINTS} | {"elbow_joint": (-3.1415, 3.1415)}

# Largest per-joint position error (rad) accepted at the end of a trajectory.
GOAL_TOLERANCE = 0.05
