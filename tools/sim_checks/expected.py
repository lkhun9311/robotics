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

# Per-joint offset from HOME used by the trajectory checks, in JOINTS order. Every joint moves,
# and the elbow moves away from its +3.1415 limit.
TARGET_OFFSET = (0.5, 0.3, -0.4, 0.3, 0.5, 0.8)
TARGET = tuple(h + d for h, d in zip(HOME, TARGET_OFFSET))

# Pick-and-place-shaped waypoints, in JOINTS order: approach above the pick spot, down to it,
# lift, approach above the place spot, down to it. Chosen during design with a damped
# least-squares search on the MuJoCo model for the attachment_site 0.095 m above (0.30, 0.40)
# and (-0.45, 0.25) m (0.25 m for the approaches, 0.30 m for LIFT) with the tool axis pointing
# down, then rounded to 1 mrad with wrist_3 at 0.
# They are inputs, not results: the simulation check measures where MuJoCo puts the site.
PICK_APPROACH = (-2.486, -1.414, 2.050, -2.207, -1.571, 0.0)
PICK = (-2.486, -1.092, 2.169, -2.648, -1.571, 0.0)
LIFT = (-2.486, -1.484, 1.979, -2.066, -1.571, 0.0)
PLACE_APPROACH = (-0.770, -1.382, 2.010, -2.199, -1.571, 0.0)
PLACE = (-0.770, -1.071, 2.126, -2.626, -1.571, 0.0)

# The scene has no table: the floor plane the arm stands on (geom "floor", z = 0) is the work
# surface.
TABLE_Z = 0.0
# The demo's object (a sphere of radius 0.04 m) hangs below the attachment_site with its centre
# 0.045 m down the tool axis, so with the tool pointing down its lowest point is 0.085 m below
# the site. It is a drawing in the replay render only; nothing in the simulation grasps it.
OBJECT_RADIUS = 0.04
OBJECT_BELOW_SITE = 0.045
OBJECT_HANG = OBJECT_BELOW_SITE + OBJECT_RADIUS
