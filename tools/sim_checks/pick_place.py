# SPDX-License-Identifier: Apache-2.0
"""The pick-and-place-shaped motion: its schedule and how a recording of it is judged.

The goal goes HOME -> PICK_APPROACH -> PICK -> LIFT -> PLACE_APPROACH -> PLACE as one
FollowJointTrajectory goal. Each waypoint is a pair of points, arrive and leave, so the arm dwells
there; the dwell is where "reached" is judged. The demo (record_motion.py --demo pick_place)
appends a retreat to PLACE_APPROACH and HOME after the same points.

There is no grasp: nothing in the simulation holds an object. The replay render
(tools/evidence/pick_place_replay.py) draws one at the tool site while it is carried, from the
lowest site point of the PICK dwell to that of the PLACE dwell.
"""
import dataclasses

from expected import GOAL_TOLERANCE, HOME, JOINTS, LIFT, OBJECT_HANG, PICK, PICK_APPROACH, PLACE, PLACE_APPROACH, TABLE_Z

WAYPOINTS = {"PICK_APPROACH": PICK_APPROACH, "PICK": PICK, "LIFT": LIFT, "PLACE_APPROACH": PLACE_APPROACH,
             "PLACE": PLACE, "HOME": HOME}
# (waypoint, arrive s, leave s) after the goal is sent.
SCHEDULE = (
    ("PICK_APPROACH", 3.0, 3.5),
    ("PICK", 5.0, 6.0),
    ("LIFT", 7.5, 8.0),
    ("PLACE_APPROACH", 11.0, 11.5),
    ("PLACE", 13.0, 14.0),
)
DEMO_SCHEDULE = SCHEDULE + (("PLACE_APPROACH", 15.5, 16.0), ("HOME", 19.0, 20.0))
# A sample stamped up to this long after a dwell ends still counts for that waypoint: the goal
# starts when the controller accepts it, a few milliseconds of simulation time after `sent`.
WINDOW_SLACK_S = 0.2
# Lowest attachment_site height allowed while LIFT -> PLACE_APPROACH carries the object.
CARRY_MIN_Z = 0.20
# PICK and PLACE bring the object down to the table: during their dwell the site comes at least
# this low (the object's lowest point within 0.02 m of the table; the waypoints aim at 0.01 m).
GRASP_MAX_Z = TABLE_Z + OBJECT_HANG + 0.02
# Between PICK and PLACE the object may touch the table but not go into it by more than this.
CONTACT_ALLOWANCE = 0.01


def points(schedule=SCHEDULE):
    """[(time from start s, positions in JOINTS order)]: arrive and leave per waypoint."""
    out = []
    for name, arrive, leave in schedule:
        out += [(arrive, WAYPOINTS[name]), (leave, WAYPOINTS[name])]
    return out


@dataclasses.dataclass(frozen=True)
class Reach:
    name: str
    best_error: float  # smallest over the window of the largest per-joint error, rad
    reached_s: float | None  # first stamp (s after send) with every joint within GOAL_TOLERANCE
    lowest_z: float | None  # lowest attachment_site height in the window, m
    lowest_s: float | None  # its stamp, s after send
    samples: int  # samples in the window


def judge(samples, sent, site_z, schedule=SCHEDULE):
    """`samples`: [(stamp s, joint -> position)] in stamp order; `sent`: stamp of the send;
    `site_z(positions)`: MuJoCo's site height. Returns (per-waypoint Reach list, {window: min
    site z}) with windows "pick_to_place" (PICK reached to PLACE reached) and "carry" (LIFT reached
    to PLACE_APPROACH reached). A window whose ends were not reached is absent."""
    rel = [(s - sent, p) for s, p in samples]
    reaches = []
    for name, arrive, leave in schedule:
        goal = WAYPOINTS[name]
        window = [(t, p) for t, p in rel if arrive <= t <= leave + WINDOW_SLACK_S]
        errors = [(t, max(abs(p[j] - g) for j, g in zip(JOINTS, goal)), p) for t, p in window]
        best = min((e for _, e, _ in errors), default=float("inf"))
        reached = next((t for t, e, _ in errors if e <= GOAL_TOLERANCE), None)
        low = min(((site_z(p), t) for t, p in window), default=(None, None))
        reaches.append(Reach(name, best, reached, low[0], low[1], len(window)))
    by = {}
    for r in reaches:
        by.setdefault(r.name, r)  # the first visit: the demo's retreat revisits PLACE_APPROACH
    heights = {}
    for key, a, b in (("pick_to_place", "PICK", "PLACE"), ("carry", "LIFT", "PLACE_APPROACH")):
        if a in by and b in by and by[a].reached_s is not None and by[b].reached_s is not None:
            zs = [site_z(p) for t, p in rel if by[a].reached_s <= t <= by[b].reached_s]
            heights[key] = min(zs) if zs else None
    return reaches, heights


def failures(reaches, heights):
    """What the simulation check would reject in a judged recording, as messages; empty when the
    recording passes. The check itself (test_pick_place_trajectory_reaches_each_waypoint) states
    the same conditions as assertions."""
    out = [f"{x.name} not reached within {GOAL_TOLERANCE} rad (best {x.best_error:.4f})"
           for x in reaches if x.reached_s is None]
    by = {}
    for x in reaches:
        by.setdefault(x.name, x)
    for name in ("PICK", "PLACE"):
        if name in by and (by[name].lowest_z is None or by[name].lowest_z > GRASP_MAX_Z):
            out.append(f"{name}: lowest site z {by[name].lowest_z} above {GRASP_MAX_Z}")
    if heights.get("pick_to_place") is None or heights["pick_to_place"] - OBJECT_HANG < TABLE_Z - CONTACT_ALLOWANCE:
        out.append(f"PICK to PLACE: lowest site z {heights.get('pick_to_place')} puts the object into the table")
    if heights.get("carry") is None or heights["carry"] < CARRY_MIN_Z:
        out.append(f"carry: lowest site z {heights.get('carry')} below {CARRY_MIN_Z}")
    return out
