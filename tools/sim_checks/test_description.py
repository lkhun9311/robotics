# SPDX-License-Identifier: Apache-2.0
"""Static checks of the installed ur5e_description: the expanded URDF against the Menagerie MJCF.

MuJoCo simulates the MJCF while robot_state_publisher publishes the URDF, so the two must
describe the same robot. Nothing here starts a simulator; it expands xacro and parses XML.
"""
import math
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import pytest

from expected import HOME, JOINTS, RANGE

PLUGIN = "mujoco_ros2_control/MujocoSystemInterface"
# Hand-written from ur5e.xml: joints without an explicit axis take class "ur5e" axis "0 1 0".
AXIS = {
    "shoulder_pan_joint": (0.0, 0.0, 1.0),
    "shoulder_lift_joint": (0.0, 1.0, 0.0),
    "elbow_joint": (0.0, 1.0, 0.0),
    "wrist_1_joint": (0.0, 1.0, 0.0),
    "wrist_2_joint": (0.0, 0.0, 1.0),
    "wrist_3_joint": (0.0, 1.0, 0.0),
}
CTRLRANGE = {
    "shoulder_pan": ("shoulder_pan_joint", (-6.2831, 6.2831)),
    "shoulder_lift": ("shoulder_lift_joint", (-6.2831, 6.2831)),
    "elbow": ("elbow_joint", (-3.1415, 3.1415)),
    "wrist_1": ("wrist_1_joint", (-6.2831, 6.2831)),
    "wrist_2": ("wrist_2_joint", (-6.2831, 6.2831)),
    "wrist_3": ("wrist_3_joint", (-6.2831, 6.2831)),
}


def _floats(text):
    return tuple(float(v) for v in text.split())


@pytest.fixture(scope="module")
def share():
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError as e:
        pytest.fail(f"ament_index_python is not importable (ROS environment not sourced?): {e}")
    try:
        return pathlib.Path(get_package_share_directory("ur5e_description"))
    except PackageNotFoundError as e:
        reason = e.args[0] if e.args else repr(e)
    # Failing outside the except block keeps the ament_index traceback out of the report.
    pytest.fail(reason)


@pytest.fixture(scope="module")
def urdf(share):
    path = share / "urdf" / "ur5e.urdf.xacro"
    try:
        p = subprocess.run(["xacro", str(path)], capture_output=True, text=True)
    except OSError as e:
        pytest.fail(f"cannot run xacro: {e}")
    if p.returncode != 0:
        pytest.fail(f"xacro {path} exited {p.returncode}: {p.stderr}")
    return ET.fromstring(p.stdout)


# --- MJCF resolver: attributes come from the element, else its class chain innermost-first. ---


def _class_chains(root):
    """Maps each <default> class name to its chain of <default> elements, innermost first."""
    chains = {}

    def walk(node, chain):
        for d in node.findall("default"):
            inner = [d] + chain
            chains[d.get("class", "main")] = inner
            walk(d, inner)

    walk(root, [])
    return chains


def _resolve_mjcf(path):
    root = ET.parse(path).getroot()
    chains = _class_chains(root)

    def lookup(elem, attr, cls, fallback=None):
        """`attr` of `elem`, else of its class chain; `fallback` when given, else an error that
        names the element and the classes searched."""
        if elem.get(attr) is not None:
            return elem.get(attr)
        for d in chains[cls]:
            child = d.find(elem.tag)
            if child is not None and child.get(attr) is not None:
                return child.get(attr)
        if fallback is None:
            searched = [d.get("class", "main") for d in chains[cls]]
            raise ValueError(f"{path}: {elem.tag} '{elem.get('name')}' has no '{attr}', on the element "
                             f"or in class chain {searched}")
        return fallback

    bodies, joints = {}, {}

    def walk(body, parent, active):
        cls = body.get("childclass", active)
        bodies[body.get("name")] = {
            "parent": parent,
            "pos": _floats(body.get("pos", "0 0 0")),
            "quat": _floats(body.get("quat", "1 0 0 0")),
        }
        for j in body.findall("joint"):
            jcls = j.get("class", cls)
            joints[j.get("name")] = {
                "body": body.get("name"),
                "axis": _floats(lookup(j, "axis", jcls, "0 0 1")),
                "range": _floats(lookup(j, "range", jcls)),
            }
        for child in body.findall("body"):
            walk(child, body.get("name"), cls)

    for top in root.find("worldbody").findall("body"):
        walk(top, None, "main")
    actuators = {
        a.get("name"): {
            "joint": a.get("joint"),
            "ctrlrange": _floats(lookup(a, "ctrlrange", a.get("class", "main"))),
            "forcerange": _floats(lookup(a, "forcerange", a.get("class", "main"))),
        }
        for a in root.find("actuator")
    }
    keys = {k.get("name"): _floats(k.get("qpos")) for k in root.find("keyframe").findall("key")}
    return {"bodies": bodies, "joints": joints, "actuators": actuators, "keys": keys}


@pytest.fixture(scope="module")
def mjcf(share):
    path = share / "mjcf" / "ur5e.xml"
    if not path.is_file():
        pytest.fail(f"installed MJCF missing: {path}")
    return _resolve_mjcf(path)


# --- Rotation helpers ---


def _rpy_matrix(r, p, y):
    """URDF fixed-axis roll-pitch-yaw: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _quat_matrix(q):
    """MuJoCo quaternion (w x y z), normalised first as MuJoCo does."""
    n = math.sqrt(sum(v * v for v in q))
    w, x, y, z = (v / n for v in q)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def _close(a, b, tol):
    return all(abs(x - y) <= tol for x, y in zip(a, b)) and len(a) == len(b)


def _urdf_joints(urdf):
    return {j.get("name"): j for j in urdf.findall("joint")}


# --- Tests ---


def test_urdf_joints_are_the_menagerie_joints_in_order(urdf):
    revolute = tuple(j.get("name") for j in urdf.findall("joint") if j.get("type") == "revolute")
    assert revolute == JOINTS


def test_ros2_control_tag_loads_the_mujoco_plugin(urdf):
    blocks = urdf.findall("ros2_control")
    assert len(blocks) == 1, f"expected one <ros2_control>, found {len(blocks)}"
    block = blocks[0]
    assert (block.get("name"), block.get("type")) == ("MujocoSystem", "system")
    hardware = block.find("hardware")
    assert hardware.findtext("plugin") == PLUGIN
    params = {p.get("name"): p.text for p in hardware.findall("param")}
    model = params.get("mujoco_model") or ""
    assert model.endswith("scene_baseline.xml"), model
    assert pathlib.Path(model).is_file(), f"mujoco_model does not exist: {model}"
    assert params.get("headless") == "true"
    assert params.get("initial_keyframe") == "home"
    assert params.get("auto_register_cameras") == "false"


def test_each_joint_exposes_position_command_and_position_velocity_state(urdf):
    joints = urdf.find("ros2_control").findall("joint")
    assert tuple(j.get("name") for j in joints) == JOINTS
    for j in joints:
        name = j.get("name")
        assert [c.get("name") for c in j.findall("command_interface")] == ["position"], name
        assert [s.get("name") for s in j.findall("state_interface")] == ["position", "velocity"], name
        # The MJCF keyframe owns the start state; an initial_value here would compete with it.
        assert not j.findall(".//param[@name='initial_value']"), name


def test_urdf_limits_equal_the_mjcf_joint_range(urdf, mjcf):
    joints = _urdf_joints(urdf)
    for name in JOINTS:
        limit = joints[name].find("limit")
        got = (float(limit.get("lower")), float(limit.get("upper")))
        assert _close(got, RANGE[name], 1e-9), f"{name}: URDF limit {got} != expected {RANGE[name]}"
        assert _close(got, mjcf["joints"][name]["range"], 1e-9), (
            f"{name}: URDF limit {got} != MJCF range {mjcf['joints'][name]['range']}"
        )


def test_actuator_ctrlrange_lies_inside_the_joint_range(mjcf):
    for act, spec in mjcf["actuators"].items():
        lo, hi = spec["ctrlrange"]
        jlo, jhi = mjcf["joints"][spec["joint"]]["range"]
        assert jlo <= lo <= hi <= jhi, f"{act}: ctrlrange {(lo, hi)} outside joint range {(jlo, jhi)}"


def test_resolved_mjcf_matches_the_hand_written_expectations(mjcf):
    assert tuple(mjcf["joints"]) == JOINTS
    for name in JOINTS:
        assert mjcf["joints"][name]["range"] == RANGE[name], name
        assert mjcf["joints"][name]["axis"] == AXIS[name], name
    assert {a: (s["joint"], s["ctrlrange"]) for a, s in mjcf["actuators"].items()} == CTRLRANGE
    assert mjcf["keys"]["home"] == HOME


def test_urdf_origins_and_axes_equal_the_mjcf_bodies(urdf, mjcf):
    bodies = mjcf["bodies"]
    links = {link.get("name") for link in urdf.findall("link")}
    assert links == set(bodies) | {"base_link"}, f"URDF links {sorted(links)} vs MJCF bodies {sorted(bodies)}"
    joints = _urdf_joints(urdf)
    assert set(joints) == set(JOINTS) | {"base_fixed"}
    mismatches = []
    for name, j in joints.items():
        parent, child = j.find("parent").get("link"), j.find("child").get("link")
        body = bodies[child]
        if (body["parent"] or "base_link") != parent:
            mismatches.append(f"{name}: parent {parent} != MJCF parent {body['parent']}")
        origin = j.find("origin")
        xyz = _floats(origin.get("xyz", "0 0 0"))
        rpy = _floats(origin.get("rpy", "0 0 0"))
        if not _close(xyz, body["pos"], 1e-9):
            mismatches.append(f"{name}: origin xyz {xyz} != body {child} pos {body['pos']}")
        got = sum(_rpy_matrix(*rpy), ())
        want = sum(_quat_matrix(body["quat"]), ())
        if not _close(got, want, 1e-6):
            mismatches.append(f"{name}: origin rpy {rpy} != body {child} quat {body['quat']}")
        if j.get("type") == "revolute":
            if mjcf["joints"][name]["body"] != child:
                mismatches.append(f"{name}: child {child} != MJCF body {mjcf['joints'][name]['body']}")
            axis = _floats(j.find("axis").get("xyz"))
            if not _close(axis, mjcf["joints"][name]["axis"], 1e-9):
                mismatches.append(f"{name}: axis {axis} != MJCF axis {mjcf['joints'][name]['axis']}")
    assert not mismatches, "\n".join(mismatches)


def test_urdf_effort_equals_the_mjcf_actuator_forcerange(urdf, mjcf):
    joints = _urdf_joints(urdf)
    assert len(mjcf["actuators"]) == 6, sorted(mjcf["actuators"])  # an empty loop would pass vacuously
    for act, spec in mjcf["actuators"].items():
        lo, hi = spec["forcerange"]
        assert lo == -hi, f"{act}: forcerange {(lo, hi)} is not symmetric; one URDF effort cannot express it"
        effort = float(joints[spec["joint"]].find("limit").get("effort"))
        assert effort == hi, f"{spec['joint']}: URDF effort {effort} != actuator {act} forcerange {hi}"


_MJCF_WITHOUT_RANGE = """<mujoco>
  <default><default class="arm"><joint axis="0 1 0"/><general ctrlrange="-1 1" forcerange="-2 2"/></default></default>
  <worldbody><body name="link" childclass="arm"><joint name="j1"/></body></worldbody>
  <actuator><general class="arm" name="a1" joint="j1"/></actuator>
  <keyframe><key name="home" qpos="0"/></keyframe>
</mujoco>
"""


def test_resolver_names_a_missing_range(tmp_path):
    path = tmp_path / "no_range.xml"
    path.write_text(_MJCF_WITHOUT_RANGE)
    with pytest.raises(ValueError, match=r"joint 'j1' has no 'range', on the element or in class chain \['arm'"):
        _resolve_mjcf(path)
