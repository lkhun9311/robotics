# SPDX-License-Identifier: Apache-2.0
"""Forward kinematics by MuJoCo itself: the installed scene the plugin loads, joint positions set
by name, `mj_kinematics`, and the `attachment_site` pose MuJoCo computes. No kinematics of this
project's own is involved, so a site position here is what the simulator would place there.

The Python bindings are the PyPI wheel of the MuJoCo version the plugin links (3.12.0, see
docker/Dockerfile); `Kinematics` refuses any other version.
"""
import pathlib

REPO = pathlib.Path(__file__).resolve().parents[2]
SITE = "attachment_site"
MUJOCO_VERSION = "3.12.0"
# Where colcon installs the scene in this workspace, for a process that has not sourced the
# install's setup script (the generic pytest stage of run_checks.sh).
INSTALLED_SHARE = REPO / "install" / "release" / "ur5e_description" / "share" / "ur5e_description"


def scene_path():
    """The installed scene_baseline.xml, the file the plugin loads (robot_description's
    `mujoco_model`)."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = pathlib.Path(get_package_share_directory("ur5e_description"))
    except (ImportError, LookupError):
        share = INSTALLED_SHARE
    path = share / "mjcf" / "scene_baseline.xml"
    if not path.is_file():
        raise FileNotFoundError(f"installed scene missing: {path} (build the workspace first)")
    return path


class Kinematics:
    def __init__(self, path=None):
        import mujoco

        if mujoco.mj_versionString() != MUJOCO_VERSION:
            raise RuntimeError(f"MuJoCo bindings {mujoco.mj_versionString()}, the plugin links {MUJOCO_VERSION}")
        self._mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(path or scene_path()))
        self.data = mujoco.MjData(self.model)
        self._site = self.model.site(SITE).id

    def set(self, positions):
        """Writes joint -> position into qpos by joint name and runs MuJoCo's kinematics."""
        for joint, value in positions.items():
            self.data.qpos[self.model.joint(joint).qposadr[0]] = value
        self._mujoco.mj_kinematics(self.model, self.data)

    def site(self, positions):
        """(position xyz, tool axis xyz) of the attachment_site, world frame, for joint -> position.
        The tool axis is the site's z axis, which points out of the flange."""
        self.set(positions)
        rot = self.data.site_xmat[self._site].reshape(3, 3)
        return tuple(float(v) for v in self.data.site_xpos[self._site]), tuple(float(v) for v in rot[:, 2])
