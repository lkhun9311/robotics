# SPDX-License-Identifier: Apache-2.0
"""UR5e in MuJoCo under ros2_control with the stock JointTrajectoryController.

Mirrors mujoco_ros2_control_demos/launch/01_basic_robot.launch.py at tag 0.1.2. The plugin needs
mujoco_ros2_control's own ros2_control_node, not the one from controller_manager.

    ros2 launch robot_bringup ur5e_baseline.launch.py [headless:=true|false] [camera:=false|true]
"""
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue

CONTROLLERS = ("joint_state_broadcaster", "arm_controller")


def _nodes(context):
    config = pathlib.Path(get_package_share_directory("robot_bringup")) / "config"
    xacro_file = pathlib.Path(get_package_share_directory("ur5e_description")) / "urdf" / "ur5e.urdf.xacro"
    headless = LaunchConfiguration("headless").perform(context)
    camera = LaunchConfiguration("camera").perform(context)
    if headless not in ("true", "false") or camera not in ("true", "false"):
        raise ValueError(f"headless and camera take true or false, got headless={headless} camera={camera}")

    import xacro

    robot_description = xacro.process_file(str(xacro_file), mappings={"headless": headless}).toxml()
    controllers_yaml = str(config / "ur5e_controllers.yaml")
    params = [{"use_sim_time": True}, ParameterFile(controllers_yaml)]
    if camera == "true":
        params.append(ParameterFile(str(config / "mujoco_plugins.yaml")))

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {"robot_description": ParameterValue(robot_description, value_type=str), "use_sim_time": True}
            ],
        ),
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=params,
            on_exit=Shutdown(),
        ),
    ] + [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--param-file", controllers_yaml],
            output="both",
        )
        for name in CONTROLLERS
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true", description="Run MuJoCo without a viewer"),
            DeclareLaunchArgument(
                "camera", default_value="false", description="Load the camera plugin from config/mujoco_plugins.yaml"
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
