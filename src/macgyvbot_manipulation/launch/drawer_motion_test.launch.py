"""Launch drawer motion test with required MoveIt robot descriptions."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bringup_share = FindPackageShare("macgyvbot_bringup")
    moveit_py_params = PathJoinSubstitution(
        [bringup_share, "config", "moveit_py.yaml"]
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description()
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            Node(
                package="macgyvbot_manipulation",
                executable="drawer_motion_test",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                ],
            ),
        ]
    )
