"""Launch drawer motion test with required MoveIt robot descriptions."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument(
                "open_offset_xyz",
                default_value="[0.0, 0.0, 0.0]",
            ),
            DeclareLaunchArgument(
                "observe_offset_xyz",
                default_value="[0.0, 0.0, 0.12]",
            ),
            DeclareLaunchArgument(
                "close_offset_xyz",
                default_value="[0.0, 0.0, 0.0]",
            ),
            DeclareLaunchArgument(
                "use_reverse_open_for_close",
                default_value="true",
            ),
            DeclareLaunchArgument("gripper_ip", default_value="192.168.1.1"),
            DeclareLaunchArgument("gripper_port", default_value="502"),
            DeclareLaunchArgument("gripper_settle_sec", default_value="0.8"),
            Node(
                package="macgyvbot_manipulation",
                executable="drawer_motion_test",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "dry_run": LaunchConfiguration("dry_run"),
                        "open_offset_xyz": LaunchConfiguration(
                            "open_offset_xyz"
                        ),
                        "observe_offset_xyz": LaunchConfiguration(
                            "observe_offset_xyz"
                        ),
                        "close_offset_xyz": LaunchConfiguration(
                            "close_offset_xyz"
                        ),
                        "use_reverse_open_for_close": LaunchConfiguration(
                            "use_reverse_open_for_close"
                        ),
                        "gripper_ip": LaunchConfiguration("gripper_ip"),
                        "gripper_port": LaunchConfiguration("gripper_port"),
                        "gripper_settle_sec": LaunchConfiguration(
                            "gripper_settle_sec"
                        ),
                    },
                ],
            ),
        ]
    )
