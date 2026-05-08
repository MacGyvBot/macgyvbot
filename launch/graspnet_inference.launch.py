from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_checkpoint_path = PathJoinSubstitution(
        [
            FindPackageShare("macgyvbot"),
            "models",
            "graspnet",
            "checkpoint-rs.tar",
        ]
    )
    default_baseline_path = PathJoinSubstitution(
        [
            FindPackageShare("macgyvbot"),
            "models",
            "graspnet",
            "graspnet-baseline",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "checkpoint_path",
                default_value=default_checkpoint_path,
            ),
            DeclareLaunchArgument(
                "graspnet_baseline_path",
                default_value=default_baseline_path,
            ),
            DeclareLaunchArgument("pose_topic", default_value="/graspnet/target_pose"),
            DeclareLaunchArgument(
                "camera_frame",
                default_value="camera_color_optical_frame",
            ),
            DeclareLaunchArgument("num_point", default_value="20000"),
            DeclareLaunchArgument("num_view", default_value="300"),
            DeclareLaunchArgument("collision_thresh", default_value="-1.0"),
            DeclareLaunchArgument("voxel_size", default_value="0.01"),
            DeclareLaunchArgument("device", default_value="cuda:0"),
            Node(
                package="macgyvbot",
                executable="graspnet_inference",
                name="graspnet_inference_node",
                output="screen",
                parameters=[
                    {
                        "checkpoint_path": LaunchConfiguration("checkpoint_path"),
                        "graspnet_baseline_path": LaunchConfiguration(
                            "graspnet_baseline_path"
                        ),
                        "pose_topic": LaunchConfiguration("pose_topic"),
                        "camera_frame": LaunchConfiguration("camera_frame"),
                        "num_point": LaunchConfiguration("num_point"),
                        "num_view": LaunchConfiguration("num_view"),
                        "collision_thresh": LaunchConfiguration("collision_thresh"),
                        "voxel_size": LaunchConfiguration("voxel_size"),
                        "device": LaunchConfiguration("device"),
                    }
                ],
            ),
        ]
    )
