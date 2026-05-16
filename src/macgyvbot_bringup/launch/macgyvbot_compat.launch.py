"""Compatibility alias for the primary migrated MacGyvBot launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_PASSTHROUGH_ARGS = [
    ("use_voice_command", "true"),
    ("use_stt", "true"),
    ("use_tts", "true"),
    ("tts_engine", "auto"),
    ("tts_voice", "ko-KR-SunHiNeural"),
    ("tts_edge_rate", "+25%"),
    ("tts_pitch", "+35Hz"),
    ("tts_timeout_sec", "20.0"),
    ("llm_model", "gemma3:1b"),
    ("llm_timeout_sec", "25.0"),
    ("parser_mode", "llm_primary"),
    ("grasp_point_mode", "center"),
    ("force_torque_topic", "/force_torque_sensor_broadcaster/wrench"),
    ("sam_enabled", "false"),
]


def generate_launch_description():
    """Delegate compatibility users to the migrated bringup launch."""
    resources_share = FindPackageShare("macgyvbot_resources")
    model_args = [
        (
            "yolo_model",
            PathJoinSubstitution(
                [resources_share, "weights", "yolov11_best.pt"]
            ),
        ),
        (
            "grasp_model",
            PathJoinSubstitution(
                [resources_share, "weights", "hand_grasp_model.pkl"]
            ),
        ),
        (
            "sam_checkpoint",
            PathJoinSubstitution(
                [resources_share, "weights", "mobile_sam.pt"]
            ),
        ),
    ]
    passthrough_args = model_args + _PASSTHROUGH_ARGS
    primary_launch = PathJoinSubstitution(
        [
            FindPackageShare("macgyvbot_bringup"),
            "launch",
            "macgyvbot.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(name, default_value=default)
            for name, default in passthrough_args
        ]
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(primary_launch),
                launch_arguments={
                    name: LaunchConfiguration(name)
                    for name, _ in passthrough_args
                }.items(),
            )
        ]
    )
