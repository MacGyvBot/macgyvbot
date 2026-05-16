from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bringup_share = FindPackageShare("macgyvbot_bringup")
    resources_share = FindPackageShare("macgyvbot_resources")
    default_yolo_model = PathJoinSubstitution(
        [resources_share, "weights", "yolov11_best.pt"]
    )
    default_grasp_model = PathJoinSubstitution(
        [resources_share, "weights", "hand_grasp_model.pkl"]
    )
    default_sam_checkpoint = PathJoinSubstitution(
        [resources_share, "weights", "mobile_sam.pt"]
    )

    use_voice_command = LaunchConfiguration("use_voice_command")
    use_stt = LaunchConfiguration("use_stt")
    use_tts = LaunchConfiguration("use_tts")
    tts_engine = LaunchConfiguration("tts_engine")
    tts_voice = LaunchConfiguration("tts_voice")
    tts_edge_rate = LaunchConfiguration("tts_edge_rate")
    tts_pitch = LaunchConfiguration("tts_pitch")
    tts_timeout_sec = LaunchConfiguration("tts_timeout_sec")
    llm_model = LaunchConfiguration("llm_model")
    llm_timeout_sec = LaunchConfiguration("llm_timeout_sec")
    parser_mode = LaunchConfiguration("parser_mode")
    sam_enabled = LaunchConfiguration("sam_enabled")
    sam_checkpoint = LaunchConfiguration("sam_checkpoint")

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

    moveit_py_params = PathJoinSubstitution(
        [bringup_share, "config", "moveit_py.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "yolo_model",
                default_value=default_yolo_model,
                description=(
                    "YOLO model path. Defaults to installed "
                    "macgyvbot_resources package share."
                ),
            ),
            DeclareLaunchArgument(
                "use_voice_command",
                default_value="true",
                description="Run the command input node.",
            ),
            DeclareLaunchArgument(
                "use_stt",
                default_value="true",
                description="Enable microphone STT.",
            ),
            DeclareLaunchArgument(
                "use_tts",
                default_value="true",
                description="Enable TTS feedback.",
            ),
            DeclareLaunchArgument("tts_engine", default_value="auto"),
            DeclareLaunchArgument("tts_voice", default_value="ko-KR-SunHiNeural"),
            DeclareLaunchArgument("tts_edge_rate", default_value="+25%"),
            DeclareLaunchArgument("tts_pitch", default_value="+35Hz"),
            DeclareLaunchArgument("tts_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("llm_model", default_value="gemma3:1b"),
            DeclareLaunchArgument("llm_timeout_sec", default_value="25.0"),
            DeclareLaunchArgument("parser_mode", default_value="llm_primary"),
            DeclareLaunchArgument(
                "grasp_point_mode",
                default_value="center",
                description="Grasp point selection mode: center or vlm",
            ),
            DeclareLaunchArgument(
                "force_torque_topic",
                default_value="/force_torque_sensor_broadcaster/wrench",
            ),
            DeclareLaunchArgument(
                "grasp_model",
                default_value=default_grasp_model,
            ),
            DeclareLaunchArgument(
                "sam_enabled",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "sam_checkpoint",
                default_value=default_sam_checkpoint,
            ),
            Node(
                package="macgyvbot_task",
                executable="macgyvbot",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "yolo_model": LaunchConfiguration("yolo_model"),
                        "grasp_point_mode": LaunchConfiguration(
                            "grasp_point_mode"
                        ),
                        "force_torque_topic": LaunchConfiguration(
                            "force_torque_topic"
                        ),
                    },
                ],
            ),
            Node(
                package="macgyvbot_perception",
                executable="hand_grasp_detection",
                name="hand_grasp_detection_node",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "color_topic": "/camera/camera/color/image_raw",
                        "camera_info_topic": "/camera/camera/color/camera_info",
                        "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                        "result_topic": "/human_grasped_tool",
                        "annotated_topic": "/hand_grasp_detection/annotated_image",
                        "mask_lock_topic": "/hand_grasp_detection/tool_mask_lock",
                        "use_depth": True,
                        "publish_base_position": False,
                        "position_frame_id": "base_link",
                        "publish_annotated": True,
                        "display": False,
                        "yolo_model": LaunchConfiguration("yolo_model"),
                        "tool_classes": "drill,hammer,pliers,screwdriver,tape_measure,wrench",
                        "yolo_conf": 0.20,
                        "yolo_imgsz": 640,
                        "max_hands": 2,
                        "depth_diff_threshold_mm": 35.0,
                        "depth_min_contact_landmarks": 4,
                        "robot_status_topic": "/robot_task_status",
                        "grasp_model": LaunchConfiguration("grasp_model"),
                        "sam_enabled": sam_enabled,
                        "sam_checkpoint": sam_checkpoint,
                        "sam_backend": "mobile_sam",
                        "sam_model_type": "vit_t",
                        "sam_device": "cuda",
                        "sam_track_interval": 10,
                        "sam_track_margin": 12,
                        "allow_bbox_lock": True,
                        "require_ml_grasp": True,
                        "require_locked_tool": True,
                        "require_depth_grasp": True,
                    },
                ],
            ),
            Node(
                package="macgyvbot_command",
                executable="command_input_node",
                name="command_input_node",
                output="screen",
                parameters=[
                    {
                        "use_gui": True,
                        "enable_microphone": use_stt,
                        "enable_tts": use_tts,
                        "tts_engine": tts_engine,
                        "tts_voice": tts_voice,
                        "tts_edge_rate": tts_edge_rate,
                        "tts_pitch": tts_pitch,
                        "tts_timeout_sec": tts_timeout_sec,
                        "model": llm_model,
                        "use_local_parser": True,
                        "use_llm_fallback": True,
                        "parser_mode": parser_mode,
                        "timeout_sec": llm_timeout_sec,
                        "min_confidence": 0.55,
                    },
                ],
                condition=IfCondition(use_voice_command),
            ),
        ]
    )
