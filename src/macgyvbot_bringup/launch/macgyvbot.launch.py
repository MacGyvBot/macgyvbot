from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from macgyvbot_bringup.joint_velocity_config import (
    apply_joint_velocity_limits_to_moveit_config,
)
from macgyvbot_config.command import (
    DEFAULT_COMMAND_MIN_CONFIDENCE,
    DEFAULT_LLM_MODEL,
    DEFAULT_LLM_TIMEOUT_SEC,
    DEFAULT_PARSER_MODE,
    DEFAULT_STT_AMBIENT_DURATION_SEC,
    DEFAULT_STT_NON_SPEAKING_DURATION_SEC,
    DEFAULT_STT_PAUSE_THRESHOLD,
    DEFAULT_STT_PHRASE_THRESHOLD,
    DEFAULT_STT_PHRASE_TIME_LIMIT_SEC,
    DEFAULT_TTS_EDGE_RATE,
    DEFAULT_TTS_ENGINE,
    DEFAULT_TTS_PITCH,
    DEFAULT_TTS_TIMEOUT_SEC,
    DEFAULT_TTS_VOICE,
)
from macgyvbot_config.models import (
    HAND_GRASP_MODEL_NAME,
    HAND_GRASP_SAM_CHECKPOINT_NAME,
    YOLO_CONFIDENCE_THRESHOLD,
    YOLO_MODEL_NAME,
)
from macgyvbot_config.robot import BASE_FRAME
from macgyvbot_config.hand_grasp import (
    HAND_GRASP_ALLOW_BBOX_LOCK,
    HAND_GRASP_LAUNCH_DEPTH_DIFF_THRESHOLD_MM,
    HAND_GRASP_LAUNCH_DEPTH_MIN_CONTACT_LANDMARKS,
    HAND_GRASP_MAX_HANDS,
    HAND_GRASP_SAM_RESEED_FROM_YOLO,
    HAND_GRASP_SAM_TRACK_INTERVAL,
    HAND_GRASP_SAM_TRACK_MARGIN,
    HAND_GRASP_SAM_TRACK_MAX_AREA_RATIO,
    HAND_GRASP_SAM_TRACK_MAX_CENTER_SHIFT_PX,
    HAND_GRASP_SAM_TRACK_MIN_AREA_RATIO,
    HAND_GRASP_YOLO_IMAGE_SIZE,
)
from macgyvbot_config.topics import (
    CAMERA_COLOR_TOPIC,
    CAMERA_DEPTH_TOPIC,
    CAMERA_INFO_TOPIC,
    FORCE_TORQUE_TOPIC,
    HAND_GRASP_IMAGE_TOPIC,
    HAND_GRASP_MASK_LOCK_TOPIC,
    HAND_GRASP_TOPIC,
    ROBOT_STATUS_TOPIC,
)
from macgyvbot_config.vlm import (
    DEFAULT_GRASP_POINT_MODE,
    GRASP_POINT_API_MODEL,
    SAM_YAW_SERVICE_NAME,
    SAM_YAW_SERVICE_RESPONSE_TIMEOUT_SEC,
    SAM_YAW_SERVICE_WAIT_TIMEOUT_SEC,
    SAM_BACKEND_DEFAULT,
    SAM_DEPTH_EXPAND_ITERATIONS,
    SAM_DEPTH_MIN_VALID_RATIO,
    SAM_DEPTH_TOLERANCE_MM,
    SAM_DEVICE_DEFAULT,
    SAM_MODEL_TYPE_DEFAULT,
    VLM_GRASP_SERVICE_NAME,
    VLM_SERVICE_RESPONSE_TIMEOUT_SEC,
    VLM_SERVICE_WAIT_TIMEOUT_SEC,
)
from moveit_configs_utils import MoveItConfigsBuilder

SCREEN_OUTPUT_FORMAT = "{line}"


COMBINED_ROBOT_URDF = "m0609_onrobot_rg2_combined.urdf"


def load_combined_robot_description():
    resources_share = Path(get_package_share_directory("macgyvbot_resources"))
    urdf_path = resources_share / "urdf" / COMBINED_ROBOT_URDF
    return urdf_path.read_text(encoding="utf-8")


def generate_launch_description():
    bringup_share = FindPackageShare("macgyvbot_bringup")
    resources_share = FindPackageShare("macgyvbot_resources")
    default_yolo_model = PathJoinSubstitution(
        [resources_share, "weights", YOLO_MODEL_NAME]
    )
    default_grasp_model = PathJoinSubstitution(
        [resources_share, "weights", HAND_GRASP_MODEL_NAME]
    )
    default_sam_checkpoint = PathJoinSubstitution(
        [resources_share, "weights", HAND_GRASP_SAM_CHECKPOINT_NAME]
    )

    use_voice_command = LaunchConfiguration("use_voice_command")
    use_stt = LaunchConfiguration("use_stt")
    stt_pause_threshold = LaunchConfiguration("stt_pause_threshold")
    stt_phrase_threshold = LaunchConfiguration("stt_phrase_threshold")
    stt_non_speaking_duration = LaunchConfiguration("stt_non_speaking_duration")
    stt_phrase_time_limit = LaunchConfiguration("stt_phrase_time_limit")
    stt_ambient_duration = LaunchConfiguration("stt_ambient_duration")
    use_tts = LaunchConfiguration("use_tts")
    tts_engine = LaunchConfiguration("tts_engine")
    tts_voice = LaunchConfiguration("tts_voice")
    tts_edge_rate = LaunchConfiguration("tts_edge_rate")
    tts_pitch = LaunchConfiguration("tts_pitch")
    tts_timeout_sec = LaunchConfiguration("tts_timeout_sec")
    llm_model = LaunchConfiguration("llm_model")
    llm_timeout_sec = LaunchConfiguration("llm_timeout_sec")
    parser_mode = LaunchConfiguration("parser_mode")
    detector_image_topic = LaunchConfiguration("detector_image_topic")
    display_debug_windows = LaunchConfiguration("display_debug_windows")
    enable_drawer_collision_scene = LaunchConfiguration(
        "enable_drawer_collision_scene"
    )
    enable_gripper_self_collision_acm = LaunchConfiguration(
        "enable_gripper_self_collision_acm"
    )
    sam_enabled = LaunchConfiguration("sam_enabled")
    sam_checkpoint = LaunchConfiguration("sam_checkpoint")
    grasp_point_api_model = LaunchConfiguration("grasp_point_api_model")
    grasp_point_api_env_file = LaunchConfiguration("grasp_point_api_env_file")
    grasp_point_api_base_url = LaunchConfiguration("grasp_point_api_base_url")
    grasp_point_api_timeout_sec = LaunchConfiguration(
        "grasp_point_api_timeout_sec"
    )
    yolo_conf = LaunchConfiguration("yolo_conf")
    vlm_service_name = LaunchConfiguration("vlm_service_name")
    vlm_service_wait_timeout_sec = LaunchConfiguration(
        "vlm_service_wait_timeout_sec"
    )
    vlm_service_response_timeout_sec = LaunchConfiguration(
        "vlm_service_response_timeout_sec"
    )
    sam_yaw_service_name = LaunchConfiguration("sam_yaw_service_name")
    sam_yaw_service_wait_timeout_sec = LaunchConfiguration(
        "sam_yaw_service_wait_timeout_sec"
    )
    sam_yaw_service_response_timeout_sec = LaunchConfiguration(
        "sam_yaw_service_response_timeout_sec"
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
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["robot_description"] = load_combined_robot_description()
    moveit_config_dict = apply_joint_velocity_limits_to_moveit_config(
        moveit_config_dict
    )

    moveit_py_params = PathJoinSubstitution(
        [bringup_share, "config", "moveit_py.yaml"]
    )

    command_input_node = Node(
        package="macgyvbot_command",
        executable="command_input_node",
        name="command_input_node",
        output="screen",
        output_format=SCREEN_OUTPUT_FORMAT,
        parameters=[
            {
                "enable_microphone": use_stt,
                "pause_threshold": stt_pause_threshold,
                "phrase_threshold": stt_phrase_threshold,
                "non_speaking_duration": stt_non_speaking_duration,
                "phrase_time_limit": stt_phrase_time_limit,
                "ambient_duration": stt_ambient_duration,
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
                "min_confidence": DEFAULT_COMMAND_MIN_CONFIDENCE,
            },
        ],
        condition=IfCondition(use_voice_command),
    )

    operator_ui_node = Node(
        package="macgyvbot_ui",
        executable="operator_ui_node",
        name="operator_ui_node",
        output="screen",
        output_format=SCREEN_OUTPUT_FORMAT,
        parameters=[
            {
                "camera_status_topic": CAMERA_COLOR_TOPIC,
                "detector_image_topic": detector_image_topic,
                "robot_status_topic": ROBOT_STATUS_TOPIC,
            },
        ],
        condition=IfCondition(use_voice_command),
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
                "yolo_conf",
                default_value=str(YOLO_CONFIDENCE_THRESHOLD),
                description="YOLO confidence threshold for runtime detectors.",
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
                "stt_pause_threshold",
                default_value=str(DEFAULT_STT_PAUSE_THRESHOLD),
                description="Seconds of silence before STT finalizes a phrase.",
            ),
            DeclareLaunchArgument(
                "stt_phrase_threshold",
                default_value=str(DEFAULT_STT_PHRASE_THRESHOLD),
                description="Minimum speaking duration accepted as a phrase.",
            ),
            DeclareLaunchArgument(
                "stt_non_speaking_duration",
                default_value=str(DEFAULT_STT_NON_SPEAKING_DURATION_SEC),
                description="Silence retained around each STT phrase.",
            ),
            DeclareLaunchArgument(
                "stt_phrase_time_limit",
                default_value=str(DEFAULT_STT_PHRASE_TIME_LIMIT_SEC),
                description="Maximum seconds captured for one STT phrase.",
            ),
            DeclareLaunchArgument(
                "stt_ambient_duration",
                default_value=str(DEFAULT_STT_AMBIENT_DURATION_SEC),
                description="Seconds used for microphone ambient-noise calibration.",
            ),
            DeclareLaunchArgument(
                "use_tts",
                default_value="true",
                description="Enable TTS feedback.",
            ),
            DeclareLaunchArgument("tts_engine", default_value=DEFAULT_TTS_ENGINE),
            DeclareLaunchArgument("tts_voice", default_value=DEFAULT_TTS_VOICE),
            DeclareLaunchArgument("tts_edge_rate", default_value=DEFAULT_TTS_EDGE_RATE),
            DeclareLaunchArgument("tts_pitch", default_value=DEFAULT_TTS_PITCH),
            DeclareLaunchArgument(
                "tts_timeout_sec",
                default_value=str(DEFAULT_TTS_TIMEOUT_SEC),
            ),
            DeclareLaunchArgument("llm_model", default_value=DEFAULT_LLM_MODEL),
            DeclareLaunchArgument(
                "llm_timeout_sec",
                default_value=str(DEFAULT_LLM_TIMEOUT_SEC),
            ),
            DeclareLaunchArgument("parser_mode", default_value=DEFAULT_PARSER_MODE),
            DeclareLaunchArgument(
                "detector_image_topic",
                default_value=HAND_GRASP_IMAGE_TOPIC,
                description="Annotated detector image topic shown in the GUI.",
            ),
            DeclareLaunchArgument(
                "display_debug_windows",
                default_value="false",
                description=(
                    "Show legacy OpenCV debug windows from macgyvbot_main_node."
                ),
            ),
            DeclareLaunchArgument(
                "enable_drawer_collision_scene",
                default_value="true",
                description="Register static drawer collision boxes in MoveIt.",
            ),
            DeclareLaunchArgument(
                "enable_gripper_self_collision_acm",
                default_value="true",
                description=(
                    "Allow only RG2 internal self-collision pairs in MoveIt's "
                    "planning scene ACM."
                ),
            ),
            DeclareLaunchArgument(
                "grasp_point_mode",
                default_value=DEFAULT_GRASP_POINT_MODE,
                description=(
                    "Grasp point selection mode: center, yolo, vlm, "
                    "vlm_only_smol, vlm_only_qwen3b, vlm_only_qwen7b, or api"
                ),
            ),
            DeclareLaunchArgument(
                "grasp_point_api_model",
                default_value=GRASP_POINT_API_MODEL,
            ),
            DeclareLaunchArgument("grasp_point_api_env_file", default_value=""),
            DeclareLaunchArgument("grasp_point_api_base_url", default_value=""),
            DeclareLaunchArgument(
                "grasp_point_api_timeout_sec",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "vlm_service_name",
                default_value=VLM_GRASP_SERVICE_NAME,
            ),
            DeclareLaunchArgument(
                "vlm_service_wait_timeout_sec",
                default_value=str(VLM_SERVICE_WAIT_TIMEOUT_SEC),
            ),
            DeclareLaunchArgument(
                "vlm_service_response_timeout_sec",
                default_value=str(VLM_SERVICE_RESPONSE_TIMEOUT_SEC),
            ),
            DeclareLaunchArgument(
                "sam_yaw_service_name",
                default_value=SAM_YAW_SERVICE_NAME,
            ),
            DeclareLaunchArgument(
                "sam_yaw_service_wait_timeout_sec",
                default_value=str(SAM_YAW_SERVICE_WAIT_TIMEOUT_SEC),
            ),
            DeclareLaunchArgument(
                "sam_yaw_service_response_timeout_sec",
                default_value=str(SAM_YAW_SERVICE_RESPONSE_TIMEOUT_SEC),
            ),
            DeclareLaunchArgument(
                "force_torque_topic",
                default_value=FORCE_TORQUE_TOPIC,
            ),
            DeclareLaunchArgument(
                "grasp_model",
                default_value=default_grasp_model,
            ),
            DeclareLaunchArgument(
                "sam_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "sam_checkpoint",
                default_value=default_sam_checkpoint,
            ),
            SetEnvironmentVariable(
                "RCUTILS_CONSOLE_OUTPUT_FORMAT",
                "{message}",
            ),
            SetEnvironmentVariable(
                "PYTHONWARNINGS",
                "ignore",
            ),
            SetEnvironmentVariable(
                "TF_CPP_MIN_LOG_LEVEL",
                "3",
            ),
            SetEnvironmentVariable(
                "GLOG_minloglevel",
                "2",
            ),
            SetEnvironmentVariable(
                "ABSL_MIN_LOG_LEVEL",
                "2",
            ),
            Node(
                package="macgyvbot_task",
                executable="macgyvbot",
                output="screen",
                output_format=SCREEN_OUTPUT_FORMAT,
            ),
            Node(
                package="macgyvbot_task",
                executable="task_coordinator_node",
                output="screen",
                output_format=SCREEN_OUTPUT_FORMAT,
                arguments=[
                    "--ros-args",
                    "--log-level",
                    "moveit.py.cpp_initializer:=warn",
                    "--log-level",
                    "moveit_rdf_loader.rdf_loader:=warn",
                    "--log-level",
                    "moveit_robot_model.robot_model:=warn",
                    "--log-level",
                    "moveit.ros_planning_interface.moveit_cpp:=warn",
                    "--log-level",
                    "moveit_ros.current_state_monitor:=warn",
                    "--log-level",
                    "moveit.ros.occupancy_map_monitor.middleware_handle:=error",
                    "--log-level",
                    "moveit.ros_planning.planning_pipeline:=warn",
                    "--log-level",
                    "moveit_ros.add_time_optimal_parameterization:=warn",
                    "--log-level",
                    "moveit_ros.fix_workspace_bounds:=warn",
                    "--log-level",
                    "moveit_ros.fix_start_state_bounds:=warn",
                    "--log-level",
                    "moveit_ros.fix_start_state_collision:=warn",
                    "--log-level",
                    "moveit_ros.planning_scene_monitor.planning_scene_monitor:=error",
                    "--log-level",
                    "moveit.pilz_industrial_motion_planner.trajectory_generator_ptp:=warn",
                    "--log-level",
                    "moveit.pilz_industrial_motion_planner.trajectory_generator:=warn",
                ],
                parameters=[
                    moveit_config_dict,
                    moveit_py_params,
                    {
                        "yolo_model": LaunchConfiguration("yolo_model"),
                        "yolo_conf": yolo_conf,
                        "grasp_point_mode": LaunchConfiguration(
                            "grasp_point_mode"
                        ),
                        "grasp_point_api_model": grasp_point_api_model,
                        "grasp_point_api_env_file": grasp_point_api_env_file,
                        "grasp_point_api_base_url": grasp_point_api_base_url,
                        "grasp_point_api_timeout_sec": grasp_point_api_timeout_sec,
                        "vlm_service_name": vlm_service_name,
                        "vlm_service_wait_timeout_sec": vlm_service_wait_timeout_sec,
                        "vlm_service_response_timeout_sec": (
                            vlm_service_response_timeout_sec
                        ),
                        "sam_yaw_service_name": sam_yaw_service_name,
                        "sam_yaw_service_wait_timeout_sec": (
                            sam_yaw_service_wait_timeout_sec
                        ),
                        "sam_yaw_service_response_timeout_sec": (
                            sam_yaw_service_response_timeout_sec
                        ),
                        "force_torque_topic": LaunchConfiguration(
                            "force_torque_topic"
                        ),
                        "display_debug_windows": display_debug_windows,
                        "enable_drawer_collision_scene": (
                            enable_drawer_collision_scene
                        ),
                        "enable_gripper_self_collision_acm": (
                            enable_gripper_self_collision_acm
                        ),
                        "sam_enabled": sam_enabled,
                        "sam_checkpoint": sam_checkpoint,
                        "sam_backend": SAM_BACKEND_DEFAULT,
                        "sam_model_type": SAM_MODEL_TYPE_DEFAULT,
                        "sam_device": SAM_DEVICE_DEFAULT,
                    },
                ],
            ),
            Node(
                package="macgyvbot_perception",
                executable="vlm_grasp_service_node",
                name="vlm_grasp_service_node",
                output="screen",
                output_format=SCREEN_OUTPUT_FORMAT,
                parameters=[
                    {
                        "vlm_service_name": vlm_service_name,
                        "grasp_point_mode": LaunchConfiguration("grasp_point_mode"),
                        "sam_enabled": sam_enabled,
                        "sam_checkpoint": sam_checkpoint,
                        "sam_backend": SAM_BACKEND_DEFAULT,
                        "sam_model_type": SAM_MODEL_TYPE_DEFAULT,
                        "sam_device": SAM_DEVICE_DEFAULT,
                    },
                ],
            ),
            Node(
                package="macgyvbot_perception",
                executable="sam_yaw_service_node",
                name="sam_yaw_service_node",
                output="screen",
                output_format=SCREEN_OUTPUT_FORMAT,
                parameters=[
                    {
                        "sam_yaw_service_name": sam_yaw_service_name,
                        "sam_enabled": sam_enabled,
                        "sam_checkpoint": sam_checkpoint,
                        "sam_backend": SAM_BACKEND_DEFAULT,
                        "sam_model_type": SAM_MODEL_TYPE_DEFAULT,
                        "sam_device": SAM_DEVICE_DEFAULT,
                        "sam_depth_tolerance_mm": SAM_DEPTH_TOLERANCE_MM,
                        "sam_depth_min_valid_ratio": SAM_DEPTH_MIN_VALID_RATIO,
                        "sam_depth_expand_iterations": SAM_DEPTH_EXPAND_ITERATIONS,
                    },
                ],
            ),
            Node(
                package="macgyvbot_perception",
                executable="hand_grasp_detection",
                name="hand_grasp_detection_node",
                output="screen",
                output_format=SCREEN_OUTPUT_FORMAT,
                parameters=[
                    {
                        "color_topic": CAMERA_COLOR_TOPIC,
                        "camera_info_topic": CAMERA_INFO_TOPIC,
                        "depth_topic": CAMERA_DEPTH_TOPIC,
                        "result_topic": HAND_GRASP_TOPIC,
                        "annotated_topic": HAND_GRASP_IMAGE_TOPIC,
                        "mask_lock_topic": HAND_GRASP_MASK_LOCK_TOPIC,
                        "use_depth": True,
                        "publish_base_position": False,
                        "position_frame_id": BASE_FRAME,
                        "publish_annotated": True,
                        "display": False,
                        "show_return_close_roi": False,
                        "yolo_model": LaunchConfiguration("yolo_model"),
                        "tool_classes": "hammer,pliers,screwdriver,tape_measure,wrench",
                        "yolo_conf": yolo_conf,
                        "yolo_imgsz": HAND_GRASP_YOLO_IMAGE_SIZE,
                        "max_hands": HAND_GRASP_MAX_HANDS,
                        "depth_diff_threshold_mm": (
                            HAND_GRASP_LAUNCH_DEPTH_DIFF_THRESHOLD_MM
                        ),
                        "depth_min_contact_landmarks": (
                            HAND_GRASP_LAUNCH_DEPTH_MIN_CONTACT_LANDMARKS
                        ),
                        "robot_status_topic": ROBOT_STATUS_TOPIC,
                        "grasp_model": LaunchConfiguration("grasp_model"),
                        "sam_enabled": sam_enabled,
                        "sam_checkpoint": sam_checkpoint,
                        "sam_backend": SAM_BACKEND_DEFAULT,
                        "sam_model_type": SAM_MODEL_TYPE_DEFAULT,
                        "sam_device": SAM_DEVICE_DEFAULT,
                        "sam_track_interval": HAND_GRASP_SAM_TRACK_INTERVAL,
                        "sam_track_margin": HAND_GRASP_SAM_TRACK_MARGIN,
                        "sam_reseed_from_yolo": HAND_GRASP_SAM_RESEED_FROM_YOLO,
                        "sam_track_max_center_shift_px": (
                            HAND_GRASP_SAM_TRACK_MAX_CENTER_SHIFT_PX
                        ),
                        "sam_track_min_area_ratio": (
                            HAND_GRASP_SAM_TRACK_MIN_AREA_RATIO
                        ),
                        "sam_track_max_area_ratio": (
                            HAND_GRASP_SAM_TRACK_MAX_AREA_RATIO
                        ),
                        "allow_bbox_lock": HAND_GRASP_ALLOW_BBOX_LOCK,
                        "require_ml_grasp": True,
                        "require_locked_tool": True,
                        "require_depth_grasp": True,
                    },
                ],
            ),
            command_input_node,
            operator_ui_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=operator_ui_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="operator_ui_node exited",
                            )
                        )
                    ],
                )
            ),
        ]
    )
