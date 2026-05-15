from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
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
    drawer_yolo_model = LaunchConfiguration("drawer_yolo_model")

    # Doosan M0609 MoveIt 기본 설정 (URDF, SRDF, kinematics, controllers 등)
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description()                # m0609.urdf.xacro
        .robot_description_semantic(file_path="config/dsr.srdf")  # dsr.srdf
        .robot_description_kinematics()     # kinematics.yaml
        .joint_limits()                     # joint_limits.yaml
        .trajectory_execution()             # moveit_controllers.yaml
        .planning_scene_monitor()           # sensors_3d.yaml 등과 연동
        .sensors_3d()
        .to_moveit_configs()
    )

    # 🔹 MoveItPy 전용 YAML 추가
    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("macgyvbot"), "config", "moveit_py.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "yolo_model",
                default_value="yolov11_best.pt",
            ),
            DeclareLaunchArgument(
                "drawer_yolo_model",
                default_value="yolov11n_drawer.pt",
                description="서랍 탐지용 YOLO 모델 경로 또는 weights/ 내 파일명",
            ),
            DeclareLaunchArgument(
                "use_voice_command",
                default_value="true",
                description="STT + 통합 명령 해석 노드를 실행할지 여부",
            ),
            DeclareLaunchArgument(
                "use_stt",
                default_value="true",
                description="마이크 STT 노드를 실행할지 여부",
            ),
            DeclareLaunchArgument(
                "use_tts",
                default_value="true",
                description="MacGyvBot GUI 응답과 주요 상태 메시지를 TTS로 출력할지 여부",
            ),
            DeclareLaunchArgument(
                "tts_engine",
                default_value="auto",
                description="TTS engine: auto, edge, espeak-ng",
            ),
            DeclareLaunchArgument(
                "tts_voice",
                default_value="ko-KR-SunHiNeural",
                description="TTS voice. edge 사용 시 예: ko-KR-SunHiNeural",
            ),
            DeclareLaunchArgument(
                "tts_edge_rate",
                default_value="+25%",
                description="edge-tts speech rate. 예: +10%, +0%, -10%",
            ),
            DeclareLaunchArgument(
                "tts_pitch",
                default_value="+35Hz",
                description="edge-tts pitch. 예: +8Hz, +0Hz, -5Hz",
            ),
            DeclareLaunchArgument(
                "tts_timeout_sec",
                default_value="20.0",
                description="TTS 생성/재생 명령 제한 시간(초)",
            ),
            DeclareLaunchArgument(
                "llm_model",
                default_value="gemma3:1b",
                description="Ollama command parser에 사용할 로컬 LLM 모델명",
            ),
            DeclareLaunchArgument(
                "llm_timeout_sec",
                default_value="25.0",
                description="Ollama command parser 응답 대기 시간(초)",
            ),
            DeclareLaunchArgument(
                "parser_mode",
                default_value="llm_primary",
                description="Command parser mode: hybrid or llm_primary",
            ),
            DeclareLaunchArgument(
                "grasp_point_mode",
                default_value="center",
                description="Grasp point selection mode: center or vlm",
            ),
            DeclareLaunchArgument(
                "force_torque_topic",
                default_value="/force_torque_sensor_broadcaster/wrench",
                description="반납 Home 하강 중 Z 반력 감지에 사용할 WrenchStamped topic",
            ),
            DeclareLaunchArgument(
                "grasp_model",
                default_value="weights/hand_grasp_model.pkl",
                description="사용자 hand grasp/open 분류 .pkl 모델 경로",
            ),
            DeclareLaunchArgument(
                "sam_enabled",
                default_value="false",
                description="robot grasp success 이후 SAM tool mask lock을 사용할지 여부",
            ),
            DeclareLaunchArgument(
                "sam_checkpoint",
                default_value="weights/mobile_sam.pt",
                description="MobileSAM 또는 SAM checkpoint 경로",
            ),
            Node(
                package="macgyvbot",
                executable="macgyvbot",
                output="screen",
                # MoveIt config + MoveItPy용 설정을 같이 넘김
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "yolo_model": LaunchConfiguration("yolo_model"),
                        "drawer_yolo_model": drawer_yolo_model,
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
                package="macgyvbot",
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
                    }
                ],
            ),
            Node(
                package="macgyvbot",
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
                    }
                ],
                condition=IfCondition(use_voice_command),
            ),
        ]
    )
