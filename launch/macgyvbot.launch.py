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
    llm_model = LaunchConfiguration("llm_model")

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
                "llm_model",
                default_value="qwen2.5:0.5b",
                description="Ollama command parser에 사용할 로컬 LLM 모델명",
            ),
            DeclareLaunchArgument(
                "grasp_point_mode",
                default_value="center",
                description="Grasp point selection mode: center or vlm",
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
                        "grasp_point_mode": LaunchConfiguration(
                            "grasp_point_mode"
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
                    {
                        "color_topic": "/camera/camera/color/image_raw",
                        "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                        "result_topic": "/human_grasped_tool",
                        "annotated_topic": "/hand_grasp_detection/annotated_image",
                        "use_depth": True,
                        "publish_annotated": True,
                        "display": False,
                        "yolo_model": LaunchConfiguration("yolo_model"),
                        "tool_classes": "drill,hammer,pliers,screwdriver,wrench",
                        "yolo_conf": 0.20,
                        "yolo_imgsz": 640,
                        "max_hands": 2,
                        "depth_diff_threshold_mm": 35.0,
                        "depth_min_contact_landmarks": 4,
                    }
                ],
            ),
            Node(
                package="macgyvbot",
                executable="stt_node",
                name="stt_node",
                output="screen",
                parameters=[
                    {
                        "use_gui": True,
                        "enable_microphone": use_stt,
                        "model": llm_model,
                        "use_local_parser": True,
                        "use_llm_fallback": True,
                        "min_confidence": 0.55,
                    }
                ],
                condition=IfCondition(use_voice_command),
            ),
        ]
    )
