"""Launch voice command input and hybrid command interpreter nodes.

이 launch는 STT와 자연어 명령 해석 노드만 실행한다.
터미널 UI는 로그와 입력이 섞이지 않도록 별도 터미널에서
`ros2 run macgyvbot voice_command_ui_node`로 실행한다.
기존 pick-and-place 실행은 별도 터미널에서 `macgyvbot.launch.py`를
실행한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_stt = LaunchConfiguration('use_stt')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_stt',
            default_value='true',
            description='마이크 STT 노드를 실행할지 여부',
        ),
        Node(
            package='macgyvbot',
            executable='stt_node',
            name='stt_node',
            output='screen',
            condition=IfCondition(use_stt),
        ),
        Node(
            package='macgyvbot',
            executable='llm_command_node',
            name='llm_command_node',
            output='screen',
            parameters=[{
                'use_local_parser': True,
                'use_llm_fallback': True,
                'model': 'qwen2.5:0.5b',
                'min_confidence': 0.55,
            }],
        ),
    ])
