"""Launch voice command input and hybrid command interpreter nodes.

이 launch는 사용자 입력을 자연어 명령으로 해석하는 부분만 실행한다.
기본값은 마이크 STT 입력이고, `use_cli_ui:=true use_stt:=false`로 실행하면
터미널 챗봇 형태로 문장을 직접 입력할 수 있다.
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
    use_cli_ui = LaunchConfiguration('use_cli_ui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_stt',
            default_value='true',
            description='마이크 STT 노드를 실행할지 여부',
        ),
        DeclareLaunchArgument(
            'use_cli_ui',
            default_value='true',
            description='터미널 입력 UI 노드를 실행할지 여부',
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
            executable='voice_command_ui_node',
            name='voice_command_ui_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(use_cli_ui),
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
