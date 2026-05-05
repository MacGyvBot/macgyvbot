"""Launch STT and hybrid command interpreter nodes.

이 launch는 음성 입력을 자연어 명령으로 해석하는 부분만 실행한다.
기존 pick-and-place 실행은 별도 터미널에서 `hf_auto_pick_place.launch.py`를
실행한다.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='macgyvbot',
            executable='stt_node',
            name='stt_node',
            output='screen',
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
