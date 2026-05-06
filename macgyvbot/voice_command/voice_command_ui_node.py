"""Terminal chat UI for macgyvbot voice command pipeline.

이 노드는 명령 입력과 상태 표시를 담당한다.
- 키보드 입력을 `/stt_text`로 발행해서 STT 없이도 테스트할 수 있다.
- 실제 STT가 발행한 `/stt_text`도 구독해서 사용자가 어떻게 말했는지 보여준다.
- `llm_command_node`의 해석 결과와 피드백을 화면에 표시한다.
- 로봇 상태 topic이 연결되면 완료/실패 상태도 표시한다.
"""

import json
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceCommandUiNode(Node):
    def __init__(self):
        super().__init__('voice_command_ui_node')

        self.declare_parameter('stt_text_topic', '/stt_text')
        self.declare_parameter('tool_command_topic', '/tool_command')
        self.declare_parameter('target_label_topic', '/target_label')
        self.declare_parameter('command_feedback_topic', '/command_feedback')
        self.declare_parameter('robot_status_topic', '/robot_task_status')

        stt_text_topic = self.get_parameter('stt_text_topic').value
        tool_command_topic = self.get_parameter('tool_command_topic').value
        target_label_topic = self.get_parameter('target_label_topic').value
        command_feedback_topic = self.get_parameter('command_feedback_topic').value
        robot_status_topic = self.get_parameter('robot_status_topic').value

        self._text_pub = self.create_publisher(String, stt_text_topic, 10)
        self.create_subscription(String, stt_text_topic, self._stt_text_cb, 10)
        self.create_subscription(String, tool_command_topic, self._tool_command_cb, 10)
        self.create_subscription(String, target_label_topic, self._target_label_cb, 10)
        self.create_subscription(String, command_feedback_topic, self._feedback_cb, 10)
        self.create_subscription(String, robot_status_topic, self._robot_status_cb, 10)

        self._last_cli_text = ''
        self._last_target_label = ''
        self._last_command = None
        self._stop_event = threading.Event()
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

        self._print_banner()
        self.get_logger().info('MacGyvBot 명령 UI 준비 완료')
        self.get_logger().info(f'문장 입력/표시 topic: {stt_text_topic}')
        self.get_logger().info(
            f'해석/피드백 topic: {tool_command_topic}, {command_feedback_topic}'
        )
        self.get_logger().info(f'로봇 상태 topic: {robot_status_topic}')
        self.get_logger().info('종료하려면 Ctrl+C 또는 /quit 을 입력하세요.')
        self._print_prompt()

    def _input_loop(self):
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                text = input()
            except EOFError:
                self._stop_event.set()
                break
            except KeyboardInterrupt:
                self._stop_event.set()
                break

            text = text.strip()
            if not text:
                self._print_prompt()
                continue

            if text in ('/q', '/quit', 'quit', 'exit'):
                self.get_logger().info('CLI 명령 UI 종료 요청')
                self._stop_event.set()
                rclpy.shutdown()
                break

            msg = String()
            msg.data = text
            self._last_cli_text = text
            self._text_pub.publish(msg)
            self._print_line('사용자 입력', text)

    def _stt_text_cb(self, msg):
        text = msg.data.strip()
        if not text:
            return

        if text == self._last_cli_text:
            self._print_line('입력 문장', text)
            self._last_cli_text = ''
            return

        self._print_line('음성 인식', text)

    def _tool_command_cb(self, msg):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'/tool_command JSON 파싱 실패: {msg.data}')
            self._print_prompt()
            return

        tool_name = command.get('tool_name', 'unknown')
        action = command.get('action', 'unknown')
        method = command.get('match_method', 'unknown')
        confidence = command.get('confidence', 0.0)
        self._last_command = command

        try:
            confidence_text = f'{float(confidence):.2f}'
        except (TypeError, ValueError):
            confidence_text = str(confidence)

        self._print_line(
            '해석 결과',
            f'tool={tool_name}, action={action}, '
            f'method={method}, confidence={confidence_text}',
        )
        self._print_prompt()

    def _target_label_cb(self, msg):
        self._last_target_label = msg.data
        self._print_line('실행 요청', f'{msg.data} pick 명령을 로봇 파이프라인으로 보냈습니다.')

    def _feedback_cb(self, msg):
        try:
            feedback = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'/command_feedback JSON 파싱 실패: {msg.data}')
            self._print_prompt()
            return

        status = feedback.get('status', 'unknown')
        message = feedback.get('message', '')
        reason = feedback.get('reason', 'unknown')

        if status == 'accepted':
            self._print_line('판단', message or '올바른 명령으로 판단했습니다.')
            return

        if status == 'rejected':
            self._print_line(
                '재입력 요청',
                f'{message or "명령을 이해하지 못했습니다."} '
                f'(reason={reason})',
            )
            self._print_prompt()
            return

        self._print_line('피드백', f'{message} (status={status}, reason={reason})')
        self._print_prompt()

    def _robot_status_cb(self, msg):
        status_text = msg.data.strip()
        if not status_text:
            return

        try:
            status = json.loads(status_text)
        except json.JSONDecodeError:
            self._print_line('로봇 상태', status_text)
            self._print_prompt()
            return

        state = status.get('status', status.get('state', 'unknown'))
        tool_name = status.get('tool_name', self._last_target_label or 'unknown')
        message = status.get('message', '')

        if state in ('done', 'completed', 'success'):
            self._print_line('완료', message or f'{tool_name} 전달 동작이 완료되었습니다.')
        elif state in ('failed', 'error'):
            self._print_line('실패', message or f'{tool_name} 동작 중 문제가 발생했습니다.')
        else:
            self._print_line('로봇 상태', message or f'{tool_name}: {state}')
        self._print_prompt()

    def _print_banner(self):
        sys.stdout.write('\n')
        sys.stdout.write('==============================\n')
        sys.stdout.write(' MacGyvBot Voice Command UI\n')
        sys.stdout.write('==============================\n')
        sys.stdout.write('예: 드라이버 가져다줘 / 그 조이는 거 가져와 / 망치 줘\n')
        sys.stdout.write('지원 공구: screwdriver, pliers, hammer, tape_measure\n')
        sys.stdout.flush()

    def _print_line(self, title, message):
        sys.stdout.write(f'\n[{title}] {message}\n')
        sys.stdout.flush()

    def _print_prompt(self):
        if self._stop_event.is_set():
            return
        sys.stdout.write('\n[MacGyvBot] 명령 입력 > ')
        sys.stdout.flush()

    def destroy_node(self):
        self._stop_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandUiNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
