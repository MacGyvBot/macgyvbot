"""Unified command-input node for macgyvbot.

- 마이크 STT와 GUI 입력을 하나의 채팅 흐름으로 수집한다.
- 입력 텍스트를 local parser -> LLM fallback 순서로 해석한다.
- 결과를 `/tool_command`, `/command_feedback`로 발행한다.
"""

import json
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from macgyvbot.ui.voice_command_window import (
    QApplication,
    QTimer,
    VoiceCommandGuiWindow,
)
from macgyvbot.util.command_input.input_mapping.command_llm_parser import (
    CommandLlmParser,
)
from macgyvbot.util.command_input.stt.speech_to_text import SpeechToTextService


class CommandInputNode(Node):
    def __init__(self):
        super().__init__('command_input_node')

        self.declare_parameter('use_gui', True)
        self.declare_parameter('enable_microphone', False)
        self.declare_parameter('language', 'ko-KR')
        self.declare_parameter('device_index', -1)
        self.declare_parameter('energy_threshold', 300.0)
        self.declare_parameter('pause_threshold', 0.8)
        self.declare_parameter('phrase_time_limit', 5.0)
        self.declare_parameter('dynamic_energy', True)
        self.declare_parameter('ambient_duration', 1.0)
        self.declare_parameter('stt_text_topic', '/stt_text')
        self.declare_parameter('compat_topic', '/stt_result')
        self.declare_parameter('publish_compat_topic', True)

        self.declare_parameter('tool_command_topic', '/tool_command')
        self.declare_parameter('command_feedback_topic', '/command_feedback')
        self.declare_parameter('robot_status_topic', '/robot_task_status')
        self.declare_parameter('camera_status_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('connection_check_period_sec', 1.0)
        self.declare_parameter('camera_timeout_sec', 3.0)
        self.declare_parameter(
            'robot_node_names',
            'macgyvbot_main_node,macgyvbot',
        )
        self.declare_parameter(
            'ollama_url',
            'http://localhost:11434/api/generate',
        )
        self.declare_parameter('model', 'gemma3:1b')
        self.declare_parameter('timeout_sec', 25.0)
        self.declare_parameter('min_confidence', 0.55)
        self.declare_parameter('use_local_parser', True)
        self.declare_parameter('use_llm_fallback', True)

        self._use_gui = bool(self.get_parameter('use_gui').value)
        self._enable_microphone = bool(self.get_parameter('enable_microphone').value)

        if self._use_gui and (QApplication is None or VoiceCommandGuiWindow is None):
            raise RuntimeError(
                'use_gui=true 이지만 PyQt5가 설치되어 있지 않습니다. '
                'sudo apt install python3-pyqt5 또는 pip install PyQt5 후 다시 실행하세요.'
            )

        self._language = self.get_parameter('language').value
        self._device_index = int(self.get_parameter('device_index').value)
        self._phrase_time_limit = float(
            self.get_parameter('phrase_time_limit').value
        )
        self._publish_compat = bool(
            self.get_parameter('publish_compat_topic').value
        )

        stt_text_topic = self.get_parameter('stt_text_topic').value
        compat_topic = self.get_parameter('compat_topic').value
        tool_command_topic = self.get_parameter('tool_command_topic').value
        command_feedback_topic = self.get_parameter('command_feedback_topic').value
        robot_status_topic = self.get_parameter('robot_status_topic').value
        camera_status_topic = self.get_parameter('camera_status_topic').value

        self.window = None
        self._last_gui_text = ''
        self._last_target_label = ''
        self._last_camera_stamp_ns = None
        self._last_connection_text = ''
        self._robot_node_names = {
            name.strip()
            for name in str(self.get_parameter('robot_node_names').value).split(',')
            if name.strip()
        }
        self._camera_timeout_ns = int(
            float(self.get_parameter('camera_timeout_sec').value) * 1_000_000_000
        )

        self._stt_pub = self.create_publisher(String, stt_text_topic, 10)
        self._compat_pub = (
            self.create_publisher(String, compat_topic, 10)
            if self._publish_compat
            else None
        )
        self._tool_command_pub = self.create_publisher(String, tool_command_topic, 10)
        self._feedback_pub = self.create_publisher(String, command_feedback_topic, 10)

        self.create_subscription(String, stt_text_topic, self._text_cb, 10)
        self.create_subscription(String, tool_command_topic, self._tool_command_cb, 10)
        self.create_subscription(String, command_feedback_topic, self._feedback_cb, 10)
        self.create_subscription(String, robot_status_topic, self._robot_status_cb, 10)
        self.create_subscription(Image, camera_status_topic, self._camera_status_cb, 10)
        self.create_timer(
            float(self.get_parameter('connection_check_period_sec').value),
            self._update_connection_status,
        )

        self._parser = CommandLlmParser(
            ollama_url=self.get_parameter('ollama_url').value,
            model=self.get_parameter('model').value,
            timeout_sec=float(self.get_parameter('timeout_sec').value),
            min_confidence=float(self.get_parameter('min_confidence').value),
            use_local_parser=bool(self.get_parameter('use_local_parser').value),
            use_llm_fallback=bool(self.get_parameter('use_llm_fallback').value),
            logger=self._log_parser,
        )

        self._self_published = {}
        self._self_pub_lock = threading.Lock()

        self._stt_service = None
        if self._enable_microphone:
            self._stt_service = SpeechToTextService(
                language=self._language,
                device_index=self._device_index,
                energy_threshold=float(self.get_parameter('energy_threshold').value),
                pause_threshold=float(self.get_parameter('pause_threshold').value),
                dynamic_energy=bool(self.get_parameter('dynamic_energy').value),
                ambient_duration=float(self.get_parameter('ambient_duration').value),
                phrase_time_limit=self._phrase_time_limit,
                logger=self._log_stt,
            )
            self._stt_service.start(self._on_stt_text)

            self.get_logger().info(
                f'STT 준비 완료: language={self._language}, '
                f'device_index={self._device_index}, '
                f'phrase_time_limit={self._phrase_time_limit:.1f}s'
            )
        else:
            self.get_logger().info('마이크 STT 비활성화 모드로 실행합니다.')

        self.get_logger().info(f'입력 topic: {stt_text_topic}')
        self.get_logger().info(
            f'출력 topic: {tool_command_topic}, {command_feedback_topic}'
        )
        if self._publish_compat:
            self.get_logger().info(f'수업 예제 호환 topic: {compat_topic}')
        if self._use_gui:
            self.get_logger().info('GUI 모드 활성화: command_input_node가 UI를 직접 연결합니다.')

    def attach_window(self, window):
        self.window = window
        self._update_connection_status()

    def publish_user_text(self, text):
        text = (text or '').strip()
        if not text:
            return

        self._last_gui_text = text
        msg = String()
        msg.data = text
        self._stt_pub.publish(msg)

    def _log_stt(self, level, message):
        logger = self.get_logger()
        if level == 'warn':
            logger.warn(message)
            return
        if level == 'error':
            logger.error(message)
            return
        logger.info(message)

    def _log_parser(self, level, message):
        self._log_stt(level, message)

    def _mark_self_published(self, text):
        with self._self_pub_lock:
            self._self_published[text] = self._self_published.get(text, 0) + 1

    def _consume_if_self_published(self, text):
        with self._self_pub_lock:
            count = self._self_published.get(text, 0)
            if count <= 0:
                return False
            if count == 1:
                del self._self_published[text]
            else:
                self._self_published[text] = count - 1
            return True

    def _on_stt_text(self, text):
        text = (text or '').strip()
        if not text:
            return

        msg = String()
        msg.data = text

        self._mark_self_published(text)
        self._stt_pub.publish(msg)
        if self._compat_pub is not None:
            self._compat_pub.publish(msg)

        self.get_logger().info(f'STT 인식 결과: "{text}"')
        self._append_user(text, source='voice')
        self._set_status('음성 명령 수신')
        self._handle_text(text)

    def _text_cb(self, msg):
        text = (msg.data or '').strip()
        if not text:
            return

        if text == self._last_gui_text:
            self._last_gui_text = ''
            self._set_status('입력 수신')
            self._handle_text(text)
            return

        if self._consume_if_self_published(text):
            return

        self.get_logger().info(f'명령 해석 요청: "{text}"')
        self._append_user(text, source='voice')
        self._set_status('입력 수신')
        self._handle_text(text)

    def _handle_text(self, text):
        result = self._parser.interpret(text)

        command = result.get('command')
        if command is not None:
            self._publish_command(command)

        for payload in result.get('feedbacks', []):
            self._publish_feedback_payload(payload)

    def _publish_command(self, command):
        command_msg = String()
        command_msg.data = json.dumps(command, ensure_ascii=False)
        self._tool_command_pub.publish(command_msg)

    def _publish_feedback_payload(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._feedback_pub.publish(msg)

    def _tool_command_cb(self, msg):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'/tool_command JSON 파싱 실패: {msg.data}')
            return

        tool_name = command.get('tool_name', 'unknown')
        action = command.get('action', 'unknown')
        target_mode = command.get('target_mode', 'named')
        method = command.get('match_method', 'unknown')
        confidence = command.get('confidence', 0.0)

        try:
            confidence_text = f'{float(confidence):.2f}'
        except (TypeError, ValueError):
            confidence_text = str(confidence)

        if self.window is not None and hasattr(self.window, 'append_command_result'):
            self.window.append_command_result(command)
            return

        self._append_system(
            f'parsed: tool={tool_name}, action={action}, target={target_mode}, '
            f'method={method}, confidence={confidence_text}'
        )

    def _feedback_cb(self, msg):
        try:
            feedback = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'/command_feedback JSON 파싱 실패: {msg.data}')
            return

        status = feedback.get('status', 'unknown')
        message = feedback.get('message', '')
        reason = feedback.get('reason', 'unknown')

        if status == 'accepted':
            self._append_bot(message or '명령을 이해했습니다.')
            self._set_status('명령 해석 완료')
            return

        if status == 'pending_confirmation':
            self._append_bot(
                message or '제가 이해한 명령이 맞나요? 네 또는 아니오로 답해주세요.'
            )
            self._set_status('확인 응답 대기')
            return

        if status == 'cancelled':
            self._append_bot(message or '알겠습니다. 실행하지 않겠습니다.')
            self._set_status('명령 취소')
            return

        if status == 'rejected':
            self._append_bot(self._build_rejected_message(reason, message))
            self._set_status('재입력 필요')
            return

        self._append_bot(message or '상태를 확인했습니다.')
        self._append_system(f'status={status}, reason={reason}')

    def _robot_status_cb(self, msg):
        status_text = msg.data.strip()
        if not status_text:
            return

        try:
            status = json.loads(status_text)
        except json.JSONDecodeError:
            self._append_system(f'robot: {status_text}')
            return

        state = status.get('status', status.get('state', 'unknown'))
        tool_name = status.get(
            'tool_name',
            self._last_target_label or 'unknown',
        )
        message = status.get('message', '')

        if state in ('accepted', 'searching', 'picking', 'waiting_handoff'):
            self._last_target_label = tool_name
            self._append_system(message or f'{tool_name}: {state}')
            self._set_status(message or state)
        elif state in ('done', 'completed', 'success'):
            self._append_bot(message or f'{tool_name} 전달 동작이 완료되었습니다.')
            self._set_status('완료')
        elif state in ('failed', 'error'):
            self._append_bot(message or f'{tool_name} 동작 중 문제가 발생했습니다.')
            self._set_status('실패')
        elif state in ('busy', 'cancelled', 'returned', 'rejected'):
            self._append_bot(message or f'{tool_name}: {state}')
            self._set_status(state)
        else:
            self._append_system(message or f'{tool_name}: {state}')

    def _camera_status_cb(self, _msg):
        self._last_camera_stamp_ns = self.get_clock().now().nanoseconds

    def _update_connection_status(self):
        if self.window is None:
            return

        robot_text = self._robot_connection_text()
        camera_text = self._camera_connection_text()
        connection_text = f'{robot_text}|{camera_text}|연결됨'

        if connection_text == self._last_connection_text:
            return

        self._last_connection_text = connection_text
        if hasattr(self.window, 'set_connection_status'):
            self.window.set_connection_status(robot_text, camera_text)

    def _robot_connection_text(self):
        node_names = set(self.get_node_names())
        robot_node_alive = bool(node_names & self._robot_node_names)
        status_publishers = self.get_publishers_info_by_topic('/robot_task_status')

        if robot_node_alive or status_publishers:
            return '실행 중'
        return '미확인'

    def _camera_connection_text(self):
        if self._last_camera_stamp_ns is None:
            return '미확인'

        elapsed_ns = self.get_clock().now().nanoseconds - self._last_camera_stamp_ns
        if elapsed_ns <= self._camera_timeout_ns:
            return '연결됨'
        return '미수신'

    def _build_rejected_message(self, reason, message):
        if reason == 'llm_failed':
            return (
                '문장을 끝까지 이해하지 못했습니다. '
                '드라이버, 드릴, 플라이어, 망치, 렌치, 줄자 중 어떤 공구인지 '
                '조금 더 구체적으로 말해주세요.'
            )
        if reason == 'unknown_tool':
            return (
                '어떤 공구인지 확실하지 않습니다. '
                '드라이버, 드릴, 플라이어, 망치, 렌치, 줄자 중에서 다시 말해주세요.'
            )
        if reason == 'unknown_action':
            return (
                '무엇을 할지 확실하지 않습니다. '
                '가져다줘, 정리해, 멈춰처럼 말해주세요.'
            )
        if reason == 'deictic_bring_not_supported':
            return (
                '가져오기 명령은 공구 이름이 필요합니다. '
                '어떤 공구를 가져올지 말해주세요.'
            )
        if reason == 'low_confidence':
            return (
                '제가 이해한 내용이 확실하지 않습니다. '
                '공구 이름과 동작을 조금 더 명확히 말해주세요.'
            )
        return message or '명령을 이해하지 못했습니다. 다시 입력해주세요.'

    def _append_user(self, text, source='keyboard'):
        if self.window is not None:
            self.window.append_user(text, source=source)

    def _append_bot(self, text):
        if self.window is not None:
            self.window.append_bot(text)

    def _append_system(self, text):
        if self.window is not None:
            self.window.append_system(text)

    def _set_status(self, text):
        if self.window is not None:
            self.window.set_status(text)

    def destroy_node(self):
        if self._stt_service is not None:
            self._stt_service.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CommandInputNode()

    if node._use_gui:
        app = QApplication(sys.argv)
        window = VoiceCommandGuiWindow(node)
        node.attach_window(window)
        window.show()

        timer = QTimer()
        timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
        timer.start(30)

        try:
            exit_code = app.exec_()
        finally:
            timer.stop()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        sys.exit(exit_code)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
