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
from macgyvbot.util.command_input.tts import TtsService


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
        self.declare_parameter('enable_tts', True)
        self.declare_parameter('tts_engine', 'auto')
        self.declare_parameter('tts_voice', 'ko-KR-SunHiNeural')
        self.declare_parameter('tts_rate', 165)
        self.declare_parameter('tts_edge_rate', '+25%')
        self.declare_parameter('tts_pitch', '+35Hz')
        self.declare_parameter('tts_timeout_sec', 20.0)

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
        self._last_robot_status_key = None
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
        self._tts_service = TtsService(
            enabled=bool(self.get_parameter('enable_tts').value),
            engine=self.get_parameter('tts_engine').value,
            voice=self.get_parameter('tts_voice').value,
            rate=int(self.get_parameter('tts_rate').value),
            edge_rate=self.get_parameter('tts_edge_rate').value,
            pitch=self.get_parameter('tts_pitch').value,
            timeout_sec=float(self.get_parameter('tts_timeout_sec').value),
            logger=self._log_tts,
        )
        self._tts_service.start()

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
        if self._tts_service.enabled:
            self.get_logger().info('TTS 모드 활성화: MacGyvBot 응답을 음성으로 출력합니다.')

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

    def _log_tts(self, level, message):
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

        self._stt_pub.publish(msg)
        if self._compat_pub is not None:
            self._compat_pub.publish(msg)

        self.get_logger().info(f'STT 인식 결과: "{text}"')

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
            confirmation_message = (
                message or '제가 이해한 명령이 맞나요? 네 또는 아니오로 답해주세요.'
            )
            if self.window is not None and hasattr(self.window, 'append_confirmation'):
                self.window.append_confirmation(confirmation_message)
                self._speak_bot(confirmation_message)
            else:
                self._append_bot(confirmation_message)
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

        if hasattr(self._parser, 'update_robot_status'):
            self._parser.update_robot_status(status)

        view = self._build_robot_status_view(status)
        self._last_target_label = view['target_label']
        self._set_status(view['panel_status'])
        self._set_task_status(view['target_label'], view['stage_text'])

        if view['show_chat']:
            self._append_bot(view['message'], speak=view['speak'])

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

    def _build_robot_status_view(self, status):
        state = str(status.get('status', status.get('state', 'unknown'))).strip()
        if not state:
            state = 'unknown'
        state = state.lower()

        tool_name = status.get('tool_name') or self._last_target_label or 'unknown'
        target_label = self._tool_display_name(tool_name)
        raw_message = str(status.get('message') or '').strip()
        reason = str(status.get('reason') or '').strip()

        message = self._robot_status_message(state, target_label, raw_message, reason)
        panel_status = self._robot_panel_status(state, message)
        stage_text = self._robot_stage_text(state, message)
        speak = self._should_speak_robot_status(state)

        key = (state, str(tool_name), raw_message or message)
        force_show = state in self._always_show_robot_statuses()
        show_chat = force_show or key != self._last_robot_status_key
        if show_chat:
            self._last_robot_status_key = key

        return {
            'state': state,
            'message': message,
            'panel_status': panel_status,
            'stage_text': stage_text,
            'target_label': target_label,
            'speak': speak and show_chat,
            'show_chat': show_chat,
        }

    def _robot_status_message(self, state, target_label, raw_message, reason):
        if raw_message:
            if state in ('failed', 'error') and reason and reason not in raw_message:
                return f'{raw_message} 원인: {reason}'
            return raw_message

        templates = {
            'accepted': f'요청을 확인했습니다. {target_label} 작업을 시작할게요.',
            'searching_drawer': f'{target_label}를 꺼낼 공구함을 찾는 중입니다.',
            'moving_to_drawer': '공구함으로 이동 중입니다.',
            'searching_drawer_handle': '서랍 손잡이를 찾는 중입니다.',
            'opening_drawer': '서랍을 여는 중입니다.',
            'closing_drawer': '서랍 문을 닫는 중입니다.',
            'searching': f'{target_label}를 찾는 중입니다.',
            'picking': f'{target_label}를 집는 위치로 이동 중입니다.',
            'approaching_tool': f'{target_label} 상단으로 접근 중입니다.',
            'grasping': '공구를 잡는 중입니다.',
            'grasp_success': '공구를 안정적으로 잡았습니다.',
            'lifting_tool': '공구를 안전 높이로 들어 올리는 중입니다.',
            'moving_to_handoff': '사용자 전달 위치로 이동 중입니다.',
            'searching_hand': '사용자 손을 찾는 중입니다.',
            'waiting_handoff': '손으로 공구를 잡아주세요.',
            'handoff_complete': '공구 전달을 완료했습니다.',
            'waiting_return_handoff': '반납할 공구를 받을 준비를 하고 있습니다.',
            'moving_return_grasp_pose': '반납 공구를 감지할 위치로 이동 중입니다.',
            'placing_return_tool': f'{target_label}를 보관 위치에 놓는 중입니다.',
            'returning_home': 'Home 위치로 복귀하는 중입니다.',
            'done': '작업이 완료되었습니다.',
            'completed': '작업이 완료되었습니다.',
            'success': '작업이 완료되었습니다.',
            'failed': '작업에 실패했습니다.',
            'error': '작업 중 오류가 발생했습니다.',
            'busy': '이미 다른 작업을 수행 중입니다.',
            'paused': '로봇이 일시정지되었습니다.',
            'resumed': '작업을 다시 시작합니다.',
            'cancelled': '작업이 취소되었습니다.',
            'returned': '반납 작업을 완료했습니다.',
            'rejected': '요청을 수행할 수 없습니다.',
        }

        message = templates.get(state, f'현재 작업 상태는 {state}입니다.')
        if state in ('failed', 'error') and reason:
            return f'{message} 원인: {reason}'
        return message

    def _robot_panel_status(self, state, message):
        labels = {
            'accepted': '요청 확인',
            'searching_drawer': '공구함 탐색',
            'moving_to_drawer': '공구함 이동',
            'searching_drawer_handle': '손잡이 탐색',
            'opening_drawer': '서랍 열기',
            'closing_drawer': '서랍 닫기',
            'searching': '공구 탐색',
            'picking': '공구 접근',
            'approaching_tool': '공구 접근',
            'grasping': '공구 파지',
            'grasp_success': '파지 성공',
            'lifting_tool': '공구 들어올림',
            'moving_to_handoff': '전달 위치 이동',
            'searching_hand': '손 탐색',
            'waiting_handoff': '전달 대기',
            'handoff_complete': '전달 완료',
            'waiting_return_handoff': '반납 대기',
            'moving_return_grasp_pose': '반납 위치 이동',
            'placing_return_tool': '공구 보관',
            'returning_home': 'Home 복귀',
            'done': '완료',
            'completed': '완료',
            'success': '완료',
            'failed': '실패',
            'error': '오류',
            'busy': '작업 중',
            'paused': '일시정지',
            'resumed': '재개',
            'cancelled': '취소',
            'returned': '반납 완료',
            'rejected': '거절',
        }
        return labels.get(state, message)

    def _robot_stage_text(self, state, message):
        stage_labels = {
            'accepted': '요청 확인',
            'searching_drawer': '공구함 찾는 중',
            'moving_to_drawer': '공구함 이동 중',
            'searching_drawer_handle': '서랍 손잡이 찾는 중',
            'opening_drawer': '서랍 여는 중',
            'closing_drawer': '서랍 닫는 중',
            'searching': '공구 찾는 중',
            'picking': '공구 집는 위치로 이동 중',
            'approaching_tool': '공구 접근 중',
            'grasping': '공구 잡는 중',
            'grasp_success': '공구 잡기 성공',
            'lifting_tool': '안전 높이로 이동 중',
            'moving_to_handoff': '전달 위치 이동 중',
            'searching_hand': '사용자 손 찾는 중',
            'waiting_handoff': '사용자 잡기 대기',
            'handoff_complete': '공구 전달 완료',
            'waiting_return_handoff': '반납 공구 수령 대기',
            'moving_return_grasp_pose': '반납 공구 감지 위치 이동 중',
            'placing_return_tool': '서랍 안에 공구 보관 중',
            'returning_home': 'Home 복귀 중',
            'done': '작업 완료',
            'completed': '작업 완료',
            'success': '작업 완료',
            'failed': '작업 실패',
            'error': '작업 오류',
            'busy': '다른 작업 수행 중',
            'paused': '작업 일시정지',
            'resumed': '작업 재개',
            'cancelled': '작업 취소',
            'returned': '반납 완료',
            'rejected': '작업 거절',
        }
        return stage_labels.get(state, message)

    def _should_speak_robot_status(self, state):
        return state in {
            'waiting_handoff',
            'waiting_return_handoff',
            'done',
            'completed',
            'success',
            'failed',
            'error',
            'busy',
            'paused',
            'resumed',
            'cancelled',
            'rejected',
        }

    def _always_show_robot_statuses(self):
        return {
            'waiting_handoff',
            'waiting_return_handoff',
            'done',
            'completed',
            'success',
            'failed',
            'error',
            'busy',
            'paused',
            'resumed',
            'cancelled',
            'rejected',
            'grasp_success',
            'handoff_complete',
        }

    def _tool_display_name(self, tool_name):
        label = str(tool_name or 'unknown').strip()
        display_names = {
            'screwdriver': '드라이버',
            'pliers': '플라이어',
            'hammer': '망치',
            'tape_measure': '줄자',
            'drill': '드릴',
            'wrench': '렌치',
            'unknown': '공구',
        }
        return display_names.get(label, label)

    def _append_user(self, text, source='keyboard'):
        if self.window is not None:
            self.window.append_user(text, source=source)

    def _append_bot(self, text, speak=True):
        if self.window is not None:
            self.window.append_bot(text)
        if speak:
            self._speak_bot(text)

    def _append_system(self, text):
        if self.window is not None:
            self.window.append_system(text)

    def _speak_bot(self, text):
        if hasattr(self, '_tts_service') and self._tts_service is not None:
            self._tts_service.speak(text)

    def _set_status(self, text):
        if self.window is not None:
            self.window.set_status(text)

    def _set_task_status(self, target_text, stage_text):
        if self.window is not None and hasattr(self.window, 'set_task_status'):
            self.window.set_task_status(target_text, stage_text)

    def destroy_node(self):
        if self._stt_service is not None:
            self._stt_service.stop()
        if self._tts_service is not None:
            self._tts_service.stop()
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
