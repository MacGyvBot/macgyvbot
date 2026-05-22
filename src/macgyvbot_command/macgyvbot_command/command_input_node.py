"""Headless command-input node for MacGyvBot.

- 마이크 STT와 `/stt_text` 입력을 명령 해석 파이프라인으로 수집한다.
- LLM 중심 parser를 실행하고 `/tool_command`, `/command_feedback`을 발행한다.
- TTS는 command 패키지에 남겨 MacGyvBot 응답을 음성으로 출력한다.

GUI 표시는 `macgyvbot_ui/operator_ui_node`가 ROS topic만 구독해서 담당한다.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from macgyvbot_command.input_mapping.command_llm_parser import CommandLlmParser
from macgyvbot_command.stt.speech_to_text import SpeechToTextService
from macgyvbot_command.tts import TtsService


class CommandInputNode(Node):
    def __init__(self):
        super().__init__('command_input_node')

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

        self.declare_parameter(
            'ollama_url',
            'http://localhost:11434/api/generate',
        )
        self.declare_parameter('model', 'gemma3:1b')
        self.declare_parameter('timeout_sec', 25.0)
        self.declare_parameter('min_confidence', 0.55)
        self.declare_parameter('use_local_parser', True)
        self.declare_parameter('use_llm_fallback', True)
        self.declare_parameter('parser_mode', 'llm_primary')
        self.declare_parameter('enable_tts', True)
        self.declare_parameter('tts_engine', 'auto')
        self.declare_parameter('tts_voice', 'ko-KR-SunHiNeural')
        self.declare_parameter('tts_rate', 165)
        self.declare_parameter('tts_edge_rate', '+25%')
        self.declare_parameter('tts_pitch', '+35Hz')
        self.declare_parameter('tts_timeout_sec', 20.0)

        self._enable_microphone = bool(self.get_parameter('enable_microphone').value)
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

        self._stt_pub = self.create_publisher(String, stt_text_topic, 10)
        self._compat_pub = (
            self.create_publisher(String, compat_topic, 10)
            if self._publish_compat
            else None
        )
        self._tool_command_pub = self.create_publisher(String, tool_command_topic, 10)
        self._feedback_pub = self.create_publisher(String, command_feedback_topic, 10)

        self.create_subscription(String, stt_text_topic, self._text_cb, 10)
        self.create_subscription(String, command_feedback_topic, self._feedback_cb, 10)
        self.create_subscription(String, robot_status_topic, self._robot_status_cb, 10)

        self._parser = CommandLlmParser(
            ollama_url=self.get_parameter('ollama_url').value,
            model=self.get_parameter('model').value,
            timeout_sec=float(self.get_parameter('timeout_sec').value),
            min_confidence=float(self.get_parameter('min_confidence').value),
            use_local_parser=bool(self.get_parameter('use_local_parser').value),
            use_llm_fallback=bool(self.get_parameter('use_llm_fallback').value),
            parser_mode=self.get_parameter('parser_mode').value,
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
        self.get_logger().info(f'로봇 상태 topic: {robot_status_topic}')
        if self._publish_compat:
            self.get_logger().info(f'수업 예제 호환 topic: {compat_topic}')
        if self._tts_service.enabled:
            self.get_logger().info('TTS 모드 활성화: MacGyvBot 응답을 음성으로 출력합니다.')

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

        self.get_logger().info(f'명령 해석 요청: "{text}"')
        self._handle_text(text)

    def _handle_text(self, text):
        result = self._parser.interpret(text)

        command = result.get('command')
        if command is not None:
            action = command.get('action')
            if action in ('resume', 'exit'):
                self.get_logger().info(
                    f'로컬 제어 명령 해석: action={action}, raw_text="{text}"'
                )
            else:
                self._publish_command(command)

        for payload in result.get('feedbacks', []):
            self._publish_feedback_payload(payload)

    def _publish_command(self, command):
        command_msg = String()
        command_msg.data = json.dumps(command, ensure_ascii=False)
        self._tool_command_pub.publish(command_msg)
        self.get_logger().info(
            '/tool_command 발행: '
            f'action={command.get("action", "unknown")}, '
            f'tool={command.get("tool_name", "unknown")}'
        )

    def _publish_feedback_payload(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._feedback_pub.publish(msg)

    def _feedback_cb(self, msg):
        try:
            feedback = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'/command_feedback JSON 파싱 실패: {msg.data}')
            return

        speech = self._feedback_speech_message(feedback)
        if speech:
            self._speak_bot(speech)

    def _robot_status_cb(self, msg):
        status_text = msg.data.strip()
        if not status_text:
            return

        try:
            status = json.loads(status_text)
        except json.JSONDecodeError:
            self.get_logger().warn(f'/robot_task_status JSON 파싱 실패: {status_text}')
            return

        if hasattr(self._parser, 'update_robot_status'):
            self._parser.update_robot_status(status)

        state = str(status.get('status', status.get('state', 'unknown'))).lower()
        if state in self._spoken_robot_statuses():
            message = str(status.get('message') or '').strip()
            if not message:
                message = self._robot_status_default_message(state)
            self._speak_bot(message)

    def _feedback_speech_message(self, feedback):
        status = feedback.get('status', 'unknown')
        message = str(feedback.get('message') or '').strip()
        reason = feedback.get('reason', 'unknown')
        command = feedback.get('command') or {}
        action = command.get('action')

        if status == 'accepted':
            if action == 'pause':
                return '정지 요청을 로봇에 전달했습니다. 작업을 재개할까요, 아니면 종료할까요?'
            if action == 'resume':
                return message or '재개 요청을 이해했습니다.'
            if action == 'exit':
                return message or '종료 요청을 이해했습니다.'
            return message or '명령을 이해했습니다.'

        if status == 'pending_confirmation':
            return message or '제가 이해한 명령이 맞나요? 네 또는 아니오로 답해주세요.'

        if status == 'cancelled':
            return message or '알겠습니다. 실행하지 않겠습니다.'

        if status == 'assistant_response':
            return message or '네, 필요한 공구가 있으면 언제든 말해주세요.'

        if status == 'rejected':
            return self._build_rejected_message(reason, message)

        return ''

    def _build_rejected_message(self, reason, message):
        if reason == 'llm_failed':
            return (
                '문장을 끝까지 이해하지 못했습니다. '
                '드라이버, 플라이어, 망치, 줄자 중 어떤 공구인지 '
                '조금 더 구체적으로 말해주세요.'
            )
        if reason == 'unknown_tool':
            return (
                '어떤 공구인지 확실하지 않습니다. '
                '드라이버, 플라이어, 망치, 줄자 중에서 다시 말해주세요.'
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

    @staticmethod
    def _spoken_robot_statuses():
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
        }

    @staticmethod
    def _robot_status_default_message(state):
        return {
            'waiting_handoff': '손으로 공구를 잡아주세요.',
            'waiting_return_handoff': '반납할 공구를 받을 준비를 하고 있습니다.',
            'done': '작업이 완료되었습니다.',
            'completed': '작업이 완료되었습니다.',
            'success': '작업이 완료되었습니다.',
            'failed': '작업에 실패했습니다.',
            'error': '작업 중 오류가 발생했습니다.',
            'busy': '이미 다른 작업을 수행 중입니다.',
            'paused': '로봇이 일시정지되었습니다.',
            'resumed': '작업을 다시 시작합니다.',
            'cancelled': '작업이 취소되었습니다.',
            'rejected': '요청을 수행할 수 없습니다.',
        }.get(state, '')

    def _speak_bot(self, text):
        if self._tts_service is not None:
            self._tts_service.speak(text)

    def destroy_node(self):
        if self._stt_service is not None:
            self._stt_service.stop()
        if self._tts_service is not None:
            self._tts_service.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CommandInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
