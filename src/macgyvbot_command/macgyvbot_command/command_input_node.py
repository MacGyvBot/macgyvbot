"""Headless command-input node for MacGyvBot.

- 마이크 STT와 `/stt_text` 입력을 명령 해석 파이프라인으로 수집한다.
- LLM 중심 parser를 실행하고 `/tool_command`, `/command_feedback`을 발행한다.
- TTS는 command 패키지에 남겨 MacGyvBot 응답을 음성으로 출력한다.

GUI 표시는 `macgyvbot_ui/operator_ui_node`가 ROS topic만 구독해서 담당한다.
"""

import rclpy
from rclpy.node import Node

from macgyvbot_config.topics import (
    COMMAND_FEEDBACK_TOPIC,
    COMMAND_SHUTDOWN_TOPIC,
    ROBOT_STATUS_TOPIC,
    STT_TEXT_TOPIC,
    TASK_CONTROL_TOPIC,
    TOOL_COMMAND_TOPIC,
)
from macgyvbot_interfaces.msg import (
    CommandFeedback,
    CommandShutdown,
    CommandText,
    RobotTaskControl,
    RobotTaskStatus,
    ToolCommand,
)
from macgyvbot_command.input_mapping.command_hard_parser import (
    find_short_control_action,
)
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
        self.declare_parameter('pause_threshold', 0.45)
        self.declare_parameter('phrase_threshold', 0.15)
        self.declare_parameter('non_speaking_duration', 0.25)
        self.declare_parameter('phrase_time_limit', 3.0)
        self.declare_parameter('dynamic_energy', True)
        self.declare_parameter('ambient_duration', 0.5)
        self.declare_parameter('stt_text_topic', STT_TEXT_TOPIC)

        self.declare_parameter('tool_command_topic', TOOL_COMMAND_TOPIC)
        self.declare_parameter('command_feedback_topic', COMMAND_FEEDBACK_TOPIC)
        self.declare_parameter('robot_status_topic', ROBOT_STATUS_TOPIC)

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

        stt_text_topic = self.get_parameter('stt_text_topic').value
        tool_command_topic = self.get_parameter('tool_command_topic').value
        command_feedback_topic = self.get_parameter('command_feedback_topic').value
        robot_status_topic = self.get_parameter('robot_status_topic').value

        self._exit_pending = False
        self._recent_bot_texts = {}
        self._bot_echo_ignore_ns = 10_000_000_000
        self._stt_pub = self.create_publisher(CommandText, stt_text_topic, 10)
        self._tool_command_pub = self.create_publisher(ToolCommand, tool_command_topic, 10)
        self._feedback_pub = self.create_publisher(
            CommandFeedback, command_feedback_topic, 10
        )
        self._task_control_pub = self.create_publisher(
            RobotTaskControl,
            TASK_CONTROL_TOPIC,
            10,
        )

        self.create_subscription(CommandText, stt_text_topic, self._text_cb, 10)
        self.create_subscription(
            CommandFeedback, command_feedback_topic, self._feedback_cb, 10
        )
        self.create_subscription(
            RobotTaskStatus, robot_status_topic, self._robot_status_cb, 10
        )
        self.create_subscription(
            CommandShutdown, COMMAND_SHUTDOWN_TOPIC, self._shutdown_cb, 10
        )

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
                phrase_threshold=float(self.get_parameter('phrase_threshold').value),
                non_speaking_duration=float(
                    self.get_parameter('non_speaking_duration').value
                ),
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

        msg = CommandText()
        msg.text = text
        msg.source = 'microphone'
        self._stt_pub.publish(msg)

        self.get_logger().info(f'STT 인식 결과: "{text}"')

    def _text_cb(self, msg):
        text = (msg.text or '').strip()
        if not text:
            return

        if self._is_recent_bot_echo(text):
            self.get_logger().info(f'TTS echo로 보이는 입력을 무시합니다: "{text}"')
            return

        self.get_logger().info(f'명령 해석 요청: "{text}"')
        self._handle_text(text)

    def _handle_text(self, text):
        if self._handle_fast_control_text(text):
            return

        result = self._parser.interpret(text)

        command = result.get('command')
        if command is not None:
            action = command.get('action')
            if action in ('pause', 'resume', 'retry', 'cancel'):
                self.get_logger().info(
                    f'작업 제어 명령 해석: action={action}, raw_text="{text}"'
                )
                for payload in result.get('feedbacks', []):
                    self._publish_feedback_payload(payload)
                self._send_task_control_request(action=action, reason=text)
                return
            if action == 'exit':
                if self._exit_pending:
                    self.get_logger().warn('이미 종료 요청을 처리 중입니다.')
                    return
                self._exit_pending = True
                self.get_logger().info(
                    f'종료 제어 명령 해석: action={action}, raw_text="{text}"'
                )
                for payload in result.get('feedbacks', []):
                    self._publish_feedback_payload(payload)
                self._send_task_control_request(action=action, reason=text)
                return
            else:
                self._publish_command(command)

        for payload in result.get('feedbacks', []):
            self._publish_feedback_payload(payload)

    def _handle_fast_control_text(self, text):
        action = find_short_control_action(text)
        if not action:
            return False

        command = {
            'tool_name': 'unknown',
            'action': action,
            'target_mode': 'unknown',
            'raw_text': text,
            'match_method': f'{action}_fast_keyword',
            'match_score': 1.0,
            'confidence': 1.0,
        }
        feedback = {
            'status': 'accepted',
            'reason': 'fast_control_keyword',
            'message': self._fast_control_feedback_message(action),
            'raw_text': text,
            'command': command,
        }

        self.get_logger().info(
            f'짧은 작업 제어 명령 즉시 처리: action={action}, raw_text="{text}"'
        )
        self._publish_feedback_payload(feedback)

        if action in ('pause', 'resume', 'retry', 'cancel'):
            self._send_task_control_request(action=action, reason=text)
            return True

        if action == 'home':
            self._publish_command(command)
            return True

        return False

    @staticmethod
    def _fast_control_feedback_message(action):
        return {
            'pause': '정지 명령으로 이해했습니다.',
            'resume': '재개 명령으로 이해했습니다. 작업 재개 요청으로 전달합니다.',
            'retry': '다시 인식하라는 뜻으로 이해했습니다.',
            'cancel': '현재 작업을 취소합니다. 다음 명령을 기다리겠습니다.',
            'home': 'Home 위치로 복귀하라는 뜻으로 이해했습니다.',
        }.get(action, '명령을 올바른 입력으로 판단했습니다.')

    def _publish_command(self, command):
        command_msg = self._tool_command_message(command)
        self._tool_command_pub.publish(command_msg)
        self.get_logger().info(
            '/tool_command 발행: '
            f'action={command.get("action", "unknown")}, '
            f'tool={command.get("tool_name", "unknown")}'
        )

    def _publish_feedback_payload(self, payload):
        msg = CommandFeedback()
        msg.status = str(payload.get('status', 'unknown'))
        msg.reason = str(payload.get('reason', ''))
        msg.message = str(payload.get('message', ''))
        msg.raw_text = str(payload.get('raw_text', ''))
        msg.command = self._tool_command_message(payload.get('command') or {})
        self._feedback_pub.publish(msg)

    def _send_task_control_request(self, action, reason):
        msg = RobotTaskControl()
        msg.action = action
        msg.reason = reason
        msg.source = 'command_input'
        self._task_control_pub.publish(msg)
        self.get_logger().info(
            f'{TASK_CONTROL_TOPIC} 발행: action={action}, reason={reason}'
        )

    def _shutdown_cb(self, msg):
        if msg.action != 'shutdown':
            return

        self.get_logger().info('operator UI 종료 신호 수신: command_input_node를 종료합니다.')
        if rclpy.ok():
            rclpy.shutdown()

    def _feedback_cb(self, msg):
        feedback = self._feedback_payload(msg)
        speech = self._feedback_speech_message(feedback)
        if speech:
            self._speak_bot(speech)

    def _robot_status_cb(self, msg):
        status = self._robot_status_payload(msg)

        if hasattr(self._parser, 'update_robot_status'):
            self._parser.update_robot_status(status)

        state = str(status.get('status', status.get('state', 'unknown'))).lower()
        if state in self._spoken_robot_statuses():
            message = str(status.get('message') or '').strip()
            if not message:
                message = self._robot_status_default_message(state)
            self._speak_bot(message)

        if self._exit_pending and str(status.get('action', '')).lower() == 'exit':
            if state in ('done', 'completed', 'success'):
                self._exit_pending = False
                self.get_logger().info(
                    '종료 완료 상태 확인: command_input_node를 종료합니다.'
                )
                if rclpy.ok():
                    rclpy.shutdown()
            elif state in ('failed', 'error', 'rejected'):
                self._exit_pending = False
                self.get_logger().warn(
                    '종료 처리가 실패하여 command_input_node를 유지합니다.'
                )

    @staticmethod
    def _tool_command_message(command):
        msg = ToolCommand()
        msg.action = str(command.get('action', 'unknown'))
        msg.tool_name = str(command.get('tool_name', 'unknown'))
        msg.target_mode = str(command.get('target_mode', 'unknown'))
        msg.raw_text = str(command.get('raw_text', ''))
        msg.match_method = str(command.get('match_method', 'unknown'))
        msg.confidence = float(command.get('confidence', 0.0))
        return msg

    @staticmethod
    def _tool_command_payload(msg):
        return {
            'action': msg.action,
            'tool_name': msg.tool_name,
            'target_mode': msg.target_mode,
            'raw_text': msg.raw_text,
            'match_method': msg.match_method,
            'confidence': msg.confidence,
        }

    @staticmethod
    def _feedback_payload(msg):
        return {
            'status': msg.status,
            'reason': msg.reason,
            'message': msg.message,
            'raw_text': msg.raw_text,
            'command': CommandInputNode._tool_command_payload(msg.command),
        }

    @staticmethod
    def _robot_status_payload(msg):
        return {
            'status': msg.status,
            'task': msg.task,
            'tool_name': msg.tool_name,
            'action': msg.action,
            'message': msg.message,
            'reason': msg.reason,
            'command': CommandInputNode._tool_command_payload(msg.command),
        }

    def _feedback_speech_message(self, feedback):
        status = feedback.get('status', 'unknown')
        message = str(feedback.get('message') or '').strip()
        reason = feedback.get('reason', 'unknown')
        command = feedback.get('command') or {}
        action = command.get('action')

        if status == 'accepted':
            if action == 'pause':
                return '정지 요청을 로봇에 전달했습니다. 작업을 재개할까요, 아니면 이번 작업을 취소할까요?'
            if action == 'resume':
                return message or '재개 요청을 이해했습니다.'
            if action == 'cancel':
                return message or '현재 작업 취소 요청을 이해했습니다.'
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
        self._remember_bot_text(text)
        if self._tts_service is not None:
            self._tts_service.speak(text)

    def _remember_bot_text(self, text):
        normalized = self._normalize_echo_text(text)
        if not normalized:
            return

        now = self.get_clock().now().nanoseconds
        self._recent_bot_texts[normalized] = now
        stale_before = now - self._bot_echo_ignore_ns
        self._recent_bot_texts = {
            key: stamp
            for key, stamp in self._recent_bot_texts.items()
            if stamp >= stale_before
        }

    def _is_recent_bot_echo(self, text):
        normalized = self._normalize_echo_text(text)
        if not normalized:
            return False

        stamp = self._recent_bot_texts.get(normalized)
        if stamp is None:
            return False
        return self.get_clock().now().nanoseconds - stamp < self._bot_echo_ignore_ns

    @staticmethod
    def _normalize_echo_text(text):
        return ''.join(str(text or '').split()).lower()

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
