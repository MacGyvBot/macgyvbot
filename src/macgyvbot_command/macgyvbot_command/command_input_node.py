"""Headless command-input node for MacGyvBot."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from macgyvbot_command.input_mapping.command_hard_parser import (
    find_short_control_action,
)
from macgyvbot_command.input_mapping.command_llm_parser import CommandLlmParser
from macgyvbot_command.stt.speech_to_text import SpeechToTextService
from macgyvbot_command.tts import TtsService
from macgyvbot_config.command import (
    DEFAULT_COMMAND_MIN_CONFIDENCE,
    DEFAULT_ENABLE_MICROPHONE,
    DEFAULT_ENABLE_TTS,
    DEFAULT_LLM_MODEL,
    DEFAULT_LLM_TIMEOUT_SEC,
    DEFAULT_OLLAMA_URL,
    DEFAULT_PARSER_MODE,
    DEFAULT_STT_AMBIENT_DURATION_SEC,
    DEFAULT_STT_DEVICE_INDEX,
    DEFAULT_STT_DYNAMIC_ENERGY,
    DEFAULT_STT_ENERGY_THRESHOLD,
    DEFAULT_STT_LANGUAGE,
    DEFAULT_STT_NON_SPEAKING_DURATION_SEC,
    DEFAULT_STT_PAUSE_THRESHOLD,
    DEFAULT_STT_PHRASE_THRESHOLD,
    DEFAULT_STT_PHRASE_TIME_LIMIT_SEC,
    DEFAULT_TTS_EDGE_RATE,
    DEFAULT_TTS_ENGINE,
    DEFAULT_TTS_PITCH,
    DEFAULT_TTS_RATE,
    DEFAULT_TTS_TIMEOUT_SEC,
    DEFAULT_TTS_VOICE,
    DEFAULT_USE_LLM_FALLBACK,
    DEFAULT_USE_LOCAL_PARSER,
)
from macgyvbot_config.topics import (
    COMMAND_FEEDBACK_TOPIC,
    COMMAND_SHUTDOWN_TOPIC,
    ROBOT_STATUS_TOPIC,
    STT_TEXT_TOPIC,
    TASK_CONTROL_TOPIC,
    TOOL_COMMAND_TOPIC,
)
from macgyvbot_config.structured_logging import format_structured_log
from macgyvbot_interfaces.msg import (
    CommandFeedback,
    CommandShutdown,
    CommandText,
    RobotTaskControl,
    RobotTaskStatus,
    ToolCommand,
)

def _format_command_log(message, *, step="log", event="status", **fields):
    return format_structured_log(
        svc="command",
        pipe="input",
        step=step,
        event=event,
        msg=message,
        **fields,
    )


class _StructuredLoggerAdapter:
    def __init__(self, logger):
        self._logger = logger

    def debug(self, message):
        self._logger.debug(self._format(message))

    def info(self, message):
        self._logger.info(self._format(message))

    def warn(self, message):
        self._logger.warn(self._format(message))

    def warning(self, message):
        self.warn(message)

    def error(self, message):
        self._logger.error(self._format(message))

    @staticmethod
    def _format(message):
        text = str(message or "")
        if text.startswith("[pkg] ") or text.startswith("pkg="):
            return text
        return _format_command_log(text)


class CommandInputNode(Node):
    def __init__(self):
        super().__init__("command_input_node")

        self.declare_parameter("enable_microphone", DEFAULT_ENABLE_MICROPHONE)
        self.declare_parameter("language", DEFAULT_STT_LANGUAGE)
        self.declare_parameter("device_index", DEFAULT_STT_DEVICE_INDEX)
        self.declare_parameter("energy_threshold", DEFAULT_STT_ENERGY_THRESHOLD)
        self.declare_parameter("pause_threshold", DEFAULT_STT_PAUSE_THRESHOLD)
        self.declare_parameter("phrase_threshold", DEFAULT_STT_PHRASE_THRESHOLD)
        self.declare_parameter(
            "non_speaking_duration",
            DEFAULT_STT_NON_SPEAKING_DURATION_SEC,
        )
        self.declare_parameter(
            "phrase_time_limit",
            DEFAULT_STT_PHRASE_TIME_LIMIT_SEC,
        )
        self.declare_parameter("dynamic_energy", DEFAULT_STT_DYNAMIC_ENERGY)
        self.declare_parameter("ambient_duration", DEFAULT_STT_AMBIENT_DURATION_SEC)
        self.declare_parameter("stt_text_topic", STT_TEXT_TOPIC)

        self.declare_parameter("tool_command_topic", TOOL_COMMAND_TOPIC)
        self.declare_parameter("command_feedback_topic", COMMAND_FEEDBACK_TOPIC)
        self.declare_parameter("robot_status_topic", ROBOT_STATUS_TOPIC)

        self.declare_parameter("ollama_url", DEFAULT_OLLAMA_URL)
        self.declare_parameter("model", DEFAULT_LLM_MODEL)
        self.declare_parameter("timeout_sec", DEFAULT_LLM_TIMEOUT_SEC)
        self.declare_parameter("min_confidence", DEFAULT_COMMAND_MIN_CONFIDENCE)
        self.declare_parameter("use_local_parser", DEFAULT_USE_LOCAL_PARSER)
        self.declare_parameter("use_llm_fallback", DEFAULT_USE_LLM_FALLBACK)
        self.declare_parameter("parser_mode", DEFAULT_PARSER_MODE)
        self.declare_parameter("enable_tts", DEFAULT_ENABLE_TTS)
        self.declare_parameter("tts_engine", DEFAULT_TTS_ENGINE)
        self.declare_parameter("tts_voice", DEFAULT_TTS_VOICE)
        self.declare_parameter("tts_rate", DEFAULT_TTS_RATE)
        self.declare_parameter("tts_edge_rate", DEFAULT_TTS_EDGE_RATE)
        self.declare_parameter("tts_pitch", DEFAULT_TTS_PITCH)
        self.declare_parameter("tts_timeout_sec", DEFAULT_TTS_TIMEOUT_SEC)

        self._enable_microphone = bool(self.get_parameter("enable_microphone").value)
        self._language = self.get_parameter("language").value
        self._device_index = int(self.get_parameter("device_index").value)
        self._phrase_time_limit = float(self.get_parameter("phrase_time_limit").value)

        stt_text_topic = self.get_parameter("stt_text_topic").value
        tool_command_topic = self.get_parameter("tool_command_topic").value
        command_feedback_topic = self.get_parameter("command_feedback_topic").value
        robot_status_topic = self.get_parameter("robot_status_topic").value

        self._exit_pending = False
        self._last_robot_state = "unknown"
        self._recent_bot_texts = {}
        self._bot_echo_ignore_ns = 10_000_000_000
        self._last_spoken_robot_status_key = None
        self._stt_pub = self.create_publisher(CommandText, stt_text_topic, 10)
        self._tool_command_pub = self.create_publisher(ToolCommand, tool_command_topic, 10)
        self._feedback_pub = self.create_publisher(
            CommandFeedback,
            command_feedback_topic,
            10,
        )
        self._task_control_pub = self.create_publisher(
            RobotTaskControl,
            TASK_CONTROL_TOPIC,
            10,
        )

        self.create_subscription(CommandText, stt_text_topic, self._text_cb, 10)
        self.create_subscription(
            CommandFeedback,
            command_feedback_topic,
            self._feedback_cb,
            10,
        )
        self.create_subscription(
            RobotTaskStatus,
            robot_status_topic,
            self._robot_status_cb,
            10,
        )
        self.create_subscription(
            CommandShutdown,
            COMMAND_SHUTDOWN_TOPIC,
            self._shutdown_cb,
            10,
        )

        self._parser = CommandLlmParser(
            ollama_url=self.get_parameter("ollama_url").value,
            model=self.get_parameter("model").value,
            timeout_sec=float(self.get_parameter("timeout_sec").value),
            min_confidence=float(self.get_parameter("min_confidence").value),
            use_local_parser=bool(self.get_parameter("use_local_parser").value),
            use_llm_fallback=bool(self.get_parameter("use_llm_fallback").value),
            parser_mode=self.get_parameter("parser_mode").value,
            logger=self._log_parser,
        )
        self._tts_service = TtsService(
            enabled=bool(self.get_parameter("enable_tts").value),
            engine=self.get_parameter("tts_engine").value,
            voice=self.get_parameter("tts_voice").value,
            rate=int(self.get_parameter("tts_rate").value),
            edge_rate=self.get_parameter("tts_edge_rate").value,
            pitch=self.get_parameter("tts_pitch").value,
            timeout_sec=float(self.get_parameter("tts_timeout_sec").value),
            logger=self._log_tts,
        )
        self._tts_service.start()

        self._stt_service = None
        if self._enable_microphone:
            self._stt_service = SpeechToTextService(
                language=self._language,
                device_index=self._device_index,
                energy_threshold=float(self.get_parameter("energy_threshold").value),
                pause_threshold=float(self.get_parameter("pause_threshold").value),
                phrase_threshold=float(self.get_parameter("phrase_threshold").value),
                non_speaking_duration=float(
                    self.get_parameter("non_speaking_duration").value
                ),
                dynamic_energy=bool(self.get_parameter("dynamic_energy").value),
                ambient_duration=float(self.get_parameter("ambient_duration").value),
                phrase_time_limit=self._phrase_time_limit,
                logger=self._log_stt,
            )
            self._stt_service.start(self._on_stt_text)
            self.get_logger().info(
                _format_command_log(
                    "stt ready",
                    step="startup",
                    event="ready",
                    language=self._language,
                    device_index=self._device_index,
                    phrase_time_limit_sec=self._phrase_time_limit,
                )
            )
        else:
            self.get_logger().info(
                _format_command_log(
                    "microphone stt disabled",
                    step="startup",
                    event="disabled",
                )
            )

        self.get_logger().info(
            _format_command_log(
                "command input ready",
                step="startup",
                event="ready",
                stt_text_topic=stt_text_topic,
                tool_command_topic=tool_command_topic,
                command_feedback_topic=command_feedback_topic,
                robot_status_topic=robot_status_topic,
                tts_enabled=self._tts_service.enabled,
            )
        )

    def get_logger(self):
        return _StructuredLoggerAdapter(super().get_logger())

    def _log_stt(self, level, message):
        logger = self.get_logger()
        if level == "warn":
            logger.warn(message)
            return
        if level == "error":
            logger.error(message)
            return
        logger.info(message)

    def _log_parser(self, level, message):
        self._log_stt(level, message)

    def _log_tts(self, level, message):
        self._log_stt(level, message)

    def _on_stt_text(self, text):
        text = (text or "").strip()
        if not text:
            return

        msg = CommandText()
        msg.text = text
        msg.source = "microphone"
        self._stt_pub.publish(msg)

        self.get_logger().info(
            _format_command_log(
                "stt text received",
                step="stt",
                event="received",
                text=text,
            )
        )

    def _text_cb(self, msg):
        text = (msg.text or "").strip()
        if not text:
            return

        if self._is_recent_bot_echo(text):
            self.get_logger().info(
                _format_command_log(
                    "ignored recent tts echo",
                    step="input",
                    event="skip",
                    text=text,
                )
            )
            return

        self.get_logger().info(
            _format_command_log(
                "command interpretation requested",
                step="interpret",
                event="start",
                text=text,
            )
        )
        self._handle_text(text)

    def _handle_text(self, text):
        if self._handle_fast_control_text(text):
            return

        result = self._parser.interpret(text)

        command = result.get("command")
        if command is not None:
            action = command.get("action")
            if action in ("pause", "resume", "retry", "cancel"):
                self.get_logger().info(
                    _format_command_log(
                        "task control interpreted",
                        step="interpret",
                        event="accepted",
                        action=action,
                        raw_text=text,
                    )
                )
                for payload in result.get("feedbacks", []):
                    self._publish_feedback_payload(payload)
                self._send_task_control_request(action=action, reason=text)
                return
            if action == "exit":
                if self._exit_pending:
                    self.get_logger().warn(
                        _format_command_log(
                            "exit already pending",
                            step="exit",
                            event="skip",
                        )
                    )
                    return
                self._exit_pending = True
                self.get_logger().info(
                    _format_command_log(
                        "exit interpreted",
                        step="exit",
                        event="accepted",
                        raw_text=text,
                    )
                )
                for payload in result.get("feedbacks", []):
                    self._publish_feedback_payload(payload)
                self._send_task_control_request(action=action, reason=text)
                return
            if action == "home" and self._handoff_decision_pending():
                self.get_logger().info(
                    _format_command_log(
                        "home interpreted as handoff fallback",
                        step="interpret",
                        event="reroute",
                        action=action,
                        raw_text=text,
                    )
                )
                for payload in result.get("feedbacks", []):
                    self._publish_feedback_payload(payload)
                self._send_task_control_request(action="cancel", reason=text)
                return
            self._publish_command(command)

        for payload in result.get("feedbacks", []):
            self._publish_feedback_payload(payload)

    def _handle_fast_control_text(self, text):
        action = find_short_control_action(text)
        if not action:
            return False

        command = {
            "tool_name": "unknown",
            "action": action,
            "target_mode": "unknown",
            "raw_text": text,
            "match_method": f"{action}_fast_keyword",
            "match_score": 1.0,
            "confidence": 1.0,
        }
        feedback = {
            "status": "accepted",
            "reason": "fast_control_keyword",
            "message": self._fast_control_feedback_message(action),
            "raw_text": text,
            "command": command,
        }

        self.get_logger().info(
            _format_command_log(
                "fast control command matched",
                step="fast_control",
                event="accepted",
                action=action,
                raw_text=text,
            )
        )
        self._publish_feedback_payload(feedback)

        if action in ("pause", "resume", "retry", "cancel"):
            self._send_task_control_request(action=action, reason=text)
            return True

        if action == "home":
            if self._handoff_decision_pending():
                self._send_task_control_request(action="cancel", reason=text)
                return True
            self._publish_command(command)
            return True

        return False

    @staticmethod
    def _fast_control_feedback_message(action):
        return {
            "pause": "정지 명령으로 이해했습니다.",
            "resume": "재개 명령으로 이해했습니다. 작업 재개 요청으로 전달합니다.",
            "retry": "다시 인식하라는 뜻으로 이해했습니다.",
            "cancel": "현재 작업을 취소합니다. 다음 명령을 기다리겠습니다.",
            "home": "Home으로 복귀합니다.",
        }.get(action, "명령을 올바른 입력으로 판단했습니다.")

    def _publish_command(self, command):
        command_msg = self._tool_command_message(command)
        self._tool_command_pub.publish(command_msg)
        self.get_logger().info(
            _format_command_log(
                "tool command published",
                step="tool_command",
                event="publish",
                topic="/tool_command",
                action=command.get("action", "unknown"),
                tool=command.get("tool_name", "unknown"),
            )
        )

    def _publish_feedback_payload(self, payload):
        msg = CommandFeedback()
        msg.status = str(payload.get("status", "unknown"))
        msg.reason = str(payload.get("reason", ""))
        msg.message = str(payload.get("message", ""))
        msg.raw_text = str(payload.get("raw_text", ""))
        msg.command = self._tool_command_message(payload.get("command") or {})
        self._feedback_pub.publish(msg)

    def _send_task_control_request(self, action, reason):
        msg = RobotTaskControl()
        msg.action = action
        msg.reason = reason
        msg.source = "command_input"
        self._task_control_pub.publish(msg)
        self.get_logger().info(
            _format_command_log(
                "task control published",
                step="task_control",
                event="publish",
                topic=TASK_CONTROL_TOPIC,
                action=action,
                reason=reason,
            )
        )

    def _shutdown_cb(self, msg):
        if msg.action != "shutdown":
            return

        self.get_logger().info(
            _format_command_log(
                "operator ui shutdown received",
                step="shutdown",
                event="received",
            )
        )
        if rclpy.ok():
            rclpy.shutdown()

    def _feedback_cb(self, msg):
        feedback = self._feedback_payload(msg)
        speech = self._feedback_speech_message(feedback)
        if speech:
            self._speak_bot(speech)

    def _robot_status_cb(self, msg):
        status = self._robot_status_payload(msg)

        if hasattr(self._parser, "update_robot_status"):
            self._parser.update_robot_status(status)

        state = str(status.get("status", status.get("state", "unknown"))).lower()
        self._last_robot_state = state
        if state in self._spoken_robot_statuses():
            message = self._robot_status_speech_message(status)
            if not message:
                message = self._robot_status_default_message(state)
            spoken_key = (
                state,
                str(status.get('action', '')).strip().lower(),
                message,
            )
            if message and spoken_key != self._last_spoken_robot_status_key:
                self._last_spoken_robot_status_key = spoken_key
                self._speak_bot(message)

        if self._exit_pending and str(status.get("action", "")).lower() == "exit":
            if state in ("done", "completed", "success"):
                self._exit_pending = False
                self.get_logger().info(
                    _format_command_log(
                        "exit completed",
                        step="exit",
                        event="done",
                    )
                )
                if rclpy.ok():
                    rclpy.shutdown()
            elif state in ("failed", "error", "rejected"):
                self._exit_pending = False
                self.get_logger().warn(
                    _format_command_log(
                        "exit failed",
                        step="exit",
                        event="fail",
                    )
                )

    @staticmethod
    def _tool_command_message(command):
        msg = ToolCommand()
        msg.action = str(command.get("action", "unknown"))
        msg.tool_name = str(command.get("tool_name", "unknown"))
        msg.target_mode = str(command.get("target_mode", "unknown"))
        msg.raw_text = str(command.get("raw_text", ""))
        msg.match_method = str(command.get("match_method", "unknown"))
        msg.confidence = float(command.get("confidence", 0.0))
        return msg

    @staticmethod
    def _tool_command_payload(msg):
        return {
            "action": msg.action,
            "tool_name": msg.tool_name,
            "target_mode": msg.target_mode,
            "raw_text": msg.raw_text,
            "match_method": msg.match_method,
            "confidence": msg.confidence,
        }

    @staticmethod
    def _feedback_payload(msg):
        return {
            "status": msg.status,
            "reason": msg.reason,
            "message": msg.message,
            "raw_text": msg.raw_text,
            "command": CommandInputNode._tool_command_payload(msg.command),
        }

    @staticmethod
    def _robot_status_payload(msg):
        return {
            "status": msg.status,
            "task": msg.task,
            "tool_name": msg.tool_name,
            "action": msg.action,
            "message": msg.message,
            "reason": msg.reason,
            "command": CommandInputNode._tool_command_payload(msg.command),
        }

    def _handoff_decision_pending(self):
        return self._last_robot_state == "handoff_inspection_pending"

    def _feedback_speech_message(self, feedback):
        status = feedback.get("status", "unknown")
        message = str(feedback.get("message") or "").strip()
        reason = feedback.get("reason", "unknown")
        command = feedback.get("command") or {}
        action = command.get("action")

        if status == "accepted":
            if action == "pause":
                return "일시 정지 요청을 전달했습니다. 작업을 다시 시작하려면 재개 명령을 말씀해 주세요."
            if action == "resume":
                return message or "재개 요청을 전달했습니다."
            if action == "cancel":
                return message or "현재 작업 취소 요청을 전달했습니다."
            if action == "exit":
                return message or "종료 요청을 전달했습니다."
            return message or "명령을 이해했습니다."

        if status == "pending_confirmation":
            return message or "제가 이해한 명령이 맞는지 확인해 주세요."

        if status == "cancelled":
            return message or "요청이 취소되었습니다."

        if status == "assistant_response":
            return message or "다시 한 번 말씀해 주세요."

        if status == "rejected":
            return self._build_rejected_message(reason, message)

        return ""

    def _robot_status_speech_message(self, status):
        state = str(status.get("status", status.get("state", "unknown"))).lower()
        message = str(status.get("message") or "").strip()
        reason = str(status.get("reason") or "").strip()
        if state not in ("failed", "error"):
            return message

        tool_name = str(status.get("tool_name") or "").strip() or "공구"
        if reason == "pick_drawer_tool_not_found":
            return f"{tool_name}를 서랍에서 찾지 못했습니다."
        if reason in {"drawer_prepare_failed", "drawer_open_failed"}:
            return "서랍을 여는 중 문제가 발생했습니다."
        if reason in {"pose_goal_plan_failed", "planning_failed"}:
            return "이동 경로를 찾지 못했습니다."
        if reason in {"handoff_pose_unavailable", "handoff_target_unavailable"}:
            return "전달 위치를 계산하지 못했습니다."
        if reason in {"home_motion_failed", "home_pose_unavailable"}:
            return "홈 위치 복귀에 실패했습니다."
        if reason in {"tool_mask_lock_failed", "mask_lock_failed"}:
            return "공구 추적 준비에 실패했습니다."
        if message:
            return self._limit_words(message.splitlines()[0].strip(), 7)
        return ""

    def _build_rejected_message(self, reason, message):
        if reason == "resume_without_paused_task":
            return "재개할 작업이 없습니다. 다음 명령을 기다리겠습니다."
        if reason == "llm_failed":
            return (
                "문장을 끝까지 이해하지 못했습니다. "
                "공구 이름이나 동작을 다시 말해 주세요."
            )
        if reason == "unknown_tool":
            return (
                "어떤 공구인지 잘 모르겠습니다. "
                "드라이버, 플라이어, 망치, 줄자, 렌치 중에서 다시 말해 주세요."
            )
        if reason == "unknown_action":
            return (
                "무엇을 할지 정확하지 않습니다. "
                "가져와, 치워줘, Home으로 같은 표현으로 다시 말해 주세요."
            )
        if reason == "deictic_bring_not_supported":
            return (
                "지금은 '이거 가져와'처럼 대상이 없는 요청은 이해하지 못합니다. "
                "공구 이름을 함께 말해 주세요."
            )
        if reason == "low_confidence":
            return (
                "제가 이해한 내용이 정확하지 않습니다. "
                "공구 이름이나 동작을 다시 말해 주세요."
            )
        return message or "명령을 이해하지 못했습니다. 다시 입력해 주세요."

    @staticmethod
    def _spoken_robot_statuses():
        return {
            'accepted',
            'searching_drawer',
            'moving_to_drawer',
            'searching_drawer_handle',
            'opening_drawer',
            'closing_drawer',
            'searching',
            'picking',
            'approaching_tool',
            'grasping',
            'grasp_success',
            'lifting_tool',
            'moving_to_handoff',
            'searching_hand',
            'waiting_handoff',
            'handoff_complete',
            'waiting_return_handoff',
            'moving_return_grasp_pose',
            'checking_return_target',
            'return_hand_detected',
            'placing_return_tool',
            'returning_home',
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
            'returned',
            'tool_dropped',
            'vlm_loading',
            'vlm_ready',
            'vlm_warning',
            'vlm_error',
        }

    @staticmethod
    def _robot_status_default_message(state):
        return {
            'accepted': '요청을 확인했습니다.',
            'searching_drawer': '공구함을 찾는 중입니다.',
            'moving_to_drawer': '공구함으로 이동 중입니다.',
            'searching_drawer_handle': '서랍 손잡이를 찾는 중입니다.',
            'opening_drawer': '서랍을 여는 중입니다.',
            'closing_drawer': '서랍을 닫는 중입니다.',
            'searching': '대상 공구를 찾는 중입니다.',
            'picking': '공구를 집는 위치로 이동 중입니다.',
            'approaching_tool': '공구 상단으로 접근 중입니다.',
            'grasping': '공구를 잡는 중입니다.',
            'grasp_success': '공구를 안정적으로 잡았습니다.',
            'lifting_tool': '공구를 안전 높이로 들어 올리는 중입니다.',
            'moving_to_handoff': '사용자 전달 위치로 이동 중입니다.',
            'searching_hand': '사용자 손을 찾는 중입니다.',
            'waiting_handoff': '손으로 공구를 잡아주세요.',
            'handoff_complete': '공구 전달을 완료했습니다.',
            'waiting_return_handoff': '반납할 공구를 받을 준비를 하고 있습니다.',
            'moving_return_grasp_pose': '반납 공구 감지 위치로 이동 중입니다.',
            'checking_return_target': '반납 공구 위치를 확인하는 중입니다.',
            'return_hand_detected': '사용자 손 위치에서 반납 공구를 받는 중입니다.',
            'placing_return_tool': '반납 공구를 보관 위치에 놓는 중입니다.',
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
            'rejected': '요청을 수행할 수 없습니다.',
            'returned': '반납 작업을 완료했습니다.',
            'tool_dropped': '공구 drop이 감지되었습니다.',
            'vlm_loading': 'VLM 모델을 로드하는 중입니다.',
            'vlm_ready': 'VLM 모델 준비가 완료되었습니다.',
            'vlm_warning': 'VLM 모델 상태를 확인해야 합니다.',
            'vlm_error': 'VLM 모델 처리 중 오류가 발생했습니다.',
        }.get(state, '')

    @staticmethod
    def _limit_words(text, max_words):
        words = str(text or "").strip().split()
        if not words:
            return ""
        if len(words) <= max_words:
            return " ".join(words)
        return " ".join(words[:max_words])

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
        return "".join(str(text or "").split()).lower()

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


if __name__ == "__main__":
    main()
