"""Operator-facing GUI node for MacGyvBot.

This node owns PyQt widgets and presentation logic only.  It communicates with
the command, task, and perception packages through ROS topics.
"""

import signal
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from macgyvbot_config.topics import (
    CAMERA_COLOR_TOPIC,
    COMMAND_FEEDBACK_TOPIC,
    COMMAND_SHUTDOWN_TOPIC,
    HAND_GRASP_TOPIC,
    HAND_GRASP_IMAGE_TOPIC,
    MANUAL_GRIPPER_SERVICE,
    ROBOT_STATUS_TOPIC,
    STT_TEXT_TOPIC,
    TASK_CONTROL_TOPIC,
    TOOL_DROP_TOPIC,
    TOOL_COMMAND_TOPIC,
)
from macgyvbot_config.ui import (
    UI_CAMERA_TIMEOUT_SEC,
    UI_CONNECTION_CHECK_PERIOD_SEC,
    UI_DETECTOR_TIMEOUT_SEC,
    UI_ROBOT_NODE_NAMES,
)
from macgyvbot_config.structured_logging import format_structured_log
from macgyvbot_interfaces.msg import (
    CommandFeedback,
    CommandShutdown,
    CommandText,
    HumanGraspResult,
    RobotTaskControl,
    RobotTaskStatus,
    ToolCommand,
    ToolDropEvent,
)
from macgyvbot_ui.event_chat import (
    command_feedback_chat,
    normal_robot_status_chat,
    robot_status_chat,
    tool_drop_chat,
)
from macgyvbot_interfaces.srv import SetGripper
from macgyvbot_ui.voice_command_window import (
    QApplication,
    QTimer,
    VoiceCommandGuiWindow,
)

def _format_operator_ros_log(message):
    text = str(message or "").strip()
    if not text or text.startswith("[pkg] ") or text.startswith("pkg="):
        return text
    return format_structured_log(
        svc="ui",
        pipe="operator",
        step="log",
        event="status",
        msg=text,
    )


def _source_to_log_labels(source):
    source_text = str(source or "").strip()
    if not source_text:
        return "ui", "operator"
    source_map = {
        "ui.operator": ("ui", "operator"),
        "ui.control": ("ui", "control"),
        "ui.chat": ("ui", "operator"),
        "ui.connection": ("ui", "system"),
        "command.stt": ("command", "input"),
        "command.parser": ("command", "input"),
        "task.status": ("task", "task"),
        "perception.hand": ("perception", "hand_grasp"),
        "perception.detector": ("perception", "perception"),
        "manipulation.gripper": ("manipulation", "gripper"),
        "camera.rgb": ("ui", "camera"),
    }
    if source_text in source_map:
        return source_map[source_text]
    if "." in source_text:
        pkg, pipe = source_text.split(".", 1)
        return pkg or "ui", pipe or "operator"
    return source_text, "operator"


class _StructuredLoggerAdapter:
    def __init__(self, logger):
        self._logger = logger

    def debug(self, message):
        self._logger.debug(_format_operator_ros_log(message))

    def info(self, message):
        self._logger.info(_format_operator_ros_log(message))

    def warn(self, message):
        self._logger.warn(_format_operator_ros_log(message))

    def warning(self, message):
        self.warn(message)

    def error(self, message):
        self._logger.error(_format_operator_ros_log(message))


class OperatorUiNode(Node):
    _GRIPPER_SAFE_STATES = {
        'idle',
        'ready',
        'done',
        'completed',
        'success',
        'failed',
        'error',
        'paused',
        'cancelled',
        'rejected',
    }
    _GRIPPER_ACTIVE_STATES = {
        'accepted',
        'opening_drawer',
        'observing_drawer',
        'moving_to_drawer',
        'searching_drawer',
        'searching_drawer_handle',
        'observing_pick_target',
        'searching',
        'picking',
        'approaching_tool',
        'grasping',
        'grasp_success',
        'lifting_tool',
        'moving_to_handoff',
        'waiting_handoff',
        'handoff_complete',
        'waiting_return_handoff',
        'moving_return_grasp_pose',
        'checking_return_target',
        'return_hand_detected',
        'placing_return_tool',
        'placing_drawer_tool',
        'returning_home',
        'closing_drawer',
        'resumed',
        'busy',
        'tool_dropped',
        'vlm_loading',
        'vlm_inferencing',
    }
    _CHAT_INPUT_DISABLED_STATES = _GRIPPER_ACTIVE_STATES - {
        'busy',
        'resumed',
    }
    _TASK_FINAL_STATES = {
        'done',
        'completed',
        'success',
        'failed',
        'error',
        'cancelled',
        'rejected',
        'busy',
        'returned',
    }
    _TASK_ACTIONS = {
        'bring',
        'return',
        'home',
        'release',
        'exit',
        'cancel',
    }

    def __init__(self):
        super().__init__('operator_ui_node')

        self.declare_parameter('stt_text_topic', STT_TEXT_TOPIC)
        self.declare_parameter('tool_command_topic', TOOL_COMMAND_TOPIC)
        self.declare_parameter('command_feedback_topic', COMMAND_FEEDBACK_TOPIC)
        self.declare_parameter('robot_status_topic', ROBOT_STATUS_TOPIC)
        self.declare_parameter('camera_status_topic', CAMERA_COLOR_TOPIC)
        self.declare_parameter('detector_image_topic', HAND_GRASP_IMAGE_TOPIC)
        self.declare_parameter('manual_gripper_service', MANUAL_GRIPPER_SERVICE)
        self.declare_parameter(
            'connection_check_period_sec',
            UI_CONNECTION_CHECK_PERIOD_SEC,
        )
        self.declare_parameter('camera_timeout_sec', UI_CAMERA_TIMEOUT_SEC)
        self.declare_parameter('detector_timeout_sec', UI_DETECTOR_TIMEOUT_SEC)
        self.declare_parameter('chat_input_release_grace_sec', 1.5)
        self.declare_parameter(
            'robot_node_names',
            UI_ROBOT_NODE_NAMES,
        )

        if QApplication is None or VoiceCommandGuiWindow is None:
            raise RuntimeError(
                'operator_ui_node 실행에는 PyQt5가 필요합니다. '
                'sudo apt install python3-pyqt5 또는 pip install PyQt5 후 다시 실행하세요.'
            )

        stt_text_topic = self.get_parameter('stt_text_topic').value
        tool_command_topic = self.get_parameter('tool_command_topic').value
        command_feedback_topic = self.get_parameter('command_feedback_topic').value
        robot_status_topic = self.get_parameter('robot_status_topic').value
        camera_status_topic = self.get_parameter('camera_status_topic').value
        detector_image_topic = self.get_parameter('detector_image_topic').value
        manual_gripper_service = self.get_parameter('manual_gripper_service').value

        self.window = None
        self._shutdown_callback = None
        self._exit_pending = False
        self._last_feedback_key = None
        self._last_feedback_stamp_ns = 0
        self._feedback_dedupe_ns = 2_000_000_000
        self._last_target_label = ''
        self._last_camera_stamp_ns = None
        self._last_detector_stamp_ns = None
        self._last_camera_image_key = None
        self._last_detector_image_key = None
        self._last_connection_text = ''
        self._last_robot_status_key = None
        self._last_robot_log_key = None
        self._last_event_chat_key = None
        self._task_chat_command_key = None
        self._shown_task_chat_keys = set()
        self._last_hand_present = None
        self._last_hand_log_key = None
        self._last_tool_drop_key = None
        self._last_robot_state = 'unknown'
        self._task_execution_active = False
        self._task_chat_count = 0
        self._task_chat_limit = 5
        self._manual_gripper_backend_available = False
        self._manual_gripper_request_pending = False
        self._last_gripper_enabled = None
        self._last_gripper_reason = ''
        self._last_chat_input_enabled = None
        self._last_chat_input_reason = ''
        self._chat_input_hold_until_ns = 0
        self._chat_input_release_timer_active = False
        self._robot_status_topic = robot_status_topic
        self._manual_gripper_service = (
            str(manual_gripper_service).strip() or MANUAL_GRIPPER_SERVICE
        )
        self._self_published = {}
        self._self_pub_lock = threading.Lock()
        self._robot_node_names = {
            name.strip()
            for name in str(self.get_parameter('robot_node_names').value).split(',')
            if name.strip()
        }
        self._camera_timeout_ns = int(
            float(self.get_parameter('camera_timeout_sec').value) * 1_000_000_000
        )
        self._detector_timeout_ns = int(
            float(self.get_parameter('detector_timeout_sec').value) * 1_000_000_000
        )
        self._chat_input_release_grace_ns = int(
            float(self.get_parameter('chat_input_release_grace_sec').value)
            * 1_000_000_000
        )

        self._stt_pub = self.create_publisher(CommandText, stt_text_topic, 10)
        self._command_shutdown_pub = self.create_publisher(
            CommandShutdown,
            COMMAND_SHUTDOWN_TOPIC,
            10,
        )
        self._task_control_pub = self.create_publisher(
            RobotTaskControl,
            TASK_CONTROL_TOPIC,
            10,
        )
        self._manual_gripper_client = self.create_client(
            SetGripper,
            self._manual_gripper_service,
        )
        self.create_subscription(CommandText, stt_text_topic, self._text_cb, 10)
        self.create_subscription(
            ToolCommand, tool_command_topic, self._tool_command_cb, 10
        )
        self.create_subscription(
            CommandFeedback, command_feedback_topic, self._feedback_cb, 10
        )
        self.create_subscription(
            RobotTaskStatus, robot_status_topic, self._robot_status_cb, 10
        )
        self.create_subscription(
            HumanGraspResult,
            HAND_GRASP_TOPIC,
            self._hand_grasp_cb,
            10,
        )
        self.create_subscription(
            ToolDropEvent,
            TOOL_DROP_TOPIC,
            self._tool_drop_cb,
            10,
        )
        self.create_subscription(Image, camera_status_topic, self._camera_status_cb, 10)
        self.create_subscription(Image, detector_image_topic, self._detector_image_cb, 10)
        self.create_timer(
            float(self.get_parameter('connection_check_period_sec').value),
            self._update_connection_status,
        )

        self.get_logger().info('operator_ui_node 초기화 완료')

    def attach_window(self, window):
        self.window = window
        self._update_connection_status()
        self._update_manual_gripper_backend_availability()
        self._refresh_gripper_control_state()
        self._refresh_chat_input_state()
        self._append_log(
            'info',
            'GUI 연결 완료',
            source='ui.operator',
            event='GUI_ATTACHED',
        )

    def set_shutdown_callback(self, callback):
        self._shutdown_callback = callback

    def request_shutdown(self):
        if self._shutdown_callback is not None:
            self._shutdown_callback()

    def publish_command_shutdown(self):
        msg = CommandShutdown()
        msg.action = 'shutdown'
        msg.source = 'operator_ui'
        self._command_shutdown_pub.publish(msg)
        self.get_logger().info('/command_shutdown 발행: command_input_node 종료 요청')

    def publish_user_text(self, text):
        text = (text or '').strip()
        if not text:
            return

        self._mark_self_published(text)
        msg = CommandText()
        msg.text = text
        msg.source = 'operator_ui'
        self._stt_pub.publish(msg)

    def publish_control_action(self, action, text=''):
        action = str(action or '').strip()
        if action not in {'pause', 'resume', 'retry', 'cancel'}:
            self.publish_user_text(text)
            return

        label = str(text or '').strip() or action

        msg = RobotTaskControl()
        msg.action = action
        msg.reason = label
        msg.source = 'operator_ui'
        self._task_control_pub.publish(msg)

        if action == 'pause':
            message = '정지 요청을 로봇에 전달했습니다.'
            event = 'CONTROL_PAUSE'
            level = 'warn'
            status = '정지 요청 전달'
        elif action == 'resume':
            message = '재개 요청을 로봇에 전달했습니다.'
            event = 'CONTROL_RESUME'
            level = 'info'
            status = '재개 요청 전달'
        elif action == 'retry':
            message = '사용자 손 인식을 다시 시도합니다.'
            event = 'CONTROL_RETRY'
            level = 'info'
            status = '손 인식 재시도'
        else:
            if label == '복귀':
                message = 'Home으로 복귀합니다.'
                event = 'CONTROL_HANDOFF_FALLBACK'
                level = 'warn'
                status = 'Home 복귀 요청'
            else:
                message = '현재 작업을 취소합니다.'
                event = 'CONTROL_CANCEL'
                level = 'warn'
                status = '작업 취소 요청'

        self._append_bot(message)
        self._append_log(
            level,
            message,
            source='ui.control',
            event=event,
            detail=f'topic={TASK_CONTROL_TOPIC}, action={action}, source=operator_ui',
        )
        if action == 'pause':
            self._append_pause_followup()
        self._set_status(status)

    def _append_pause_followup(self):
        followup_message = '작업을 재개할까요, 아니면 이번 작업을 취소할까요?'
        self._append_bot(followup_message)
        if self.window is not None and hasattr(self.window, 'append_control_actions'):
            self.window.append_control_actions(
                (
                    ('재개', '재개', 'resume'),
                    ('취소', '취소', 'cancel'),
                )
            )
        self._append_log(
            'info',
            followup_message,
            source='ui.chat',
            event='QUICK_ACTIONS',
            detail='actions=resume,cancel',
        )

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

    def _text_cb(self, msg):
        text = (msg.text or '').strip()
        if not text:
            return

        if self._consume_if_self_published(text):
            return

        self.get_logger().info(f'외부 입력 수신: "{text}"')
        self._append_user(text, source='voice')
        self._append_log(
            'info',
            '외부 STT 입력을 수신했습니다.',
            source='command.stt',
            event='STT_TEXT_RECEIVED',
            detail=f'topic={STT_TEXT_TOPIC}, source={msg.source}, text="{text}"',
        )
        self._set_status('입력 수신')

    def _tool_command_cb(self, msg):
        command = self._tool_command_payload(msg)

        action = command.get('action', 'unknown')
        tool_name = command.get('tool_name', 'unknown')
        if action == 'pause':
            self._append_log(
                'warn',
                '정지 명령을 로봇 노드로 전달했습니다.',
                source='command.parser',
                event='TOOL_COMMAND',
                detail=self._command_detail(command),
            )
            return

        if self.window is not None and hasattr(self.window, 'append_command_result'):
            self.window.append_command_result(command)

        self._append_log(
            'info',
            f'/tool_command 수신: action={action}, tool={tool_name}',
            source='command.parser',
            event='TOOL_COMMAND',
            detail=self._command_detail(command),
        )

    def _feedback_cb(self, msg):
        feedback = self._feedback_payload(msg)

        if self._is_duplicate_feedback(feedback):
            return

        status = feedback.get('status', 'unknown')
        message = feedback.get('message', '')
        reason = feedback.get('reason', 'unknown')
        feedback_detail = self._feedback_detail(feedback)

        if status == 'accepted':
            command = feedback.get('command') or {}
            action = command.get('action')
            if action in {'pause', 'resume', 'retry', 'cancel', 'exit'}:
                if self.window is not None and hasattr(self.window, 'append_command_result'):
                    self.window.append_command_result(command)

            if action == 'pause':
                stop_message = '정지 요청을 로봇에 전달했습니다.'
                self._append_bot(stop_message)
                self._append_log(
                    'warn',
                    stop_message,
                    source='command.parser',
                    event='CONTROL_PAUSE',
                    detail=feedback_detail,
                )
                followup_message = '작업을 재개할까요, 아니면 이번 작업을 취소할까요?'
                self._append_bot(followup_message)
                if self.window is not None and hasattr(self.window, 'append_control_actions'):
                    self.window.append_control_actions(
                        (
                            ('재개', '재개', 'resume'),
                            ('취소', '취소', 'cancel'),
                        )
                    )
                self._append_log(
                    'info',
                    followup_message,
                    source='ui.chat',
                    event='QUICK_ACTIONS',
                    detail='actions=resume,cancel',
                )
                self._set_status('정지 요청 전달')
                return

            if action == 'resume':
                resume_message = (
                    message
                    or '재개 요청을 이해했습니다. 제어 인터페이스 연결 후 사용할 수 있습니다.'
                )
                self._append_bot(resume_message)
                self._append_log(
                    'info',
                    '재개 명령 해석 완료',
                    source='command.parser',
                    event='CONTROL_RESUME',
                    detail=feedback_detail,
                )
                self._set_status('재개 대기')
                return

            if action == 'retry':
                retry_message = message or '사용자 손 인식을 다시 시도합니다.'
                self._append_bot(retry_message)
                self._append_log('info', 'handoff inspection 재시도 요청 발행')
                self._set_status('손 인식 재시도')
                return

            if action == 'cancel':
                cancel_message = (
                    message or '현재 작업을 취소합니다. 다음 명령을 기다리겠습니다.'
                )
                self._append_bot(cancel_message)
                self._append_log(
                    'warn',
                    '현재 작업 취소 요청 발행: queue와 진행 motion 정리',
                    source='command.parser',
                    event='CONTROL_CANCEL',
                    detail=feedback_detail,
                )
                self._set_status('작업 취소 처리 중')
                return

            if action == 'exit':
                self._exit_pending = True
                exit_message = (
                    message
                    or '종료 요청을 전달했습니다. 작업을 정리하고 Home 위치로 복귀한 뒤 종료합니다.'
                )
                self._append_bot(exit_message)
                self._append_log(
                    'info',
                    '종료 명령 해석 완료: 로봇 작업 중단 요청 발행',
                    source='command.parser',
                    event='CONTROL_EXIT',
                    detail=feedback_detail,
                )
                self._set_status('종료 처리 중')
                return

            accepted_message = message or '명령을 이해했습니다.'
            self._append_bot(accepted_message)
            self._append_log(
                'info',
                accepted_message,
                source='command.parser',
                event='COMMAND_ACCEPTED',
                detail=feedback_detail,
            )
            self._set_status('명령 해석 완료')
            return

        if status == 'pending_confirmation':
            confirmation_message = (
                message or '제가 이해한 명령이 맞나요? 네 또는 아니오로 답해주세요.'
            )
            if self.window is not None and hasattr(self.window, 'append_confirmation'):
                self.window.append_confirmation(confirmation_message)
            else:
                self._append_bot(confirmation_message)
            self._append_log(
                'info',
                confirmation_message,
                source='command.parser',
                event='PENDING_CONFIRMATION',
                detail=feedback_detail,
            )
            self._set_status('확인 응답 대기')
            return

        if status == 'cancelled':
            cancel_message = message or '알겠습니다. 실행하지 않겠습니다.'
            self._append_bot(cancel_message)
            self._append_log(
                'warn',
                cancel_message,
                source='command.parser',
                event='COMMAND_CANCELLED',
                detail=feedback_detail,
            )
            self._set_status('명령 취소')
            return

        if status == 'assistant_response':
            response_message = message or '네, 필요한 공구가 있으면 말해주세요.'
            self._append_bot(response_message)
            self._append_log(
                'info',
                response_message,
                source='command.parser',
                event='ASSISTANT_RESPONSE',
                detail=feedback_detail,
            )
            self._set_status('대화 응답')
            return

        if status == 'rejected':
            rejected_message = (
                command_feedback_chat(status, reason, message)
                or self._build_rejected_message(reason, message)
            )
            self._append_bot(rejected_message)
            self._append_log(
                'warn',
                rejected_message,
                source='command.stt',
                event='PARSE_FAILED',
                detail=feedback_detail,
            )
            self._set_status('재입력 필요')
            return

        self._append_bot(message or '상태를 확인했습니다.')
        self._append_log(
            'info',
            message or '상태를 확인했습니다.',
            source='command.parser',
            event='COMMAND_FEEDBACK',
            detail=feedback_detail,
        )

    def _is_duplicate_feedback(self, feedback):
        status = feedback.get('status', 'unknown')
        if status not in (
            'accepted',
            'pending_confirmation',
            'cancelled',
            'assistant_response',
            'rejected',
        ):
            return False

        command = feedback.get('command') or {}
        key = (
            status,
            feedback.get('reason', ''),
            feedback.get('message', ''),
            feedback.get('raw_text', ''),
            command.get('action', ''),
            command.get('tool_name', ''),
        )
        now = self.get_clock().now().nanoseconds
        if (
            key == self._last_feedback_key
            and now - self._last_feedback_stamp_ns < self._feedback_dedupe_ns
        ):
            return True

        self._last_feedback_key = key
        self._last_feedback_stamp_ns = now
        return False

    def _robot_status_cb(self, msg):
        status = self._robot_status_payload(msg)

        view = self._build_robot_status_view(status)
        self._last_target_label = view['target_label']
        self._set_status(view['panel_status'])
        self._set_task_status(view['target_label'], view['stage_text'])
        self._update_task_execution_state(status, view['state'])
        self._last_robot_state = view['state']
        self._refresh_gripper_control_state()
        self._refresh_chat_input_state()

        if view['show_log']:
            self._append_log(
                view['severity'],
                view['message'],
                ros=False,
                source='task.status',
                event=self._event_name_for_state(view['state']),
                detail=view['log_message'],
            )

        if view['show_chat']:
            appended = self._append_event_chat(view['state'], view['chat_message'])
            if (
                appended
                and view['state'] == 'handoff_inspection_pending'
                and self.window is not None
                and hasattr(self.window, 'append_control_actions')
            ):
                self.window.append_control_actions(
                    (
                        ('재시도', '재시도', 'retry'),
                        ('복귀', '복귀', 'cancel'),
                    )
                )

        self._handle_exit_status(status, view['state'])

    def _hand_grasp_cb(self, msg):
        hand_present = bool(msg.hand_present)
        payload = self._hand_grasp_payload(msg)
        detail = self._format_detail(payload)
        log_key = (
            hand_present,
            payload.get('state'),
            payload.get('human_grasped_tool'),
            payload.get('hand_pixel'),
        )
        if log_key != self._last_hand_log_key:
            self._last_hand_log_key = log_key
            self._append_log(
                'info' if hand_present else 'warn',
                '손이 인식되었습니다.' if hand_present else '손이 인식되지 않았습니다.',
                source='perception.hand',
                event='HAND_DETECTED' if hand_present else 'HAND_NOT_FOUND',
                detail=detail,
            )

        self._last_hand_present = hand_present

    def _tool_drop_cb(self, msg):
        payload = self._tool_drop_payload(msg)
        key = (
            payload.get('event'),
            payload.get('tool_name'),
            payload.get('reason'),
            payload.get('width_mm'),
        )
        if key == self._last_tool_drop_key:
            return
        self._last_tool_drop_key = key

        event = str(payload.get('event') or 'unknown').strip()
        severity = 'error' if event == 'tool_dropped' else 'warn'
        message = (
            tool_drop_chat(event)
            or f'공구 drop monitor 이벤트를 수신했습니다: {event}'
        )
        self._append_log(
            severity,
            message,
            source='manipulation.gripper',
            event='TOOL_DROPPED' if event == 'tool_dropped' else 'DROP_EVENT',
            detail=self._format_detail(payload),
        )
        if event == 'tool_dropped':
            self._append_event_chat('tool_dropped', message, force=True)

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
            'command': OperatorUiNode._tool_command_payload(msg.command),
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
            'command': OperatorUiNode._tool_command_payload(msg.command),
        }

    @staticmethod
    def _hand_grasp_payload(msg):
        hand_pixel = ''
        if msg.has_hand_pixel:
            hand_pixel = f'({msg.hand_u},{msg.hand_v})'
        return {
            'topic': HAND_GRASP_TOPIC,
            'state': msg.state,
            'hand_present': msg.hand_present,
            'human_grasped_tool': msg.human_grasped_tool,
            'grasp_counter': msg.grasp_counter,
            'grasp_score': f'{float(msg.grasp_score):.3f}',
            'ml_state': msg.ml_stable_state or msg.ml_raw_state,
            'depth_available': msg.depth_available,
            'mask_locked': msg.mask_locked,
            'tool_label': msg.tool_label if msg.has_tool_label else '',
            'tool_confidence': (
                f'{float(msg.tool_confidence):.3f}'
                if msg.has_tool_confidence
                else ''
            ),
            'hand_pixel': hand_pixel,
            'handedness': msg.active_handedness if msg.has_active_handedness else '',
        }

    @staticmethod
    def _tool_drop_payload(msg):
        return {
            'topic': TOOL_DROP_TOPIC,
            'event': msg.event,
            'tool_name': msg.tool_name,
            'action': msg.action,
            'reason': msg.reason,
            'width_mm': (
                f'{float(msg.width_mm):.1f}'
                if msg.has_width_mm
                else ''
            ),
            'error': msg.error,
            'command': OperatorUiNode._tool_command_payload(msg.command),
        }

    def _handle_exit_status(self, status, state):
        if not self._exit_pending:
            return
        if str(status.get('action', '')).strip().lower() != 'exit':
            return

        if state in {'done', 'completed', 'success'}:
            self._append_log('info', 'Home 복귀 완료 상태 확인: operator UI를 종료합니다.')
            self._set_status('종료')
            self._exit_pending = False
            if QTimer is not None:
                QTimer.singleShot(500, self.request_shutdown)
            else:
                self.request_shutdown()
            return

        if state in {'failed', 'error', 'rejected'}:
            self._append_log('warn', '종료 처리가 완료되지 않아 GUI를 유지합니다.')
            self._set_status('종료 실패')
            self._exit_pending = False

    def _camera_status_cb(self, msg):
        self._last_camera_stamp_ns = self.get_clock().now().nanoseconds
        key = (
            int(getattr(msg, 'width', 0)),
            int(getattr(msg, 'height', 0)),
            str(getattr(msg, 'encoding', '')),
        )
        if key != self._last_camera_image_key:
            self._last_camera_image_key = key
            self._append_log(
                'info',
                '카메라 이미지 topic을 수신했습니다.',
                source='camera.rgb',
                event='IMAGE_RECEIVED',
                detail=(
                    f'topic={CAMERA_COLOR_TOPIC}, width={key[0]}, '
                    f'height={key[1]}, encoding={key[2]}'
                ),
            )

    def _detector_image_cb(self, msg):
        self._last_detector_stamp_ns = self.get_clock().now().nanoseconds
        key = (
            int(getattr(msg, 'width', 0)),
            int(getattr(msg, 'height', 0)),
            str(getattr(msg, 'encoding', '')),
        )
        if key != self._last_detector_image_key:
            self._last_detector_image_key = key
            self._append_log(
                'info',
                'Detector annotated image topic을 수신했습니다.',
                source='perception.detector',
                event='IMAGE_RECEIVED',
                detail=(
                    f'topic={HAND_GRASP_IMAGE_TOPIC}, width={key[0]}, '
                    f'height={key[1]}, encoding={key[2]}'
                ),
            )
        if self.window is not None and hasattr(self.window, 'set_detector_image'):
            try:
                self.window.set_detector_image(msg)
            except ValueError as exc:
                self.get_logger().warn(f'detector image 표시 실패: {exc}')

    def _update_connection_status(self):
        if self.window is None:
            return

        self._update_manual_gripper_backend_availability()

        robot_text = self._robot_connection_text()
        camera_text = self._camera_connection_text()
        detector_text = self._detector_connection_text()
        connection_text = f'{robot_text}|{camera_text}|{detector_text}|연결됨'

        if connection_text == self._last_connection_text:
            return

        self._last_connection_text = connection_text
        if hasattr(self.window, 'set_connection_status'):
            self.window.set_connection_status(
                robot_text,
                camera_text,
                gui_text='연결됨',
                detector_text=detector_text,
            )
        self._append_log(
            'info',
            'GUI topic 연결 상태가 갱신되었습니다.',
            source='ui.connection',
            event='CONNECTION_STATUS',
            detail=(
                f'robot={robot_text}, camera={camera_text}, '
                f'detector={detector_text}, gui=연결됨'
            ),
            ros=False,
        )

    def _update_manual_gripper_backend_availability(self):
        try:
            available = self._manual_gripper_client.wait_for_service(
                timeout_sec=0.0
            )
        except Exception as exc:
            available = False
            self.get_logger().warn(f'그리퍼 service 확인 실패: {exc}')

        if available != self._manual_gripper_backend_available:
            self._manual_gripper_backend_available = available
            state_text = '연결됨' if available else '미연결'
            self._append_log(
                'info' if available else 'warn',
                f'수동 그리퍼 service {state_text}: {self._manual_gripper_service}',
                ros=False,
            )
            self._refresh_gripper_control_state()

    def _robot_connection_text(self):
        node_names = set(self.get_node_names())
        robot_node_alive = bool(node_names & self._robot_node_names)
        status_publishers = self.get_publishers_info_by_topic(
            self._robot_status_topic
        )

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

    def _detector_connection_text(self):
        if self._last_detector_stamp_ns is None:
            return '대기 중'

        elapsed_ns = self.get_clock().now().nanoseconds - self._last_detector_stamp_ns
        if elapsed_ns <= self._detector_timeout_ns:
            return '수신 중'
        return '미수신'

    def _build_rejected_message(self, reason, message):
        if reason == 'resume_without_paused_task':
            return '재개할 작업이 없습니다. 다음 명령을 기다리겠습니다.'
        if reason == 'llm_failed':
            return '다시 말씀해주세요.'
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
        return message or '다시 말씀해주세요.'

    def _build_robot_status_view(self, status):
        state = str(status.get('status', status.get('state', 'unknown'))).strip()
        if not state:
            state = 'unknown'
        state = state.lower()

        tool_name = status.get('tool_name') or self._last_target_label or 'unknown'
        target_label = self._tool_display_name(tool_name)
        action = self._status_action(status)
        raw_message = str(status.get('message') or '').strip()
        reason = str(status.get('reason') or '').strip()

        abnormal_message = (
            ''
            if state == 'handoff_inspection_pending'
            else robot_status_chat(state, reason, raw_message)
        )
        normal_chat_message = normal_robot_status_chat(state, action, reason)
        message = (
            abnormal_message
            or self._robot_status_message(state, target_label, raw_message, reason)
        )
        panel_status = self._robot_panel_status(state, message)
        stage_text = self._robot_stage_text(state, message)
        severity = self._robot_status_severity(state)
        log_message = self._robot_log_message(status, state, target_label, message, reason)

        command_key = self._status_command_key(status, action, tool_name)
        if command_key != self._task_chat_command_key:
            self._task_chat_command_key = command_key
            self._shown_task_chat_keys.clear()
            self._task_chat_count = 0

        chat_message = abnormal_message or normal_chat_message
        if not chat_message and state in self._operational_chat_statuses():
            chat_message = message
        key = (command_key, state, chat_message)
        show_chat = bool(chat_message) and key not in self._shown_task_chat_keys
        if (
            show_chat
            and self._task_chat_count >= self._task_chat_limit
            and state not in self._chat_limit_exempt_statuses()
        ):
            show_chat = False
        if show_chat:
            self._shown_task_chat_keys.add(key)
            self._task_chat_count += 1
            self._last_robot_status_key = key

        log_key = (state, str(tool_name), raw_message or message, reason)
        show_log = log_key != self._last_robot_log_key
        if show_log:
            self._last_robot_log_key = log_key

        return {
            'state': state,
            'message': message,
            'chat_message': chat_message,
            'log_message': log_message,
            'panel_status': panel_status,
            'stage_text': stage_text,
            'target_label': target_label,
            'severity': severity,
            'show_chat': show_chat,
            'show_log': show_log,
        }

    @staticmethod
    def _status_action(status):
        action = str(status.get('action') or '').strip().lower()
        if action and action != 'unknown':
            return action
        command = status.get('command') or {}
        if isinstance(command, dict):
            command_action = str(command.get('action') or '').strip().lower()
            if command_action and command_action != 'unknown':
                return command_action
        return action or 'unknown'

    def _update_task_execution_state(self, status, state):
        action = self._status_action(status)
        if state in self._TASK_FINAL_STATES:
            self._task_execution_active = False
            return
        if action in self._TASK_ACTIONS and state in self._GRIPPER_ACTIVE_STATES:
            self._task_execution_active = True
            return
        if action in {'bring', 'return', 'home', 'release', 'exit'}:
            self._task_execution_active = True

    @staticmethod
    def _status_command_key(status, action, tool_name):
        command = status.get('command') or {}
        if isinstance(command, dict):
            raw_text = str(command.get('raw_text') or '').strip()
            command_tool = str(command.get('tool_name') or '').strip()
            command_action = str(command.get('action') or '').strip()
            if raw_text or command_tool or command_action:
                return (
                    command_action or action,
                    command_tool or str(tool_name or ''),
                    raw_text,
                )
        return (action, str(tool_name or ''), '')

    def _robot_status_message(self, state, target_label, raw_message, reason):
        if reason in {'handoff_search_failed', 'hand_target_not_found'}:
            return '손이 인식되지 않았습니다. 움직여서 카메라에 나오게 하세요!'
        if state == 'tool_dropped':
            return '공구를 떨어트렸습니다. inspection하여 다시 찾습니다.'

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
            'waiting_handoff': '손이 인식되었습니다. 손으로 공구를 잡아주세요.',
            'handoff_inspection_pending': (
                '사용자의 손을 인식하지 못했습니다. '
                '다시 인식할까요, 복귀할까요?'
            ),
            'handoff_complete': '공구 전달을 완료했습니다.',
            'waiting_return_handoff': '반납할 공구를 받을 준비를 하고 있습니다.',
            'moving_return_grasp_pose': '반납 공구를 감지할 위치로 이동 중입니다.',
            'checking_return_target': '반납 공구 위치를 확인하는 중입니다.',
            'return_hand_detected': '사용자 손 위치에서 반납 공구를 받는 중입니다.',
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
            'tool_dropped': '공구 drop이 감지되었습니다.',
            'vlm_loading': 'VLM grasp 모델을 로드하는 중입니다. 잠시만 기다려주세요.',
            'vlm_inferencing': 'VLM으로 공구 파지점을 탐색하는 중입니다.',
            'vlm_ready': 'VLM grasp 모델 준비가 완료되었습니다.',
            'vlm_warning': 'VLM grasp 모델 상태를 확인해야 합니다.',
            'vlm_error': 'VLM grasp 모델 처리 중 오류가 발생했습니다.',
        }
        message = templates.get(state, f'현재 작업 상태는 {state}입니다.')
        if state in ('failed', 'error') and reason:
            return f'{message} 원인: {reason}'
        return message

    @staticmethod
    def _robot_panel_status(state, message):
        return {
            'accepted': '요청 확인',
            'searching_drawer': '서랍 탐색 중',
            'moving_to_drawer': '서랍 이동 중',
            'searching_drawer_handle': '손잡이 탐색',
            'opening_drawer': '서랍 열기',
            'closing_drawer': '서랍 닫기',
            'searching': '공구 탐색',
            'picking': '공구 접근',
            'approaching_tool': '공구 접근',
            'grasping': '공구 파지',
            'grasp_success': '파지 성공',
            'lifting_tool': '공구상승',
            'moving_to_handoff': '전달 위치 이동',
            'searching_hand': '손 탐색',
            'waiting_handoff': '전달 대기',
            'handoff_inspection_pending': '재시도 선택 대기',
            'handoff_complete': '전달 완료',
            'waiting_return_handoff': '반납 대기',
            'moving_return_grasp_pose': '반납 위치 이동',
            'checking_return_target': '반납 위치 확인',
            'return_hand_detected': '공구 수령 중',
            'placing_return_tool': '공구 보관',
            'returning_home': '홈 복귀 중',
            'done': '완료',
            'completed': '완료',
            'success': '완료',
            'failed': '실패',
            'error': '오류',
            'busy': '작업 중',
            'paused': '일시정지',
            'resumed': '작업 재개',
            'cancelled': '취소',
            'returned': '반납 완료',
            'rejected': '거절',
            'tool_dropped': '공구낙하',
            'vlm_loading': 'VLM 로드 중',
            'vlm_inferencing': '파지점 탐색',
            'vlm_ready': 'VLM 준비 완료',
            'vlm_warning': 'VLM 경고',
            'vlm_error': 'VLM 오류',
        }.get(state, OperatorUiNode._compact_status_text(message))

    @staticmethod
    def _robot_stage_text(state, message):
        return {
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
            'handoff_inspection_pending': '재인식 또는 복귀 선택 대기',
            'handoff_complete': '공구 전달 완료',
            'waiting_return_handoff': '반납 공구 수령 대기',
            'moving_return_grasp_pose': '반납 공구 감지 위치 이동 중',
            'checking_return_target': '반납 공구 위치 확인 중',
            'return_hand_detected': '손 위치에서 공구 수령 중',
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
            'tool_dropped': '공구 낙하 감지',
            'vlm_loading': 'VLM 모델 로드 중',
            'vlm_inferencing': 'VLM 파지점 탐색 중',
            'vlm_ready': 'VLM 모델 준비 완료',
            'vlm_warning': 'VLM 상태 경고',
            'vlm_error': 'VLM 상태 오류',
        }.get(state, message)

    @staticmethod
    def _chat_robot_statuses():
        return set()

    @staticmethod
    def _operational_chat_statuses():
        return {
            'failed',
            'error',
            'busy',
            'rejected',
            'cancelled',
            'tool_dropped',
            'vlm_loading',
            'vlm_inferencing',
            'vlm_ready',
            'vlm_warning',
            'vlm_error',
        }

    @staticmethod
    def _always_show_robot_statuses():
        return {
            'handoff_inspection_pending',
            'waiting_return_handoff',
            'done',
            'completed',
            'success',
            'busy',
            'paused',
            'resumed',
            'cancelled',
            'rejected',
            'handoff_complete',
            'tool_dropped',
            'vlm_loading',
            'vlm_inferencing',
            'vlm_ready',
            'vlm_warning',
            'vlm_error',
        }

    @staticmethod
    def _chat_limit_exempt_statuses():
        return {
            'handoff_inspection_pending',
            'failed',
            'error',
            'busy',
            'rejected',
            'cancelled',
            'tool_dropped',
            'vlm_warning',
            'vlm_error',
        }

    @staticmethod
    def _robot_status_severity(state):
        if state in ('failed', 'error', 'tool_dropped', 'vlm_error'):
            return 'error'
        if state in (
            'busy',
            'paused',
            'cancelled',
            'rejected',
            'handoff_inspection_pending',
            'vlm_warning',
        ):
            return 'warn'
        return 'info'

    @staticmethod
    def _robot_log_message(status, state, target_label, message, reason):
        task = status.get('task')
        action = status.get('action')
        tool_name = status.get('tool_name')
        base_parts = [
            'topic=/robot_task_status',
            f'status={state}',
        ]
        if task not in (None, ''):
            base_parts.append(f'task={task}')
        if action not in (None, ''):
            base_parts.append(f'action={action}')
        if tool_name not in (None, ''):
            base_parts.append(f'tool={tool_name}')
        base_parts.extend(
            [
                f'target={target_label}',
                f'message="{message}"',
            ]
        )
        base = ', '.join(base_parts)
        details = []
        for key in (
            'step',
            'phase',
            'progress',
            'source',
            'reason',
            'bx',
            'by',
            'bz',
            'vlm_yaw_deg',
            'drawer_id',
            'width_mm',
        ):
            value = status.get(key)
            if value not in (None, ''):
                details.append(f'{key}={value}')

        command = status.get('command')
        if isinstance(command, dict):
            for source_key, label in (
                ('action', 'cmd_action'),
                ('target_mode', 'target_mode'),
                ('match_method', 'method'),
                ('confidence', 'confidence'),
                ('command_id', 'command_id'),
                ('raw_text', 'raw_text'),
            ):
                value = command.get(source_key)
                if value not in (None, ''):
                    details.append(f'{label}={value}')

        if reason and not any(item == f'reason={reason}' for item in details):
            details.append(f'reason={reason}')
        if details:
            return f'{base}, {", ".join(details)}'
        return base

    @staticmethod
    def _tool_display_name(tool_name):
        label = str(tool_name or 'unknown').strip()
        display_names = {
            'screwdriver': '드라이버',
            'pliers': '플라이어',
            'hammer': '망치',
            'tape_measure': '줄자',
            'wrench': '렌치',
            'unknown': '공구',
        }
        return display_names.get(label, label)

    @staticmethod
    def _event_name_for_state(state):
        normalized = str(state or 'unknown').strip().upper()
        if not normalized:
            return 'ROBOT_STATUS'
        return normalized.replace('-', '_')

    @staticmethod
    def _command_detail(command):
        if not isinstance(command, dict):
            return ''
        return OperatorUiNode._format_detail(
            {
                'topic': TOOL_COMMAND_TOPIC,
                'action': command.get('action'),
                'tool': command.get('tool_name'),
                'target_mode': command.get('target_mode'),
                'method': command.get('match_method'),
                'confidence': command.get('confidence'),
                'raw_text': command.get('raw_text'),
            }
        )

    @staticmethod
    def _feedback_detail(feedback):
        command = feedback.get('command') or {}
        return OperatorUiNode._format_detail(
            {
                'topic': COMMAND_FEEDBACK_TOPIC,
                'status': feedback.get('status'),
                'reason': feedback.get('reason'),
                'raw_text': feedback.get('raw_text'),
                'cmd_action': command.get('action'),
                'tool': command.get('tool_name'),
                'method': command.get('match_method'),
                'confidence': command.get('confidence'),
            }
        )

    @staticmethod
    def _format_detail(payload):
        if not isinstance(payload, dict):
            return str(payload or '').strip()

        parts = []
        for key, value in payload.items():
            if value in (None, ''):
                continue
            if isinstance(value, dict):
                nested = OperatorUiNode._format_detail(value)
                if nested:
                    parts.append(f'{key}=({nested})')
                continue
            parts.append(f'{key}={value}')
        return ', '.join(parts)

    def _append_user(self, text, source='keyboard'):
        if self.window is not None:
            self.window.append_user(text, source=source)

    def _append_bot(self, text):
        if self.window is not None:
            self.window.append_bot(text)

    def _append_event_chat(self, event, text, force=False):
        message = str(text or '').strip()
        if not message:
            return False

        key = (str(event or ''), message)
        if not force and key == self._last_event_chat_key:
            return False
        self._last_event_chat_key = key
        self._append_bot(message)
        return True

    def _append_system(self, text):
        if self.window is not None:
            self.window.append_system(text)

    def get_logger(self):
        return _StructuredLoggerAdapter(super().get_logger())

    def _append_log(
        self,
        level,
        message,
        ros=True,
        source='ui.operator',
        event='EVENT',
        detail='',
    ):
        level = str(level or 'info').lower()
        message = str(message or '').strip()
        if not message:
            return

        gui_message = self._format_gui_log_message(
            level,
            message,
            source=source,
            event=event,
            detail=detail,
        )
        if self.window is not None and hasattr(self.window, 'append_task_log'):
            self.window.append_task_log(
                level,
                gui_message,
            )

        if not ros:
            return

        ros_message = _format_operator_ros_log(message)
        logger = self.get_logger()
        if level == 'error':
            logger.error(ros_message)
        elif level in ('warn', 'warning'):
            logger.warn(ros_message)
        else:
            logger.info(ros_message)

    def _set_status(self, text):
        if self.window is not None:
            self.window.set_status(self._compact_status_text(text))

    def _set_task_status(self, target_text, stage_text):
        if self.window is not None and hasattr(self.window, 'set_task_status'):
            self.window.set_task_status(target_text, stage_text)

    @staticmethod
    def _format_gui_log_message(
        level,
        message,
        source='ui.operator',
        event='EVENT',
        detail='',
    ):
        text = str(message or '').strip()
        if not text or text.startswith('[pkg] '):
            return text
        pkg, pipe = _source_to_log_labels(source)
        return format_structured_log(
            pkg=pkg,
            pipe=pipe,
            msg=text,
            level=str(level or 'info').upper(),
            event_name=str(event or 'EVENT').strip().upper() or 'EVENT',
            detail=str(detail or '').strip(),
        )

    @staticmethod
    def _compact_status_text(text):
        raw = str(text or '').strip()
        if not raw:
            return '대기'
        normalized = raw.replace(' ', '').lower()
        mapping = {
            '명령대기': '대기',
            '대기': '대기',
            '로봇동작중': '로봇 동작 중',
            '로봇노드실행대기중입니다': '로봇 노드 대기',
            '입력수신': '입력 받음',
            '정지요청전달': '정지 요청 전달',
            '재개대기': '재개 대기',
            '손인식재시도': '손 인식 재시도',
            '작업취소처리중': '작업 취소 중',
            '종료처리중': '종료 처리 중',
            '명령해석완료': '명령 해석 완료',
            '확인응답대기': '확인 응답 대기',
            '명령취소': '명령취소',
            '대화응답': '응답 완료',
            '재입력필요': '재입력 필요',
            '종료': '종료',
            '종료실패': '종료실패',
            'home복귀': '홈 복귀',
            'home복귀요청': '홈 복귀 요청',
            'home복귀중': '홈 복귀 중',
            'vlm로드중': 'VLM 로드 중',
            'vlm탐색중': '파지점 탐색',
            'vlm준비완료': 'VLM 준비 완료',
            'vlm경고': 'VLM 경고',
            'vlm오류': 'VLM 오류',
        }
        compact = mapping.get(normalized)
        if compact:
            return compact
        koreanized = raw.replace('Home', '홈').replace('VLM', '모델')
        koreanized = ' '.join(koreanized.split())
        if len(koreanized) <= 16:
            return koreanized
        return koreanized[:15] + '…'

    def request_manual_gripper_width(self, width_mm):
        try:
            width_mm = int(width_mm)
        except (TypeError, ValueError):
            width_mm = -1

        self._update_manual_gripper_backend_availability()
        if not self._manual_gripper_backend_available:
            message = '수동 그리퍼 조작 백엔드가 아직 연결되지 않았습니다.'
            self._append_bot(message)
            self._append_log('warn', f'{message} requested_width_mm={width_mm}')
            self._refresh_gripper_control_state()
            return False

        if self._manual_gripper_request_pending:
            message = '이전 그리퍼 명령을 처리 중입니다. 잠시만 기다려주세요.'
            self._append_bot(message)
            self._append_log('warn', f'{message} requested_width_mm={width_mm}')
            self._refresh_gripper_control_state()
            return False

        if not self._gripper_state_is_safe(self._last_robot_state):
            message = '작업 실행 중에는 수동 그리퍼 조작을 사용할 수 없습니다.'
            self._append_bot(message)
            self._append_log(
                'warn',
                f'{message} state={self._last_robot_state}, width_mm={width_mm}',
            )
            self._refresh_gripper_control_state()
            return False

        if width_mm < 0:
            message = '그리퍼 폭 값이 올바르지 않습니다.'
            self._append_bot(message)
            self._append_log('warn', f'{message} width_mm={width_mm}')
            return False

        request = SetGripper.Request()
        request.width_mm = float(width_mm)
        request.source = 'operator_ui'
        self._manual_gripper_request_pending = True
        self._refresh_gripper_control_state()

        try:
            future = self._manual_gripper_client.call_async(request)
        except Exception as exc:
            self._manual_gripper_request_pending = False
            message = '그리퍼 명령 전송에 실패했습니다.'
            self._append_bot(message)
            self._append_log('error', f'{message} error={type(exc).__name__}: {exc}')
            self._refresh_gripper_control_state()
            return False

        future.add_done_callback(self._handle_manual_gripper_response)
        self._append_log('info', f'수동 그리퍼 명령 요청: width_mm={width_mm}')
        return True

    def _handle_manual_gripper_response(self, future):
        self._manual_gripper_request_pending = False
        try:
            response = future.result()
        except Exception as exc:
            message = '그리퍼 명령 응답을 받지 못했습니다.'
            self._append_bot(message)
            self._append_log('error', f'{message} error={type(exc).__name__}: {exc}')
            self._refresh_gripper_control_state()
            return

        response_message = str(getattr(response, 'message', '') or '').strip()
        if bool(getattr(response, 'success', False)):
            applied_width = float(getattr(response, 'applied_width_mm', 0.0))
            message = f'그리퍼를 {applied_width:.0f} mm로 적용합니다.'
            self._append_bot(message)
            self._append_log(
                'info',
                f'그리퍼 명령 완료: width_mm={applied_width:.1f}, '
                f'status={getattr(response, "status", "")}, '
                f'message={response_message}',
            )
        else:
            message = response_message or '그리퍼 명령이 거부되었습니다.'
            self._append_bot(message)
            self._append_log(
                'warn',
                f'그리퍼 명령 거부: status={getattr(response, "status", "")}, '
                f'requested_width_mm={getattr(response, "requested_width_mm", 0.0)}',
            )
        self._refresh_gripper_control_state()

    def _refresh_gripper_control_state(self):
        if self.window is None or not hasattr(self.window, 'set_gripper_control_state'):
            return

        enabled, reason = self._gripper_control_state()
        if (
            enabled == self._last_gripper_enabled
            and reason == self._last_gripper_reason
        ):
            return
        self._last_gripper_enabled = enabled
        self._last_gripper_reason = reason
        self.window.set_gripper_control_state(enabled, reason)

    def _gripper_control_state(self):
        state = str(self._last_robot_state or 'unknown').strip().lower()
        if self._manual_gripper_request_pending:
            return False, '비활성화: 그리퍼 명령 처리 중'
        if not self._manual_gripper_backend_available:
            return False, '비활성화: 안전한 그리퍼 제어 인터페이스 없음'
        if state in self._GRIPPER_ACTIVE_STATES:
            return False, '비활성화: 작업 실행 중'
        return True, '활성화: 수동 조작 가능'

    def _gripper_state_is_safe(self, state):
        normalized = str(state or 'unknown').strip().lower()
        return normalized not in self._GRIPPER_ACTIVE_STATES

    def _refresh_chat_input_state(self):
        if self.window is None or not hasattr(self.window, 'set_chat_input_enabled'):
            return

        enabled, reason = self._chat_input_state()
        if (
            enabled == self._last_chat_input_enabled
            and reason == self._last_chat_input_reason
        ):
            return

        self._last_chat_input_enabled = enabled
        self._last_chat_input_reason = reason
        self.window.set_chat_input_enabled(enabled, reason)

    def _chat_input_state(self):
        state = str(self._last_robot_state or 'unknown').strip().lower()
        if state in {'unknown', 'initializing'}:
            return True, ''
        if state in self._CHAT_INPUT_DISABLED_STATES:
            now_ns = self.get_clock().now().nanoseconds
            self._chat_input_hold_until_ns = max(
                self._chat_input_hold_until_ns,
                now_ns + self._chat_input_release_grace_ns,
            )
            return False, '동작 실행 중... 상태 버튼이나 음성 명령을 사용해주세요.'
        if self._task_execution_active:
            now_ns = self.get_clock().now().nanoseconds
            self._chat_input_hold_until_ns = max(
                self._chat_input_hold_until_ns,
                now_ns + self._chat_input_release_grace_ns,
            )
            return False, '동작 실행 중... 상태 버튼이나 음성 명령을 사용해주세요.'
        if self._chat_input_hold_until_ns > self.get_clock().now().nanoseconds:
            self._schedule_chat_input_release_refresh()
            return False, '동작 정리 중... 잠시만 기다려주세요.'
        return True, ''

    def _schedule_chat_input_release_refresh(self):
        if self._chat_input_release_timer_active:
            return

        now_ns = self.get_clock().now().nanoseconds
        delay_ns = max(0, self._chat_input_hold_until_ns - now_ns)
        delay_ms = max(50, int(delay_ns / 1_000_000))
        self._chat_input_release_timer_active = True

        def refresh():
            self._chat_input_release_timer_active = False
            self._refresh_chat_input_state()

        QTimer.singleShot(delay_ms, refresh)


def main(args=None):
    rclpy.init(args=args)
    node = OperatorUiNode()

    app = QApplication(sys.argv)
    window = VoiceCommandGuiWindow(
        on_user_text=node.publish_user_text,
        on_gripper_width=node.request_manual_gripper_width,
        on_control_action=node.publish_control_action,
    )
    node.attach_window(window)
    window.show()

    shutdown_requested = {'value': False}

    def request_shutdown(_signum=None, _frame=None):
        if shutdown_requested['value']:
            return
        shutdown_requested['value'] = True
        node.publish_command_shutdown()
        node.get_logger().info('종료 신호를 받아 operator_ui_node를 종료합니다.')
        window.close()
        app.quit()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    node.set_shutdown_callback(request_shutdown)
    app.aboutToQuit.connect(request_shutdown)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start(30)

    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        request_shutdown()
        exit_code = 0
    finally:
        timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
