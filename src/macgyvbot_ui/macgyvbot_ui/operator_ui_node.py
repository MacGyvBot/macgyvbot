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
    HAND_GRASP_IMAGE_TOPIC,
    ROBOT_STATUS_TOPIC,
    STT_TEXT_TOPIC,
    TOOL_COMMAND_TOPIC,
)
from macgyvbot_domain.logging import MacGyvbotLogger
from macgyvbot_interfaces.msg import (
    CommandFeedback,
    CommandShutdown,
    CommandText,
    RobotTaskStatus,
    ToolCommand,
)
from macgyvbot_ui.voice_command_window import (
    QApplication,
    QTimer,
    VoiceCommandGuiWindow,
)


class OperatorUiNode(Node):
    def __init__(self):
        super().__init__('operator_ui_node')
        self.service_log = MacGyvbotLogger(super().get_logger(), svc="ui")

        self.declare_parameter('stt_text_topic', STT_TEXT_TOPIC)
        self.declare_parameter('tool_command_topic', TOOL_COMMAND_TOPIC)
        self.declare_parameter('command_feedback_topic', COMMAND_FEEDBACK_TOPIC)
        self.declare_parameter('robot_status_topic', ROBOT_STATUS_TOPIC)
        self.declare_parameter('camera_status_topic', CAMERA_COLOR_TOPIC)
        self.declare_parameter('detector_image_topic', HAND_GRASP_IMAGE_TOPIC)
        self.declare_parameter('connection_check_period_sec', 1.0)
        self.declare_parameter('camera_timeout_sec', 3.0)
        self.declare_parameter('detector_timeout_sec', 3.0)
        self.declare_parameter(
            'robot_node_names',
            'macgyvbot_main_node,macgyvbot',
        )

        if QApplication is None or VoiceCommandGuiWindow is None:
            raise RuntimeError(
                'operator_ui_node ?ㅽ뻾?먮뒗 PyQt5媛 ?꾩슂?⑸땲?? '
                'sudo apt install python3-pyqt5 ?먮뒗 pip install PyQt5 ???ㅼ떆 ?ㅽ뻾?섏꽭??'
            )

        stt_text_topic = self.get_parameter('stt_text_topic').value
        tool_command_topic = self.get_parameter('tool_command_topic').value
        command_feedback_topic = self.get_parameter('command_feedback_topic').value
        robot_status_topic = self.get_parameter('robot_status_topic').value
        camera_status_topic = self.get_parameter('camera_status_topic').value
        detector_image_topic = self.get_parameter('detector_image_topic').value

        self.window = None
        self._shutdown_callback = None
        self._exit_pending = False
        self._last_feedback_key = None
        self._last_feedback_stamp_ns = 0
        self._feedback_dedupe_ns = 2_000_000_000
        self._last_target_label = ''
        self._last_camera_stamp_ns = None
        self._last_detector_stamp_ns = None
        self._last_connection_text = ''
        self._last_robot_status_key = None
        self._last_robot_log_key = None
        self._robot_status_topic = robot_status_topic
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

        self._stt_pub = self.create_publisher(CommandText, stt_text_topic, 10)
        self._command_shutdown_pub = self.create_publisher(
            CommandShutdown,
            COMMAND_SHUTDOWN_TOPIC,
            10,
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
        self.create_subscription(Image, camera_status_topic, self._camera_status_cb, 10)
        self.create_subscription(Image, detector_image_topic, self._detector_image_cb, 10)
        self.create_timer(
            float(self.get_parameter('connection_check_period_sec').value),
            self._update_connection_status,
        )

        self.service_log.bind("legacy").info('operator_ui_node 珥덇린???꾨즺')
        self.service_log.info("node", "done", pipe="startup", msg="operator UI initialized")

    def attach_window(self, window):
        self.window = window
        self._update_connection_status()
        self._append_log('info', 'GUI ?곌껐 ?꾨즺')

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
        self.service_log.bind("legacy").info('/command_shutdown 諛쒗뻾: command_input_node 醫낅즺 ?붿껌')

    def publish_user_text(self, text):
        text = (text or '').strip()
        if not text:
            return

        self._mark_self_published(text)
        msg = CommandText()
        msg.text = text
        msg.source = 'operator_ui'
        self._stt_pub.publish(msg)

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

        self.service_log.bind("legacy").info(f'?몃? ?낅젰 ?섏떊: "{text}"')
        self._append_user(text, source='voice')
        self._set_status('?낅젰 ?섏떊')

    def _tool_command_cb(self, msg):
        command = self._tool_command_payload(msg)

        action = command.get('action', 'unknown')
        tool_name = command.get('tool_name', 'unknown')
        if action == 'pause':
            self._append_log('warn', '?뺤? 紐낅졊??濡쒕큸 ?몃뱶濡??꾨떖?덉뒿?덈떎.')
            return

        if self.window is not None and hasattr(self.window, 'append_command_result'):
            self.window.append_command_result(command)

        self._append_log('info', f'/tool_command ?섏떊: action={action}, tool={tool_name}')

    def _feedback_cb(self, msg):
        feedback = self._feedback_payload(msg)

        if self._is_duplicate_feedback(feedback):
            return

        status = feedback.get('status', 'unknown')
        message = feedback.get('message', '')
        reason = feedback.get('reason', 'unknown')

        if status == 'accepted':
            command = feedback.get('command') or {}
            action = command.get('action')
            if action in {'pause', 'resume', 'cancel', 'exit'}:
                if self.window is not None and hasattr(self.window, 'append_command_result'):
                    self.window.append_command_result(command)

            if action == 'pause':
                stop_message = '?뺤? ?붿껌??濡쒕큸???꾨떖?덉뒿?덈떎.'
                self._append_bot(stop_message)
                self._append_log('warn', stop_message)
                followup_message = '?묒뾽???ш컻?좉퉴?? ?꾨땲硫??대쾲 ?묒뾽??痍⑥냼?좉퉴??'
                self._append_bot(followup_message)
                if self.window is not None and hasattr(self.window, 'append_control_actions'):
                    self.window.append_control_actions(
                        (
                            ('?ш컻', '?ш컻'),
                            ('痍⑥냼', '痍⑥냼'),
                        )
                    )
                self._append_log('info', followup_message)
                self._set_status('?뺤? ?붿껌 ?꾨떖')
                return

            if action == 'resume':
                resume_message = (
                    message
                    or '?ш컻 ?붿껌???댄빐?덉뒿?덈떎. ?쒖뼱 ?명꽣?섏씠???곌껐 ???ъ슜?????덉뒿?덈떎.'
                )
                self._append_bot(resume_message)
                self._append_log('info', '?ш컻 紐낅졊 ?댁꽍 ?꾨즺')
                self._set_status('resume pending')
                return

            if action == 'cancel':
                cancel_message = (
                    message or '?꾩옱 ?묒뾽??痍⑥냼?⑸땲?? ?ㅼ쓬 紐낅졊??湲곕떎由ш쿋?듬땲??'
                )
                self._append_bot(cancel_message)
                self._append_log('warn', '?꾩옱 ?묒뾽 痍⑥냼 ?붿껌 諛쒗뻾: queue? 吏꾪뻾 motion ?뺣━')
                self._set_status('cancel pending')
                return

            if action == 'exit':
                self._exit_pending = True
                exit_message = (
                    message
                    or '醫낅즺 ?붿껌???꾨떖?덉뒿?덈떎. ?묒뾽???뺣━?섍퀬 Home ?꾩튂濡?蹂듦?????醫낅즺?⑸땲??'
                )
                self._append_bot(exit_message)
                self._append_log('info', '醫낅즺 紐낅졊 ?댁꽍 ?꾨즺: 濡쒕큸 ?묒뾽 以묐떒 ?붿껌 諛쒗뻾')
                self._set_status('exit pending')
                return

            accepted_message = message or '紐낅졊???댄빐?덉뒿?덈떎.'
            self._append_bot(accepted_message)
            self._append_log('info', accepted_message)
            self._set_status('紐낅졊 ?댁꽍 ?꾨즺')
            return

        if status == 'pending_confirmation':
            confirmation_message = (
                message or '?쒓? ?댄빐??紐낅졊??留욌굹?? ???먮뒗 ?꾨땲?ㅻ줈 ?듯빐二쇱꽭??'
            )
            if self.window is not None and hasattr(self.window, 'append_confirmation'):
                self.window.append_confirmation(confirmation_message)
            else:
                self._append_bot(confirmation_message)
            self._append_log('info', confirmation_message)
            self._set_status('confirmation pending')
            return

        if status == 'cancelled':
            cancel_message = message or '?뚭쿋?듬땲?? ?ㅽ뻾?섏? ?딄쿋?듬땲??'
            self._append_bot(cancel_message)
            self._append_log('warn', cancel_message)
            self._set_status('紐낅졊 痍⑥냼')
            return

        if status == 'assistant_response':
            response_message = message or '?? ?꾩슂??怨듦뎄媛 ?덉쑝硫?留먰빐二쇱꽭??'
            self._append_bot(response_message)
            self._append_log('info', response_message)
            self._set_status('????묐떟')
            return

        if status == 'rejected':
            rejected_message = self._build_rejected_message(reason, message)
            self._append_bot(rejected_message)
            self._append_log('warn', rejected_message)
            self._set_status('?ъ엯???꾩슂')
            return

        self._append_bot(message or '?곹깭瑜??뺤씤?덉뒿?덈떎.')
        self._append_system(f'status={status}, reason={reason}')

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

        if view['show_log']:
            self._append_log(view['severity'], view['log_message'], ros=False)

        if view['show_chat']:
            self._append_bot(view['chat_message'])

        self._handle_exit_status(status, view['state'])

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

    def _handle_exit_status(self, status, state):
        if not self._exit_pending:
            return
        if str(status.get('action', '')).strip().lower() != 'exit':
            return

        if state in {'done', 'completed', 'success'}:
            self._append_log('info', 'Home 蹂듦? ?꾨즺 ?곹깭 ?뺤씤: operator UI瑜?醫낅즺?⑸땲??')
            self._set_status('醫낅즺')
            self._exit_pending = False
            if QTimer is not None:
                QTimer.singleShot(500, self.request_shutdown)
            else:
                self.request_shutdown()
            return

        if state in {'failed', 'error', 'rejected'}:
            self._append_log('warn', '醫낅즺 泥섎━媛 ?꾨즺?섏? ?딆븘 GUI瑜??좎??⑸땲??')
            self._set_status('醫낅즺 ?ㅽ뙣')
            self._exit_pending = False

    def _camera_status_cb(self, _msg):
        self._last_camera_stamp_ns = self.get_clock().now().nanoseconds

    def _detector_image_cb(self, msg):
        self._last_detector_stamp_ns = self.get_clock().now().nanoseconds
        if self.window is not None and hasattr(self.window, 'set_detector_image'):
            try:
                self.window.set_detector_image(msg)
            except ValueError as exc:
                self.service_log.bind("legacy").warn(f'detector image ?쒖떆 ?ㅽ뙣: {exc}')

    def _update_connection_status(self):
        if self.window is None:
            return

        robot_text = self._robot_connection_text()
        camera_text = self._camera_connection_text()
        detector_text = self._detector_connection_text()
        connection_text = f'{robot_text}|{camera_text}|{detector_text}|connection'

        if connection_text == self._last_connection_text:
            return

        self._last_connection_text = connection_text
        if hasattr(self.window, 'set_connection_status'):
            self.window.set_connection_status(
                robot_text,
                camera_text,
                gui_text='connection',
                detector_text=detector_text,
            )

    def _robot_connection_text(self):
        node_names = set(self.get_node_names())
        robot_node_alive = bool(node_names & self._robot_node_names)
        status_publishers = self.get_publishers_info_by_topic(
            self._robot_status_topic
        )

        if robot_node_alive or status_publishers:
            return 'running'
        return 'unknown'

    def _camera_connection_text(self):
        if self._last_camera_stamp_ns is None:
            return 'unknown'

        elapsed_ns = self.get_clock().now().nanoseconds - self._last_camera_stamp_ns
        if elapsed_ns <= self._camera_timeout_ns:
            return 'connected'
        return 'timeout'

    def _detector_connection_text(self):
        if self._last_detector_stamp_ns is None:
            return 'waiting'

        elapsed_ns = self.get_clock().now().nanoseconds - self._last_detector_stamp_ns
        if elapsed_ns <= self._detector_timeout_ns:
            return 'receiving'
        return 'timeout'

    def _build_rejected_message(self, reason, message):
        if reason == 'llm_failed':
            return (
                '臾몄옣???앷퉴吏 ?댄빐?섏? 紐삵뻽?듬땲?? '
                '?쒕씪?대쾭, ?뚮씪?댁뼱, 留앹튂, 以꾩옄 以??대뼡 怨듦뎄?몄? '
                '議곌툑 ??援ъ껜?곸쑝濡?留먰빐二쇱꽭??'
            )
        if reason == 'unknown_tool':
            return (
                '?대뼡 怨듦뎄?몄? ?뺤떎?섏? ?딆뒿?덈떎. '
                '?쒕씪?대쾭, ?뚮씪?댁뼱, 留앹튂, 以꾩옄 以묒뿉???ㅼ떆 留먰빐二쇱꽭??'
            )
        if reason == 'unknown_action':
            return (
                '臾댁뾿???좎? ?뺤떎?섏? ?딆뒿?덈떎. '
                '媛?몃떎以? ?뺣━?? 硫덉떠泥섎읆 留먰빐二쇱꽭??'
            )
        if reason == 'deictic_bring_not_supported':
            return (
                '媛?몄삤湲?紐낅졊? 怨듦뎄 ?대쫫???꾩슂?⑸땲?? '
                '?대뼡 怨듦뎄瑜?媛?몄삱吏 留먰빐二쇱꽭??'
            )
        if reason == 'low_confidence':
            return (
                '?쒓? ?댄빐???댁슜???뺤떎?섏? ?딆뒿?덈떎. '
                '怨듦뎄 ?대쫫怨??숈옉??議곌툑 ??紐낇솗??留먰빐二쇱꽭??'
            )
        return message or '紐낅졊???댄빐?섏? 紐삵뻽?듬땲?? ?ㅼ떆 ?낅젰?댁＜?몄슂.'

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
        severity = self._robot_status_severity(state)
        log_message = self._robot_log_message(status, state, target_label, message, reason)

        key = (state, str(tool_name), raw_message or message)
        chat_state = state in self._chat_robot_statuses()
        force_show = state in self._always_show_robot_statuses()
        show_chat = chat_state and (force_show or key != self._last_robot_status_key)
        if show_chat:
            self._last_robot_status_key = key

        log_key = (state, str(tool_name), raw_message or message, reason)
        show_log = log_key != self._last_robot_log_key
        if show_log:
            self._last_robot_log_key = log_key

        return {
            'state': state,
            'message': message,
            'chat_message': message,
            'log_message': log_message,
            'panel_status': panel_status,
            'stage_text': stage_text,
            'target_label': target_label,
            'severity': severity,
            'show_chat': show_chat,
            'show_log': show_log,
        }

    def _robot_status_message(self, state, target_label, raw_message, reason):
        if raw_message:
            if state in ('failed', 'error') and reason and reason not in raw_message:
                return f'{raw_message} ?먯씤: {reason}'
            return raw_message

        templates = {
            'accepted': f'?붿껌???뺤씤?덉뒿?덈떎. {target_label} ?묒뾽???쒖옉?좉쾶??',
            'searching_drawer': f'{target_label}瑜?爰쇰궪 怨듦뎄?⑥쓣 李얜뒗 以묒엯?덈떎.',
            'moving_to_drawer': '怨듦뎄?⑥쑝濡??대룞 以묒엯?덈떎.',
            'searching_drawer_handle': '?쒕엻 ?먯옟?대? 李얜뒗 以묒엯?덈떎.',
            'opening_drawer': '?쒕엻???щ뒗 以묒엯?덈떎.',
            'closing_drawer': '?쒕엻 臾몄쓣 ?ル뒗 以묒엯?덈떎.',
            'searching': f'{target_label}瑜?李얜뒗 以묒엯?덈떎.',
            'picking': f'{target_label}瑜?吏묐뒗 ?꾩튂濡??대룞 以묒엯?덈떎.',
            'approaching_tool': f'{target_label} ?곷떒?쇰줈 ?묎렐 以묒엯?덈떎.',
            'grasping': '怨듦뎄瑜??〓뒗 以묒엯?덈떎.',
            'grasp_success': '怨듦뎄瑜??덉젙?곸쑝濡??≪븯?듬땲??',
            'lifting_tool': '怨듦뎄瑜??덉쟾 ?믪씠濡??ㅼ뼱 ?щ━??以묒엯?덈떎.',
            'moving_to_handoff': '?ъ슜???꾨떖 ?꾩튂濡??대룞 以묒엯?덈떎.',
            'searching_hand': '?ъ슜???먯쓣 李얜뒗 以묒엯?덈떎.',
            'waiting_handoff': '?먯쑝濡?怨듦뎄瑜??≪븘二쇱꽭??',
            'handoff_complete': '怨듦뎄 ?꾨떖???꾨즺?덉뒿?덈떎.',
            'waiting_return_handoff': '諛섎궔??怨듦뎄瑜?諛쏆쓣 以鍮꾨? ?섍퀬 ?덉뒿?덈떎.',
            'moving_return_grasp_pose': '諛섎궔 怨듦뎄瑜?媛먯????꾩튂濡??대룞 以묒엯?덈떎.',
            'checking_return_target': '諛섎궔 怨듦뎄 ?꾩튂瑜??뺤씤?섎뒗 以묒엯?덈떎.',
            'return_hand_detected': '?ъ슜?????꾩튂?먯꽌 諛섎궔 怨듦뎄瑜?諛쏅뒗 以묒엯?덈떎.',
            'placing_return_tool': f'{target_label}瑜?蹂닿? ?꾩튂???볥뒗 以묒엯?덈떎.',
            'returning_home': 'Home ?꾩튂濡?蹂듦??섎뒗 以묒엯?덈떎.',
            'done': '?묒뾽???꾨즺?섏뿀?듬땲??',
            'completed': '?묒뾽???꾨즺?섏뿀?듬땲??',
            'success': '?묒뾽???꾨즺?섏뿀?듬땲??',
            'failed': '?묒뾽???ㅽ뙣?덉뒿?덈떎.',
            'error': '?묒뾽 以??ㅻ쪟媛 諛쒖깮?덉뒿?덈떎.',
            'busy': '?대? ?ㅻⅨ ?묒뾽???섑뻾 以묒엯?덈떎.',
            'paused': '濡쒕큸???쇱떆?뺤??섏뿀?듬땲??',
            'resumed': '?묒뾽???ㅼ떆 ?쒖옉?⑸땲??',
            'cancelled': '?묒뾽??痍⑥냼?섏뿀?듬땲??',
            'returned': '諛섎궔 ?묒뾽???꾨즺?덉뒿?덈떎.',
            'rejected': '?붿껌???섑뻾?????놁뒿?덈떎.',
            'tool_dropped': '怨듦뎄 drop??媛먯??섏뿀?듬땲??',
            'vlm_loading': 'VLM grasp 紐⑤뜽??濡쒕뱶?섎뒗 以묒엯?덈떎. ?좎떆留?湲곕떎?ㅼ＜?몄슂.',
            'vlm_ready': 'VLM grasp 紐⑤뜽 以鍮꾧? ?꾨즺?섏뿀?듬땲??',
            'vlm_warning': 'VLM grasp 紐⑤뜽 ?곹깭瑜??뺤씤?댁빞 ?⑸땲??',
            'vlm_error': 'VLM grasp 紐⑤뜽 泥섎━ 以??ㅻ쪟媛 諛쒖깮?덉뒿?덈떎.',
        }
        message = templates.get(state, f'?꾩옱 ?묒뾽 ?곹깭??{state}?낅땲??')
        if state in ('failed', 'error') and reason:
            return f'{message} ?먯씤: {reason}'
        return message

    @staticmethod
    def _robot_panel_status(state, message):
        return OperatorUiNode._status_label(state, message)

    @staticmethod
    def _robot_stage_text(state, message):
        return OperatorUiNode._status_label(state, message)

    @staticmethod
    def _status_label(state, message=None):
        labels = {
            'accepted': 'accepted',
            'searching_drawer': 'searching drawer',
            'moving_to_drawer': 'moving to drawer',
            'searching_drawer_handle': 'searching handle',
            'opening_drawer': 'opening drawer',
            'closing_drawer': 'closing drawer',
            'searching': 'searching tool',
            'picking': 'picking tool',
            'approaching_tool': 'approaching tool',
            'grasping': 'grasping tool',
            'grasp_success': 'grasp success',
            'lifting_tool': 'lifting tool',
            'moving_to_handoff': 'moving to handoff',
            'searching_hand': 'searching hand',
            'waiting_handoff': 'waiting handoff',
            'handoff_complete': 'handoff complete',
            'waiting_return_handoff': 'waiting return handoff',
            'moving_return_grasp_pose': 'moving return grasp pose',
            'checking_return_target': 'checking return target',
            'return_hand_detected': 'return hand detected',
            'placing_return_tool': 'placing return tool',
            'returning_home': 'returning home',
            'done': 'done',
            'completed': 'completed',
            'success': 'success',
            'failed': 'failed',
            'error': 'error',
            'busy': 'busy',
            'paused': 'paused',
            'resumed': 'resumed',
            'cancelled': 'cancelled',
            'returned': 'returned',
            'rejected': 'rejected',
            'tool_dropped': 'tool dropped',
            'vlm_loading': 'vlm loading',
            'vlm_ready': 'vlm ready',
            'vlm_warning': 'vlm warning',
            'vlm_error': 'vlm error',
        }
        return labels.get(state, message or state or 'idle')


    @staticmethod
    def _chat_robot_statuses():
        return {
            'waiting_handoff',
            'waiting_return_handoff',
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
            'tool_dropped',
            'handoff_complete',
            'vlm_loading',
            'vlm_ready',
            'vlm_warning',
            'vlm_error',
        }

    @staticmethod
    def _always_show_robot_statuses():
        return {
            'waiting_handoff',
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
            'vlm_ready',
            'vlm_warning',
            'vlm_error',
        }

    @staticmethod
    def _robot_status_severity(state):
        if state in ('failed', 'error', 'tool_dropped', 'vlm_error'):
            return 'error'
        if state in ('busy', 'paused', 'cancelled', 'rejected', 'vlm_warning'):
            return 'warn'
        return 'info'

    @staticmethod
    def _robot_log_message(status, state, target_label, message, reason):
        base = f'robot_status={state}, target={target_label}, message={message}'
        details = []
        for key in ('action', 'step', 'phase', 'progress', 'source'):
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

        if reason:
            details.append(f'reason={reason}')
        if details:
            return f'{base}, {", ".join(details)}'
        return base

    @staticmethod
    def _tool_display_name(tool_name):
        label = str(tool_name or 'unknown').strip()
        display_names = {
            'screwdriver': '?쒕씪?대쾭',
            'pliers': '?뚮씪?댁뼱',
            'hammer': '留앹튂',
            'tape_measure': '以꾩옄',
            'drill': '?쒕┫',
            'wrench': '?뚯튂',
            'unknown': '怨듦뎄',
        }
        return display_names.get(label, label)

    def _append_user(self, text, source='keyboard'):
        if self.window is not None:
            self.window.append_user(text, source=source)

    def _append_bot(self, text):
        if self.window is not None:
            self.window.append_bot(text)

    def _append_system(self, text):
        if self.window is not None:
            self.window.append_system(text)

    def _append_log(self, level, message, ros=True):
        level = str(level or 'info').lower()
        message = str(message or '').strip()
        if not message:
            return

        if self.window is not None and hasattr(self.window, 'append_task_log'):
            self.window.append_task_log(level, message)

        if not ros:
            return

        logger = self.service_log.bind("legacy")
        if level == 'error':
            self.service_log.error("gui_log", "status", pipe="ui", msg=message)
        elif level in ('warn', 'warning'):
            self.service_log.warn("gui_log", "status", pipe="ui", msg=message)
        else:
            self.service_log.info("gui_log", "status", pipe="ui", msg=message)

    def _set_status(self, text):
        if self.window is not None:
            self.window.set_status(text)

    def _set_task_status(self, target_text, stage_text):
        if self.window is not None and hasattr(self.window, 'set_task_status'):
            self.window.set_task_status(target_text, stage_text)


def main(args=None):
    rclpy.init(args=args)
    node = OperatorUiNode()

    app = QApplication(sys.argv)
    window = VoiceCommandGuiWindow(on_user_text=node.publish_user_text)
    node.attach_window(window)
    window.show()

    shutdown_requested = {'value': False}

    def request_shutdown(_signum=None, _frame=None):
        if shutdown_requested['value']:
            return
        shutdown_requested['value'] = True
        node.publish_command_shutdown()
        node.service_log.info("shutdown", "requested")
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

