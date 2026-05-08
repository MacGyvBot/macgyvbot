"""PyQt chat UI for the macgyvbot voice command pipeline."""

import json
import sys
from html import escape

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QPushButton,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover - runtime environment guidance
    QApplication = None


class VoiceCommandGuiWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self._node = node
        self.setWindowTitle('MacGyvBot Assistant')
        self.resize(620, 520)
        self.setMinimumSize(520, 460)

        self._chat_view = QTextEdit()
        self._chat_view.setReadOnly(True)
        self._chat_view.setAcceptRichText(True)

        self._input = QLineEdit()
        self._input.setPlaceholderText('예: 드라이버 가져다줘')
        self._input.returnPressed.connect(self._send_text)

        self._send_button = QPushButton('전송')
        self._send_button.clicked.connect(self._send_text)

        self._status = QLabel('준비 완료')
        self._title = QLabel('MacGyvBot Assistant')
        self._subtitle = QLabel('음성 명령 기반 공구 전달 로봇')

        input_layout = QHBoxLayout()
        input_layout.addWidget(self._input)
        input_layout.addWidget(self._send_button)

        header_layout = QVBoxLayout()
        header_layout.addWidget(self._title)
        header_layout.addWidget(self._subtitle)

        layout = QVBoxLayout()
        layout.addLayout(header_layout)
        layout.addWidget(self._chat_view)
        layout.addLayout(input_layout)
        layout.addWidget(self._status)

        root = QWidget()
        root.setLayout(layout)
        self.setCentralWidget(root)
        self._apply_style()

        self.append_system('입력 상태: 공구 이름을 직접 말하거나 자연어로 요청할 수 있습니다.')
        self.append_system('확인 상태: 확인 질문에는 네 또는 아니오로 답해주세요.')

    def _send_text(self):
        text = self._input.text().strip()
        if not text:
            return

        self._input.clear()
        self.append_user(text)
        self._node.publish_user_text(text)

    def append_user(self, text, source='keyboard'):
        label = '나' if source == 'keyboard' else '음성 입력'
        self._append_bubble(label, text, align='right', role='user')

    def append_bot(self, text):
        self._append_bubble('MacGyvBot', text, align='center', role='bot')

    def append_system(self, text):
        self._append_bubble('상태', text, align='center', role='system')

    def set_status(self, text):
        self._status.setText(text)

    def _append_bubble(self, speaker, text, align, role):
        colors = {
            'user': ('#FFFFFF', '#0F3557'),
            'bot': ('#E7F3FF', '#1D2733'),
            'system': ('#EAF4FF', '#45617A'),
        }
        background, foreground = colors[role]
        width = '72%' if role != 'system' else '84%'
        speaker_html = escape(speaker)
        text_html = escape(str(text))
        html = f'''
        <table width="100%" cellspacing="0" cellpadding="0">
          <tr>
            <td align="{align}">
              <div style="
                max-width:{width};
                margin:6px 4px;
                padding:9px 12px;
                border-radius:12px;
                background-color:{background};
                color:{foreground};
                font-size:14px;
                line-height:1.45;
              ">
                <div style="font-size:11px; color:#5B7590; margin-bottom:3px;">
                  {speaker_html}
                </div>
                {text_html}
              </div>
            </td>
          </tr>
        </table>
        '''
        self._chat_view.append(html)
        self._chat_view.verticalScrollBar().setValue(
            self._chat_view.verticalScrollBar().maximum()
        )

    def _apply_style(self):
        self.setStyleSheet(
            '''
            QMainWindow {
                background-color: #DCEEFF;
            }
            QWidget {
                font-family: "Arial", "Noto Sans CJK KR", sans-serif;
                color: #1D2733;
            }
            QLabel {
                color: #163B5C;
            }
            QTextEdit {
                background-color: #CFE6FA;
                border: 1px solid #A8CBEA;
                border-radius: 10px;
                padding: 8px;
            }
            QLineEdit {
                background-color: #FFFFFF;
                border: 1px solid #9BC4E8;
                border-radius: 8px;
                padding: 10px;
                font-size: 14px;
            }
            QPushButton {
                background-color: #2F80D1;
                color: #FFFFFF;
                border: none;
                border-radius: 8px;
                padding: 10px 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1F6FBF;
            }
            '''
        )
        self._title.setStyleSheet('font-size: 20px; font-weight: bold;')
        self._subtitle.setStyleSheet('font-size: 12px; color: #4D6B85;')
        self._status.setStyleSheet(
            'background-color: #BBDCF7; border-radius: 7px; padding: 7px;'
        )


class VoiceCommandGuiNode(Node):
    def __init__(self):
        super().__init__('voice_command_gui_node')

        if QApplication is None:
            raise RuntimeError(
                'PyQt5가 설치되어 있지 않습니다. '
                'sudo apt install python3-pyqt5 또는 pip install PyQt5 후 '
                '다시 실행하세요.'
            )

        self.declare_parameter('stt_text_topic', '/stt_text')
        self.declare_parameter('tool_command_topic', '/tool_command')
        self.declare_parameter('target_label_topic', '/target_label')
        self.declare_parameter('command_feedback_topic', '/command_feedback')
        self.declare_parameter('robot_status_topic', '/robot_task_status')

        stt_text_topic = self.get_parameter('stt_text_topic').value
        tool_command_topic = self.get_parameter('tool_command_topic').value
        target_label_topic = self.get_parameter('target_label_topic').value
        command_feedback_topic = self.get_parameter(
            'command_feedback_topic'
        ).value
        robot_status_topic = self.get_parameter('robot_status_topic').value

        self.window = None
        self._last_gui_text = ''
        self._last_target_label = ''

        self._text_pub = self.create_publisher(String, stt_text_topic, 10)
        self.create_subscription(String, stt_text_topic, self._stt_text_cb, 10)
        self.create_subscription(
            String,
            tool_command_topic,
            self._tool_command_cb,
            10,
        )
        self.create_subscription(
            String,
            target_label_topic,
            self._target_label_cb,
            10,
        )
        self.create_subscription(
            String,
            command_feedback_topic,
            self._feedback_cb,
            10,
        )
        self.create_subscription(
            String,
            robot_status_topic,
            self._robot_status_cb,
            10,
        )

        self.get_logger().info('MacGyvBot GUI 명령 UI 준비 완료')

    def attach_window(self, window):
        self.window = window

    def publish_user_text(self, text):
        self._last_gui_text = text
        msg = String()
        msg.data = text
        self._text_pub.publish(msg)
        self.get_logger().info(f'GUI 사용자 명령 전송: "{text}"')

    def _stt_text_cb(self, msg):
        text = msg.data.strip()
        if not text:
            return

        if text == self._last_gui_text:
            self._last_gui_text = ''
            return

        self._append_user(text, source='voice')
        self._set_status('음성 명령 수신')

    def _tool_command_cb(self, msg):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'/tool_command JSON 파싱 실패: {msg.data}')
            return

        tool_name = command.get('tool_name', 'unknown')
        action = command.get('action', 'unknown')
        method = command.get('match_method', 'unknown')
        confidence = command.get('confidence', 0.0)

        try:
            confidence_text = f'{float(confidence):.2f}'
        except (TypeError, ValueError):
            confidence_text = str(confidence)

        self._append_system(
            f'parsed: tool={tool_name}, action={action}, '
            f'method={method}, confidence={confidence_text}'
        )

    def _target_label_cb(self, msg):
        self._last_target_label = msg.data
        self._append_bot(f'{msg.data} pick 명령을 로봇에게 보냈습니다.')
        self._set_status(f'실행 요청: {msg.data}')

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
            self._append_bot(message or '명령을 이해하지 못했습니다. 다시 입력해주세요.')
            self._append_system(f'reason={reason}')
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

        if state in ('done', 'completed', 'success'):
            self._append_bot(message or f'{tool_name} 전달 동작이 완료되었습니다.')
            self._set_status('완료')
        elif state in ('failed', 'error'):
            self._append_bot(message or f'{tool_name} 동작 중 문제가 발생했습니다.')
            self._set_status('실패')
        else:
            self._append_system(message or f'{tool_name}: {state}')

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


def main(args=None):
    if QApplication is None:
        raise RuntimeError(
            'PyQt5가 설치되어 있지 않습니다. '
            'sudo apt install python3-pyqt5 또는 pip install PyQt5 후 다시 실행하세요.'
        )

    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = VoiceCommandGuiNode()
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


if __name__ == '__main__':
    main()
