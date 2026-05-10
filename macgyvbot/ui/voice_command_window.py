"""PyQt window components for the macgyvbot voice command GUI node."""

from datetime import datetime
from html import escape

try:
    from PyQt5.QtCore import Qt, QTimer
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
    Qt = None
    QTimer = None
    VoiceCommandGuiWindow = None
else:

    class VoiceCommandGuiWindow(QMainWindow):
        def __init__(self, node):
            super().__init__()
            self._node = node
            self.setWindowTitle('MacGyvBot Assistant')
            self.resize(860, 680)
            self.setMinimumSize(680, 540)

            self._chat_view = QTextEdit()
            self._chat_view.setReadOnly(True)
            self._chat_view.setAcceptRichText(True)

            self._input = QLineEdit()
            self._input.setPlaceholderText('메시지를 입력하거나 음성으로 말해주세요.')
            self._input.returnPressed.connect(self._send_text)

            self._send_button = QPushButton('전송  >')
            self._send_button.clicked.connect(self._send_text)

            self._status = QLabel('로봇 연결 상태: 대기 중')
            self._title = QLabel('MacGyvBot Assistant')
            self._subtitle = QLabel('음성 명령 기반 공구 전달 로봇')
            self._avatar = QLabel('M')
            self._avatar.setAlignment(Qt.AlignCenter)

            input_layout = QHBoxLayout()
            input_layout.setContentsMargins(16, 10, 16, 12)
            input_layout.setSpacing(10)
            input_layout.addWidget(self._input)
            input_layout.addWidget(self._send_button)

            header_layout = QVBoxLayout()
            header_layout.setContentsMargins(0, 14, 0, 10)
            header_layout.setSpacing(2)
            header_layout.addWidget(self._avatar)
            header_layout.addWidget(self._title)
            header_layout.addWidget(self._subtitle)
            header_layout.setAlignment(self._avatar, Qt.AlignHCenter)
            header_layout.setAlignment(self._title, Qt.AlignHCenter)
            header_layout.setAlignment(self._subtitle, Qt.AlignHCenter)

            layout = QVBoxLayout()
            layout.setContentsMargins(16, 8, 16, 10)
            layout.setSpacing(0)
            layout.addLayout(header_layout)
            layout.addWidget(self._chat_view)
            layout.addLayout(input_layout)
            layout.addWidget(self._status)

            root = QWidget()
            root.setLayout(layout)
            self.setCentralWidget(root)
            self._apply_style()

            self.append_bot(
                '안녕하세요. 공구 이름을 말하거나 자연어로 요청해주세요.\n'
                '예: 드라이버 가져다줘, 망치 정리해줘'
            )

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
            self._append_bubble('MacGyvBot', text, align='left', role='bot')

        def append_system(self, text):
            self._append_bubble('상태', text, align='left', role='system')

        def append_command_result(self, command):
            self._append_command_card(command)

        def set_status(self, text):
            self._status.setText(f'로봇 연결 상태: {text}')

        def _append_bubble(self, speaker, text, align, role):
            colors = {
                'user': ('#2F6FDC', '#FFFFFF', '#1E5EC8'),
                'bot': ('#FFFFFF', '#1D2733', '#D9E3F0'),
                'system': ('#F2F7FF', '#45617A', '#D8E8FA'),
            }
            background, foreground, border = colors[role]
            width = '64%' if role == 'user' else '72%'
            speaker_html = escape(speaker)
            text_html = escape(str(text)).replace('\n', '<br>')
            time_html = self._timestamp()
            avatar = self._avatar_html(role)
            left_avatar = avatar if role != 'user' else ''
            right_avatar = avatar if role == 'user' else ''
            html = f'''
            <table width="100%" cellspacing="0" cellpadding="0">
              <tr>
                <td align="{align}" style="padding:8px 6px;">
                  {left_avatar}
                  <div style="
                    max-width:{width};
                    display:inline-block;
                    margin:2px 8px;
                    padding:12px 15px;
                    border-radius:16px;
                    background-color:{background};
                    border:1px solid {border};
                    color:{foreground};
                    font-size:15px;
                    line-height:1.55;
                    box-shadow:0 2px 8px rgba(31, 63, 101, 0.08);
                    text-align:left;
                  ">
                    <div style="font-size:12px; color:#7B8EA5; margin-bottom:4px;">
                      {speaker_html}
                    </div>
                    {text_html}
                  </div>
                  {right_avatar}
                  <span style="color:#99A7B8; font-size:11px; vertical-align:bottom;">
                    {time_html}
                  </span>
                </td>
              </tr>
            </table>
            '''
            self._chat_view.append(html)
            self._chat_view.verticalScrollBar().setValue(
                self._chat_view.verticalScrollBar().maximum()
            )

        def _append_command_card(self, command):
            tool_name = escape(str(command.get('tool_name', 'unknown')))
            action = escape(self._action_label(command.get('action', 'unknown')))
            target_mode = escape(self._target_label(command.get('target_mode', 'unknown')))
            method = escape(self._method_label(command.get('match_method', 'unknown')))
            confidence = command.get('confidence', 0.0)
            try:
                confidence_value = max(0.0, min(1.0, float(confidence)))
            except (TypeError, ValueError):
                confidence_value = 0.0
            confidence_percent = int(round(confidence_value * 100))
            time_html = self._timestamp()
            html = f'''
            <table width="100%" cellspacing="0" cellpadding="0">
              <tr>
                <td align="left" style="padding:2px 6px 12px 6px;">
                  {self._avatar_html('bot')}
                  <div style="
                    display:inline-block;
                    width:330px;
                    margin:2px 8px;
                    border:1px solid #D6E2F0;
                    border-radius:16px;
                    background:#FFFFFF;
                    color:#233142;
                    box-shadow:0 4px 12px rgba(31, 63, 101, 0.10);
                    overflow:hidden;
                    vertical-align:top;
                  ">
                    <div style="
                      padding:11px 14px;
                      background:#EEF6FF;
                      color:#2E69B8;
                      font-weight:700;
                      font-size:13px;
                    ">
                      명령 해석 결과
                    </div>
                    <table width="100%" cellspacing="0" cellpadding="0"
                      style="font-size:13px; line-height:1.7; padding:12px 14px;">
                      {self._card_row('공구', tool_name)}
                      {self._card_row('동작', action)}
                      {self._card_row('대상', target_mode)}
                      {self._card_row('방식', method)}
                      <tr>
                        <td style="color:#6D7F93; width:82px; padding:4px 0;">신뢰도</td>
                        <td style="padding:4px 0;">
                          {confidence_percent}%
                          <div style="
                            display:inline-block;
                            width:120px;
                            height:7px;
                            margin-left:8px;
                            background:#E8EEF5;
                            border-radius:4px;
                            vertical-align:middle;
                          ">
                            <div style="
                              width:{confidence_percent}%;
                              height:7px;
                              background:#4FBF7A;
                              border-radius:4px;
                            "></div>
                          </div>
                        </td>
                      </tr>
                    </table>
                  </div>
                  <span style="color:#99A7B8; font-size:11px; vertical-align:bottom;">
                    {time_html}
                  </span>
                </td>
              </tr>
            </table>
            '''
            self._chat_view.append(html)
            self._chat_view.verticalScrollBar().setValue(
                self._chat_view.verticalScrollBar().maximum()
            )

        @staticmethod
        def _card_row(label, value):
            return f'''
              <tr>
                <td style="color:#6D7F93; width:82px; padding:4px 0;">{escape(label)}</td>
                <td style="font-weight:600; padding:4px 0;">{value}</td>
              </tr>
            '''

        @staticmethod
        def _timestamp():
            return datetime.now().strftime('%H:%M:%S')

        @staticmethod
        def _avatar_html(role):
            if role == 'user':
                text = '나'
                background = '#9CB7DC'
                color = '#FFFFFF'
            else:
                text = 'M'
                background = '#EAF3FF'
                color = '#2F6FDC'
            return f'''
              <span style="
                display:inline-block;
                width:34px;
                height:34px;
                line-height:34px;
                border-radius:17px;
                text-align:center;
                background:{background};
                color:{color};
                font-weight:800;
                vertical-align:top;
                border:1px solid #D5E4F5;
              ">{text}</span>
            '''

        @staticmethod
        def _action_label(action):
            return {
                'bring': '가져오기',
                'return': '정리하기',
                'release': '놓기',
                'stop': '정지',
                'unknown': '알 수 없음',
            }.get(str(action), str(action))

        @staticmethod
        def _target_label(target_mode):
            return {
                'named': '직접 지정',
                'deictic': '가리킨 대상',
                'unknown': '알 수 없음',
            }.get(str(target_mode), str(target_mode))

        @staticmethod
        def _method_label(method):
            return {
                'alias': 'Alias',
                'fuzzy': 'Fuzzy',
                'llm': 'LLM',
                'local_deictic': 'Local',
                'unknown': 'Unknown',
            }.get(str(method), str(method))

        def _apply_style(self):
            self.setStyleSheet(
                '''
                QMainWindow {
                    background-color: #F4F8FC;
                }
                QWidget {
                    font-family: "Noto Sans CJK KR", "Noto Sans KR", "Segoe UI", "Arial", sans-serif;
                    color: #1D2733;
                }
                QLabel {
                    color: #163B5C;
                }
                QTextEdit {
                    background-color: #F8FBFF;
                    border: 1px solid #CAD9EA;
                    border-radius: 14px;
                    padding: 14px;
                }
                QLineEdit {
                    background-color: #FFFFFF;
                    border: 1px solid #B7CAE1;
                    border-radius: 12px;
                    padding: 12px 14px;
                    font-size: 15px;
                }
                QPushButton {
                    background-color: #2F6FDC;
                    color: #FFFFFF;
                    border: none;
                    border-radius: 12px;
                    padding: 12px 20px;
                    font-weight: bold;
                    font-size: 14px;
                }
                QPushButton:hover {
                    background-color: #245FC4;
                }
                '''
            )
            self._avatar.setStyleSheet(
                '''
                background-color: #E7F1FF;
                border: 1px solid #C7DBF5;
                color: #2F6FDC;
                border-radius: 18px;
                min-width: 36px;
                max-width: 36px;
                min-height: 36px;
                max-height: 36px;
                font-size: 18px;
                font-weight: 900;
                '''
            )
            self._title.setStyleSheet(
                'font-size: 22px; font-weight: 800; color: #223B5C;'
            )
            self._subtitle.setStyleSheet('font-size: 13px; color: #6A7E96;')
            self._status.setStyleSheet(
                '''
                background-color: #FFFFFF;
                border: 1px solid #D5E2F0;
                border-radius: 10px;
                padding: 8px 12px;
                color: #4FBF7A;
                font-size: 13px;
                font-weight: 600;
                '''
            )
