"""PyQt window components for the macgyvbot voice command GUI node."""

from html import escape

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
    QTimer = None
    VoiceCommandGuiWindow = None
else:

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
