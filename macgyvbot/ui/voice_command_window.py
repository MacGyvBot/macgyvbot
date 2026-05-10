"""PyQt window components for the macgyvbot voice command GUI node."""

from datetime import datetime

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QFrame,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QPushButton,
        QScrollArea,
        QSizePolicy,
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
            self.resize(1080, 720)
            self.setMinimumSize(960, 620)

            self._chat_scroll = QScrollArea()
            self._chat_scroll.setWidgetResizable(True)
            self._chat_container = QWidget()
            self._chat_container.setObjectName('chatContainer')
            self._chat_layout = QVBoxLayout()
            self._chat_layout.setContentsMargins(22, 18, 22, 18)
            self._chat_layout.setSpacing(4)
            self._chat_layout.addStretch(1)
            self._chat_container.setLayout(self._chat_layout)
            self._chat_scroll.setWidget(self._chat_container)

            self._input = QLineEdit()
            self._input.setPlaceholderText('메시지를 입력하거나 음성으로 말해주세요.')
            self._input.returnPressed.connect(self._send_text)

            self._send_button = QPushButton('전송  >')
            self._send_button.clicked.connect(self._send_text)

            self._robot_connection_status = QLabel('로봇 노드: 미확인')
            self._camera_connection_status = QLabel('카메라 노드: 미확인')
            self._gui_connection_status = QLabel('GUI 노드: 연결됨')
            self._current_status = QLabel('현재 상태: 명령 대기')
            self._task_target_status = QLabel('작업 대상: 없음')
            self._task_stage_status = QLabel('작업 단계: 대기')
            self._title = QLabel('MacGyvBot Assistant')
            self._subtitle = QLabel('음성 명령 기반 공구 전달 로봇')
            self._avatar = QLabel('M')
            self._avatar.setAlignment(Qt.AlignCenter)

            input_layout = QHBoxLayout()
            input_layout.setContentsMargins(0, 12, 0, 0)
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

            status_title = QLabel('Robot Status')
            status_title.setObjectName('statusPanelTitle')
            status_subtitle = QLabel('실시간 연결 및 작업 상태')
            status_subtitle.setObjectName('statusPanelSubtitle')

            status_panel = QFrame()
            status_panel.setObjectName('statusPanel')
            status_panel.setFixedWidth(250)
            status_panel_layout = QVBoxLayout()
            status_panel_layout.setContentsMargins(16, 16, 16, 16)
            status_panel_layout.setSpacing(10)
            status_panel.setLayout(status_panel_layout)
            status_panel_layout.addWidget(status_title)
            status_panel_layout.addWidget(status_subtitle)
            status_panel_layout.addSpacing(8)
            status_panel_layout.addWidget(self._robot_connection_status)
            status_panel_layout.addWidget(self._camera_connection_status)
            status_panel_layout.addWidget(self._gui_connection_status)
            status_panel_layout.addSpacing(12)
            status_panel_layout.addWidget(self._current_status)
            status_panel_layout.addWidget(self._task_target_status)
            status_panel_layout.addWidget(self._task_stage_status)
            status_panel_layout.addStretch(1)

            chat_panel = QWidget()
            chat_panel_layout = QVBoxLayout()
            chat_panel_layout.setContentsMargins(0, 0, 0, 0)
            chat_panel_layout.setSpacing(0)
            chat_panel.setLayout(chat_panel_layout)
            chat_panel_layout.addWidget(self._chat_scroll)
            chat_panel_layout.addLayout(input_layout)

            content_layout = QHBoxLayout()
            content_layout.setContentsMargins(0, 0, 0, 0)
            content_layout.setSpacing(14)
            content_layout.addWidget(status_panel)
            content_layout.addWidget(chat_panel, 1)

            layout = QVBoxLayout()
            layout.setContentsMargins(16, 8, 16, 10)
            layout.setSpacing(10)
            layout.addLayout(header_layout)
            layout.addLayout(content_layout, 1)

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

        def append_confirmation(self, text):
            self.append_bot(text)
            self._append_confirmation_actions()

        def append_command_result(self, command):
            self._append_command_card(command)

        def set_status(self, text):
            self._current_status.setText(f'현재 상태: {text}')

        def set_connection_status(self, robot_text, camera_text, gui_text='연결됨'):
            self._robot_connection_status.setText(f'로봇 노드: {robot_text}')
            self._camera_connection_status.setText(f'카메라 노드: {camera_text}')
            self._gui_connection_status.setText(f'GUI 노드: {gui_text}')
            self._robot_connection_status.setStyleSheet(
                self._connection_style(robot_text)
            )
            self._camera_connection_status.setStyleSheet(
                self._connection_style(camera_text)
            )
            self._gui_connection_status.setStyleSheet(
                self._connection_style(gui_text)
            )

        def set_task_status(self, target_text, stage_text):
            self._task_target_status.setText(f'작업 대상: {target_text}')
            self._task_stage_status.setText(f'작업 단계: {stage_text}')

        def _append_bubble(self, speaker, text, align, role):
            colors = {
                'user': ('#2F6FDC', '#FFFFFF', '#1F58B8', '#17406F'),
                'bot': ('#FFFFFF', '#1D2733', '#C8D8EA', '#2F6FDC'),
                'system': ('#EAF3FF', '#36566F', '#BDD6F2', '#5B7FA8'),
            }
            background, foreground, border, speaker_color = colors[role]

            row = QWidget()
            row_layout = QVBoxLayout()
            row_layout.setContentsMargins(0, 4, 0, 8)
            row_layout.setSpacing(2)
            row.setLayout(row_layout)

            speaker_label = QLabel(speaker)
            speaker_label.setStyleSheet(
                f'color: {speaker_color}; font-size: 12px; font-weight: 700;'
            )
            speaker_label.setAlignment(
                Qt.AlignRight if role == 'user' else Qt.AlignLeft
            )
            row_layout.addWidget(speaker_label)

            line = QHBoxLayout()
            line.setContentsMargins(0, 0, 0, 0)
            line.setSpacing(8)

            bubble = QLabel(str(text))
            bubble.setWordWrap(True)
            bubble.setTextInteractionFlags(Qt.TextSelectableByMouse)
            bubble.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)
            max_width = self._message_max_width()
            bubble.setMaximumWidth(max_width)
            bubble.setMinimumWidth(self._message_min_width(max_width, role))
            bubble.setStyleSheet(
                f'''
                QLabel {{
                    background-color: {background};
                    color: {foreground};
                    border: 1px solid {border};
                    border-radius: 16px;
                    padding: 11px 14px;
                    font-size: 15px;
                    line-height: 1.45;
                }}
                '''
            )

            time_label = QLabel(self._timestamp())
            time_label.setStyleSheet('color: #8394A8; font-size: 11px;')
            time_label.setAlignment(Qt.AlignBottom)

            if role == 'user':
                line.addStretch(1)
                line.addWidget(time_label, 0, Qt.AlignBottom)
                line.addWidget(bubble, 0, Qt.AlignRight)
            else:
                line.addWidget(bubble, 0, Qt.AlignLeft)
                line.addWidget(time_label, 0, Qt.AlignBottom)
                line.addStretch(1)

            row_layout.addLayout(line)
            self._add_chat_widget(row)

        def _append_confirmation_actions(self):
            row = QWidget()
            row_layout = QHBoxLayout()
            row_layout.setContentsMargins(0, 0, 0, 10)
            row_layout.setSpacing(8)
            row.setLayout(row_layout)

            yes_button = QPushButton('예, 맞아요')
            no_button = QPushButton('아니요')
            for button in (yes_button, no_button):
                button.setCursor(Qt.PointingHandCursor)
                button.setStyleSheet(
                    '''
                    QPushButton {
                        background-color: #FFFFFF;
                        color: #2F6FDC;
                        border: 1px solid #BFD4EE;
                        border-radius: 12px;
                        padding: 8px 14px;
                        font-size: 13px;
                        font-weight: 700;
                    }
                    QPushButton:hover {
                        background-color: #EEF6FF;
                    }
                    '''
                )

            yes_button.clicked.connect(lambda: self._send_quick_reply('네'))
            no_button.clicked.connect(lambda: self._send_quick_reply('아니오'))

            row_layout.addWidget(yes_button, 0, Qt.AlignLeft)
            row_layout.addWidget(no_button, 0, Qt.AlignLeft)
            row_layout.addStretch(1)
            self._add_chat_widget(row)

        def _send_quick_reply(self, text):
            self.append_user(text)
            self._node.publish_user_text(text)

        def _add_chat_widget(self, widget):
            self._chat_layout.insertWidget(self._chat_layout.count() - 1, widget)
            QTimer.singleShot(0, self._scroll_to_bottom)
            QTimer.singleShot(80, self._scroll_to_bottom)

        def _scroll_to_bottom(self):
            scrollbar = self._chat_scroll.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())

        def _append_command_card(self, command):
            tool_name = str(command.get('tool_name', 'unknown'))
            action = self._action_label(command.get('action', 'unknown'))
            target_mode = self._target_label(command.get('target_mode', 'unknown'))
            method = self._method_label(command.get('match_method', 'unknown'))
            confidence = command.get('confidence', 0.0)
            try:
                confidence_value = max(0.0, min(1.0, float(confidence)))
            except (TypeError, ValueError):
                confidence_value = 0.0
            confidence_percent = int(round(confidence_value * 100))

            row = QWidget()
            row_layout = QVBoxLayout()
            row_layout.setContentsMargins(0, 4, 0, 10)
            row_layout.setSpacing(2)
            row.setLayout(row_layout)

            speaker_label = QLabel('MacGyvBot')
            speaker_label.setStyleSheet(
                'color: #2F6FDC; font-size: 12px; font-weight: 700;'
            )
            row_layout.addWidget(speaker_label)

            line = QHBoxLayout()
            line.setContentsMargins(0, 0, 0, 0)
            line.setSpacing(8)

            card = QFrame()
            card.setObjectName('commandCard')
            card.setMaximumWidth(380)
            card.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)
            card.setStyleSheet(
                '''
                QFrame#commandCard {
                    background-color: #FFFFFF;
                    border: 1px solid #C8D8EA;
                    border-radius: 16px;
                }
                '''
            )
            card_layout = QVBoxLayout()
            card_layout.setContentsMargins(0, 0, 0, 0)
            card_layout.setSpacing(0)
            card.setLayout(card_layout)

            header = QLabel('명령 해석 결과')
            header.setStyleSheet(
                '''
                QLabel {
                    background-color: #EEF6FF;
                    color: #2E69B8;
                    border-top-left-radius: 15px;
                    border-top-right-radius: 15px;
                    padding: 10px 14px;
                    font-size: 13px;
                    font-weight: 800;
                }
                '''
            )
            card_layout.addWidget(header)

            body = QWidget()
            body.setObjectName('commandCardBody')
            body_layout = QVBoxLayout()
            body_layout.setContentsMargins(14, 12, 14, 12)
            body_layout.setSpacing(8)
            body.setLayout(body_layout)
            body_layout.addLayout(self._card_row_widget('공구', tool_name))
            body_layout.addLayout(self._card_row_widget('동작', action))
            body_layout.addLayout(self._card_row_widget('대상', target_mode))
            body_layout.addLayout(self._card_row_widget('방식', method))
            body_layout.addLayout(
                self._confidence_row_widget(confidence_percent, confidence_value)
            )
            card_layout.addWidget(body)

            time_label = QLabel(self._timestamp())
            time_label.setStyleSheet('color: #8394A8; font-size: 11px;')
            time_label.setAlignment(Qt.AlignBottom)

            line.addWidget(card, 0, Qt.AlignLeft)
            line.addWidget(time_label, 0, Qt.AlignBottom)
            line.addStretch(1)
            row_layout.addLayout(line)
            self._add_chat_widget(row)

        @staticmethod
        def _card_row_widget(label, value):
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(14)

            label_widget = QLabel(label)
            label_widget.setFixedWidth(72)
            label_widget.setStyleSheet('color: #6D7F93; font-size: 13px;')

            value_widget = QLabel(str(value))
            value_widget.setStyleSheet(
                'color: #1D2733; font-size: 13px; font-weight: 700;'
            )
            value_widget.setWordWrap(True)

            row.addWidget(label_widget)
            row.addWidget(value_widget, 1)
            return row

        def _confidence_row_widget(self, confidence_percent, confidence_value):
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(14)

            label_widget = QLabel('신뢰도')
            label_widget.setFixedWidth(72)
            label_widget.setStyleSheet('color: #6D7F93; font-size: 13px;')

            value_layout = QHBoxLayout()
            value_layout.setContentsMargins(0, 0, 0, 0)
            value_layout.setSpacing(8)

            percent_label = QLabel(f'{confidence_percent}%')
            percent_label.setStyleSheet(
                'color: #1D2733; font-size: 13px; font-weight: 700;'
            )

            bar = QFrame()
            bar.setObjectName('confidenceBar')
            bar.setFixedSize(120, 8)
            bar.setStyleSheet(
                '''
                QFrame#confidenceBar {
                    background-color: #E8EEF5;
                    border: none;
                    border-radius: 4px;
                }
                '''
            )
            fill = QFrame(bar)
            fill.setObjectName('confidenceFill')
            fill.setGeometry(0, 0, int(120 * confidence_value), 8)
            fill.setStyleSheet(
                '''
                QFrame#confidenceFill {
                    background-color: #4FBF7A;
                    border: none;
                    border-radius: 4px;
                }
                '''
            )

            value_layout.addWidget(percent_label)
            value_layout.addWidget(bar)
            value_layout.addStretch(1)

            row.addWidget(label_widget)
            row.addLayout(value_layout, 1)
            return row

        @staticmethod
        def _timestamp():
            return datetime.now().strftime('%H:%M')

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
                'deictic': '건네받은 공구',
                'unknown': '알 수 없음',
            }.get(str(target_mode), str(target_mode))

        def _message_max_width(self):
            viewport_width = self._chat_scroll.viewport().width()
            return max(380, min(620, int(viewport_width * 0.50)))

        @staticmethod
        def _message_min_width(max_width, role):
            preferred_width = 320 if role == 'user' else 360
            return min(max_width, preferred_width)

        @staticmethod
        def _connection_style(status_text):
            connected = status_text in ('연결됨', '실행 중')
            color = '#25A55F' if connected else '#7A8794'
            border = '#CFEBDD' if connected else '#D5E2F0'
            background = '#F5FFF9' if connected else '#FFFFFF'
            return f'''
                background-color: {background};
                border: 1px solid {border};
                border-radius: 10px;
                padding: 8px 12px;
                color: {color};
                font-size: 13px;
                font-weight: 700;
            '''

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
                QScrollArea {
                    background-color: #E7F1FA;
                    border: 1px solid #B7CEE7;
                    border-radius: 14px;
                }
                QWidget#chatContainer {
                    background-color: #E7F1FA;
                }
                QFrame#statusPanel {
                    background-color: #F8FBFF;
                    border: 1px solid #C9DAEC;
                    border-radius: 16px;
                }
                QLabel#statusPanelTitle {
                    color: #223B5C;
                    font-size: 17px;
                    font-weight: 900;
                }
                QLabel#statusPanelSubtitle {
                    color: #6A7E96;
                    font-size: 12px;
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
            self._robot_connection_status.setStyleSheet(
                self._connection_style('미확인')
            )
            self._camera_connection_status.setStyleSheet(
                self._connection_style('미확인')
            )
            self._gui_connection_status.setStyleSheet(
                self._connection_style('연결됨')
            )
            self._current_status.setStyleSheet(
                '''
                background-color: #FFFFFF;
                border: 1px solid #D5E2F0;
                border-radius: 10px;
                padding: 8px 12px;
                color: #49677F;
                font-size: 13px;
                font-weight: 600;
                '''
            )
            task_style = '''
                background-color: #FFFFFF;
                border: 1px solid #D5E2F0;
                border-radius: 10px;
                padding: 8px 12px;
                color: #49677F;
                font-size: 13px;
                font-weight: 600;
            '''
            self._task_target_status.setStyleSheet(task_style)
            self._task_stage_status.setStyleSheet(task_style)
