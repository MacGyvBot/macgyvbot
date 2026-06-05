"""PyQt window components for the macgyvbot voice command GUI node."""

from datetime import datetime

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtGui import QImage, QPixmap
    from PyQt5.QtWidgets import (
        QApplication,
        QFrame,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QPushButton,
        QScrollArea,
        QSizePolicy,
        QSlider,
        QSpinBox,
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
        def __init__(self, on_user_text=None, on_gripper_width=None):
            super().__init__()
            self._on_user_text = on_user_text
            self._on_gripper_width = on_gripper_width
            self.setWindowTitle('MacGyvBot Assistant')
            self.setFixedSize(1420, 900)
            self._detector_pixmap = None

            self._chat_scroll = QScrollArea()
            self._chat_scroll.setObjectName('chatScroll')
            self._chat_scroll.setWidgetResizable(True)
            self._chat_container = QWidget()
            self._chat_container.setObjectName('chatContainer')
            self._chat_layout = QVBoxLayout()
            self._chat_layout.setContentsMargins(22, 18, 22, 18)
            self._chat_layout.setSpacing(4)
            self._chat_layout.addStretch(1)
            self._chat_container.setLayout(self._chat_layout)
            self._chat_scroll.setWidget(self._chat_container)
            self._chat_scroll.viewport().setStyleSheet(
                'background-color: #E7F1FA; border-radius: 16px;'
            )

            self._input = QLineEdit()
            self._input.setPlaceholderText('메시지를 입력하거나 음성으로 말해주세요.')
            self._input.returnPressed.connect(self._send_text)

            self._send_button = QPushButton('전송  >')
            self._send_button.clicked.connect(self._send_text)

            self._robot_connection_status = QLabel('로봇 노드: 미확인')
            self._camera_connection_status = QLabel('카메라 노드: 미확인')
            self._detector_connection_status = QLabel('Detector 영상: 대기 중')
            self._gui_connection_status = QLabel('GUI 노드: 연결됨')
            self._current_status = QLabel('현재 상태: 명령 대기')
            self._task_target_status = QLabel('작업 대상: 없음')
            self._task_stage_status = QLabel('작업 단계: 대기')
            self._home_button = QPushButton('복귀')
            self._home_button.setObjectName('homeControlButton')
            self._home_button.clicked.connect(
                lambda _checked=False: self._send_control_text('홈위치로 가')
            )
            self._exit_button = QPushButton('종료')
            self._exit_button.setObjectName('exitControlButton')
            self._exit_button.clicked.connect(
                lambda _checked=False: self._send_control_text('종료')
            )
            control_button_layout = QHBoxLayout()
            control_button_layout.setContentsMargins(0, 0, 0, 0)
            control_button_layout.setSpacing(8)
            control_button_layout.addWidget(self._home_button)
            control_button_layout.addWidget(self._exit_button)
            self._task_log_entries = []
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

            status_panel = QFrame()
            status_panel.setObjectName('statusPanel')
            status_panel.setFixedWidth(290)
            status_panel.setMinimumHeight(455)
            status_panel.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
            status_panel_layout = QVBoxLayout()
            status_panel_layout.setContentsMargins(16, 16, 16, 16)
            status_panel_layout.setSpacing(10)
            status_panel.setLayout(status_panel_layout)
            status_panel_layout.addWidget(status_title)
            status_panel_layout.addSpacing(8)
            status_panel_layout.addWidget(self._robot_connection_status)
            status_panel_layout.addWidget(self._camera_connection_status)
            status_panel_layout.addWidget(self._detector_connection_status)
            status_panel_layout.addWidget(self._gui_connection_status)
            status_panel_layout.addSpacing(12)
            status_panel_layout.addWidget(self._current_status)
            status_panel_layout.addWidget(self._task_target_status)
            status_panel_layout.addWidget(self._task_stage_status)
            status_panel_layout.addStretch(1)
            status_panel_layout.addLayout(control_button_layout)

            gripper_title = QLabel('Gripper Control')
            gripper_title.setObjectName('gripperPanelTitle')
            self._gripper_status = QLabel('비활성화: 안전한 제어 인터페이스 없음')
            self._gripper_status.setObjectName('gripperStatus')
            self._gripper_value = QLabel('폭: 80 mm')
            self._gripper_value.setObjectName('gripperValue')

            self._gripper_slider = QSlider(Qt.Horizontal)
            self._gripper_slider.setObjectName('gripperSlider')
            self._gripper_slider.setRange(0, 80)
            self._gripper_slider.setValue(80)
            self._gripper_slider.setSingleStep(1)
            self._gripper_slider.setPageStep(5)
            self._gripper_slider.setTracking(False)
            self._gripper_slider.valueChanged.connect(self._sync_gripper_from_slider)

            self._gripper_width_input = QSpinBox()
            self._gripper_width_input.setObjectName('gripperWidthInput')
            self._gripper_width_input.setRange(
                self._gripper_slider.minimum(),
                self._gripper_slider.maximum(),
            )
            self._gripper_width_input.setButtonSymbols(QSpinBox.NoButtons)
            self._gripper_width_input.setSuffix(' mm')
            self._gripper_width_input.setValue(self._gripper_slider.value())
            self._gripper_width_input.valueChanged.connect(
                self._sync_gripper_from_input
            )

            gripper_width_layout = QHBoxLayout()
            gripper_width_layout.setContentsMargins(0, 0, 0, 0)
            gripper_width_layout.setSpacing(8)
            gripper_width_layout.addWidget(self._gripper_value)
            gripper_width_layout.addWidget(self._gripper_width_input)

            gripper_scale_layout = QHBoxLayout()
            gripper_scale_layout.setContentsMargins(0, 0, 0, 0)
            gripper_scale_layout.setSpacing(0)
            gripper_scale_layout.addWidget(QLabel('닫힘'))
            gripper_scale_layout.addStretch(1)
            gripper_scale_layout.addWidget(QLabel('열림'))

            self._gripper_apply_button = QPushButton('적용')
            self._gripper_apply_button.setObjectName('gripperApplyButton')
            self._gripper_apply_button.clicked.connect(
                self._request_gripper_width_from_slider
            )

            gripper_panel = QFrame()
            gripper_panel.setObjectName('gripperPanel')
            gripper_panel.setFixedWidth(290)
            gripper_panel.setMinimumHeight(210)
            gripper_panel.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
            gripper_panel_layout = QVBoxLayout()
            gripper_panel_layout.setContentsMargins(16, 14, 16, 14)
            gripper_panel_layout.setSpacing(8)
            gripper_panel.setLayout(gripper_panel_layout)
            gripper_panel_layout.addWidget(gripper_title)
            gripper_panel_layout.addWidget(self._gripper_status)
            gripper_panel_layout.addLayout(gripper_width_layout)
            gripper_panel_layout.addWidget(self._gripper_slider)
            gripper_panel_layout.addLayout(gripper_scale_layout)
            gripper_panel_layout.addWidget(self._gripper_apply_button)
            self._set_gripper_controls_enabled(False)

            chat_panel = QWidget()
            chat_panel_layout = QVBoxLayout()
            chat_panel_layout.setContentsMargins(0, 0, 0, 0)
            chat_panel_layout.setSpacing(0)
            chat_panel.setLayout(chat_panel_layout)
            chat_panel_layout.addWidget(self._chat_scroll)
            chat_panel_layout.addLayout(input_layout)

            detector_title = QLabel('Detector View')
            detector_title.setObjectName('detectorPanelTitle')

            detector_panel = QFrame()
            detector_panel.setObjectName('detectorPanel')
            detector_panel.setFixedWidth(540)
            detector_panel.setMinimumHeight(455)
            detector_panel_layout = QVBoxLayout()
            detector_panel_layout.setContentsMargins(14, 14, 14, 10)
            detector_panel_layout.setSpacing(8)
            detector_panel.setLayout(detector_panel_layout)

            self._detector_image = QLabel('Detector 영상 대기 중')
            self._detector_image.setObjectName('detectorImage')
            self._detector_image.setAlignment(Qt.AlignCenter)
            self._detector_image.setFixedSize(512, 384)
            self._detector_image.setSizePolicy(
                QSizePolicy.Fixed,
                QSizePolicy.Fixed,
            )
            self._detector_status = QLabel('Detector 상태: 대기 중')
            self._detector_status.setObjectName('detectorStatus')

            detector_panel_layout.addWidget(detector_title)
            detector_panel_layout.addWidget(self._detector_image, 0, Qt.AlignTop)
            detector_panel_layout.addSpacing(8)
            detector_panel_layout.addWidget(self._detector_status)
            detector_panel_layout.addStretch(1)

            log_title = QLabel('Task Log')
            log_title.setObjectName('taskLogTitle')
            self._task_log = QLabel('로그 대기 중')
            self._task_log.setObjectName('taskLog')
            self._task_log.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self._task_log.setAlignment(Qt.AlignTop | Qt.AlignLeft)
            self._task_log.setWordWrap(True)

            log_scroll = QScrollArea()
            log_scroll.setObjectName('taskLogScroll')
            log_scroll.setWidgetResizable(True)
            log_scroll.setMinimumHeight(185)
            log_scroll.setWidget(self._task_log)
            self._task_log_scroll = log_scroll

            log_panel = QFrame()
            log_panel.setObjectName('taskLogPanel')
            log_panel.setMinimumHeight(210)
            log_panel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
            log_panel_layout = QVBoxLayout()
            log_panel_layout.setContentsMargins(14, 12, 14, 12)
            log_panel_layout.setSpacing(8)
            log_panel.setLayout(log_panel_layout)
            log_panel_layout.addWidget(log_title)
            log_panel_layout.addWidget(log_scroll, 1)

            left_workspace = QWidget()
            left_workspace_layout = QVBoxLayout()
            left_workspace_layout.setContentsMargins(0, 0, 0, 0)
            left_workspace_layout.setSpacing(18)
            left_workspace.setLayout(left_workspace_layout)

            workspace_grid = QGridLayout()
            workspace_grid.setContentsMargins(0, 0, 0, 0)
            workspace_grid.setHorizontalSpacing(14)
            workspace_grid.setVerticalSpacing(14)
            workspace_grid.addWidget(status_panel, 0, 0)
            workspace_grid.addWidget(detector_panel, 0, 1)
            workspace_grid.addWidget(gripper_panel, 1, 0)
            workspace_grid.addWidget(log_panel, 1, 1)
            workspace_grid.setColumnStretch(0, 0)
            workspace_grid.setColumnStretch(1, 1)
            workspace_grid.setRowStretch(0, 0)
            workspace_grid.setRowStretch(1, 1)
            left_workspace_layout.addLayout(workspace_grid, 1)

            content_layout = QHBoxLayout()
            content_layout.setContentsMargins(0, 0, 0, 0)
            content_layout.setSpacing(14)
            content_layout.addWidget(left_workspace, 0)
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
            if self._on_user_text is not None:
                self._on_user_text(text)

        def append_user(self, text, source='keyboard'):
            self._append_bubble('나', text, align='right', role='user')

        def append_bot(self, text):
            self._append_bubble('MacGyvBot', text, align='left', role='bot')

        def append_system(self, text):
            self._append_bubble('상태', text, align='left', role='system')

        def append_confirmation(self, text):
            self.append_bot(text)
            self._append_confirmation_actions()

        def append_control_actions(self, actions):
            self._append_quick_reply_actions(actions)

        def append_command_result(self, command):
            self._append_command_card(command)

        def set_status(self, text):
            self._current_status.setText(f'현재 상태: {text}')

        def set_connection_status(
            self,
            robot_text,
            camera_text,
            gui_text='연결됨',
            detector_text='대기 중',
        ):
            self._robot_connection_status.setText(f'로봇 노드: {robot_text}')
            self._camera_connection_status.setText(f'카메라 노드: {camera_text}')
            self._detector_connection_status.setText(
                f'Detector 영상: {detector_text}'
            )
            self._gui_connection_status.setText(f'GUI 노드: {gui_text}')
            self._robot_connection_status.setStyleSheet(
                self._connection_style(robot_text)
            )
            self._camera_connection_status.setStyleSheet(
                self._connection_style(camera_text)
            )
            self._detector_connection_status.setStyleSheet(
                self._connection_style(detector_text)
            )
            self._gui_connection_status.setStyleSheet(
                self._connection_style(gui_text)
            )
            self.set_detector_status(detector_text)

        def set_task_status(self, target_text, stage_text):
            self._task_target_status.setText(f'작업 대상: {target_text}')
            self._task_stage_status.setText(f'작업 단계: {stage_text}')

        def set_gripper_control_state(self, enabled, reason):
            reason = str(reason or '').strip()
            if enabled:
                self._gripper_status.setText(reason or '활성화: 수동 조작 가능')
            else:
                self._gripper_status.setText(reason or '비활성화')
            self._set_gripper_controls_enabled(bool(enabled))

        def _set_gripper_controls_enabled(self, enabled):
            self._gripper_slider.setEnabled(enabled)
            self._gripper_width_input.setEnabled(enabled)
            self._gripper_apply_button.setEnabled(enabled)
            self._gripper_status.setStyleSheet(
                self._gripper_status_style(enabled)
            )

        def _set_gripper_width_value(self, value, *, source):
            width_mm = int(value)
            self._gripper_value.setText(f'폭: {width_mm} mm')

            if source != 'slider' and self._gripper_slider.value() != width_mm:
                self._gripper_slider.blockSignals(True)
                self._gripper_slider.setValue(width_mm)
                self._gripper_slider.blockSignals(False)

            if (
                source != 'input'
                and self._gripper_width_input.value() != width_mm
            ):
                self._gripper_width_input.blockSignals(True)
                self._gripper_width_input.setValue(width_mm)
                self._gripper_width_input.blockSignals(False)

        def _sync_gripper_from_slider(self, value):
            self._set_gripper_width_value(value, source='slider')

        def _sync_gripper_from_input(self, value):
            self._set_gripper_width_value(value, source='input')

        def _request_gripper_width_from_slider(self):
            width_mm = int(self._gripper_width_input.value())
            if self._on_gripper_width is not None:
                self._on_gripper_width(width_mm)

        def append_task_log(self, level, message, source='ui', event='EVENT', detail=''):
            level = str(level or 'INFO').upper()
            message = str(message or '').strip()
            if not message:
                return

            source = str(source or 'ui').strip() or 'ui'
            event = str(event or 'EVENT').strip().upper() or 'EVENT'
            detail = str(detail or '').strip()
            entry = (
                f'[{datetime.now().strftime("%H:%M:%S")}] '
                f'[{source}] [{level}] {event} - {message}'
            )
            if detail:
                entry = f'{entry} | {detail}'
            self._task_log_entries.append(entry)
            self._task_log_entries = self._task_log_entries[-80:]
            self._task_log.setText('\n'.join(self._task_log_entries))
            QTimer.singleShot(0, self._scroll_task_log_to_bottom)

        def _scroll_task_log_to_bottom(self):
            scrollbar = self._task_log_scroll.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())

        def set_detector_status(self, status_text):
            self._detector_status.setText(f'Detector 상태: {status_text}')
            self._detector_status.setStyleSheet(
                self._detector_status_style(status_text)
            )
            if self._detector_pixmap is None and status_text == '대기 중':
                self._detector_image.setText('Detector 영상 대기 중')
            if status_text == '미수신' and self._detector_pixmap is None:
                self._detector_image.setText('Detector 영상 미수신')

        def set_detector_image(self, msg):
            image = self._image_msg_to_qimage(msg)
            self._detector_pixmap = QPixmap.fromImage(image)
            self._refresh_detector_pixmap()
            self.set_detector_status('수신 중')

        def resizeEvent(self, event):
            super().resizeEvent(event)
            self._refresh_detector_pixmap()

        def _refresh_detector_pixmap(self):
            if self._detector_pixmap is None:
                return

            scaled = self._detector_pixmap.scaled(
                self._detector_image.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
            self._detector_image.setPixmap(scaled)

        def _image_msg_to_qimage(self, msg):
            width = int(getattr(msg, 'width', 0))
            height = int(getattr(msg, 'height', 0))
            step = int(getattr(msg, 'step', 0))
            encoding = str(getattr(msg, 'encoding', '')).lower()

            if width <= 0 or height <= 0 or step <= 0:
                raise ValueError('잘못된 image 크기입니다.')

            data = bytes(msg.data)
            if encoding == 'rgb8':
                return QImage(
                    data,
                    width,
                    height,
                    step,
                    QImage.Format_RGB888,
                ).copy()

            if encoding == 'bgr8':
                return QImage(
                    data,
                    width,
                    height,
                    step,
                    QImage.Format_RGB888,
                ).rgbSwapped().copy()

            if encoding == 'mono8':
                grayscale_format = getattr(
                    QImage,
                    'Format_Grayscale8',
                    QImage.Format_Indexed8,
                )
                return QImage(
                    data,
                    width,
                    height,
                    step,
                    grayscale_format,
                ).copy()

            raise ValueError(f'지원하지 않는 image encoding입니다: {encoding}')

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
            self._append_quick_reply_actions(
                (
                    ('예, 맞아요', '네'),
                    ('아니요', '아니오'),
                )
            )

        def _append_quick_reply_actions(self, actions):
            row = QWidget()
            row_layout = QHBoxLayout()
            row_layout.setContentsMargins(0, 0, 0, 10)
            row_layout.setSpacing(8)
            row.setLayout(row_layout)

            buttons = []
            for label, reply in actions:
                button = QPushButton(label)
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
                button.clicked.connect(
                    lambda _checked=False, text=reply: self._send_quick_reply(text)
                )
                buttons.append(button)

            for button in buttons:
                row_layout.addWidget(button, 0, Qt.AlignLeft)
            row_layout.addStretch(1)
            self._add_chat_widget(row)

        def _send_quick_reply(self, text):
            self.append_user(text)
            if self._on_user_text is not None:
                self._on_user_text(text)

        def _send_control_text(self, text):
            self.append_user(text)
            if self._on_user_text is not None:
                self._on_user_text(text)

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
            return datetime.now().strftime('%H:%M:%S')

        @staticmethod
        def _action_label(action):
            return {
                'bring': '가져오기',
                'return': '정리하기',
                'release': '놓기',
                'pause': '정지',
                'resume': '재개',
                'cancel': '취소',
                'exit': '종료',
                'home': 'Home 복귀',
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
            connected = status_text in ('연결됨', '실행 중', '수신 중')
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
        def _detector_status_style(status_text):
            connected = status_text == '수신 중'
            color = '#25A55F' if connected else '#7A8794'
            background = '#F5FFF9' if connected else '#FFFFFF'
            border = '#CFEBDD' if connected else '#D5E2F0'
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
                'resume_keyword': 'Control',
                'cancel_keyword': 'Control',
                'exit_keyword': 'Control',
                'home_keyword': 'Control',
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
                QScrollArea#chatScroll {
                    background-color: #E7F1FA;
                    border: 1px solid #B7CEE7;
                    border-radius: 16px;
                }
                QWidget#chatContainer {
                    background-color: #E7F1FA;
                    border-radius: 16px;
                }
                QFrame#statusPanel {
                    background-color: #F8FBFF;
                    border: 1px solid #C9DAEC;
                    border-radius: 16px;
                }
                QFrame#gripperPanel {
                    background-color: #F8FBFF;
                    border: 1px solid #C9DAEC;
                    border-radius: 16px;
                }
                QFrame#detectorPanel {
                    background-color: #F8FBFF;
                    border: 1px solid #C9DAEC;
                    border-radius: 16px;
                }
                QLabel#statusPanelTitle {
                    color: #223B5C;
                    font-size: 17px;
                    font-weight: 900;
                }
                QLabel#gripperPanelTitle {
                    color: #223B5C;
                    font-size: 15px;
                    font-weight: 900;
                }
                QLabel#gripperValue {
                    color: #34536F;
                    font-size: 13px;
                    font-weight: 800;
                }
                QSpinBox#gripperWidthInput {
                    background-color: #FFFFFF;
                    color: #163B5C;
                    border: 1px solid #BFD4EE;
                    border-radius: 10px;
                    padding: 5px 8px;
                    font-size: 12px;
                    font-weight: 700;
                    min-width: 72px;
                }
                QSpinBox#gripperWidthInput:disabled {
                    background-color: #EFF4F9;
                    color: #8A9AAA;
                    border: 1px solid #D3DFEA;
                }
                QSlider#gripperSlider::groove:horizontal {
                    height: 8px;
                    background: #DCE8F5;
                    border-radius: 4px;
                }
                QSlider#gripperSlider::handle:horizontal {
                    background: #2F6FDC;
                    border: 1px solid #1E55B3;
                    width: 18px;
                    margin: -6px 0;
                    border-radius: 9px;
                }
                QSlider#gripperSlider::handle:horizontal:disabled {
                    background: #9AA9B8;
                    border: 1px solid #8A9AAA;
                }
                QPushButton#gripperApplyButton {
                    background-color: #FFFFFF;
                    color: #2563B8;
                    border: 1px solid #BFD4EE;
                    border-radius: 12px;
                    padding: 9px 12px;
                    font-weight: 800;
                }
                QPushButton#gripperApplyButton:disabled {
                    background-color: #EFF4F9;
                    color: #8A9AAA;
                    border: 1px solid #D3DFEA;
                }
                QLabel#statusPanelSubtitle {
                    color: #6A7E96;
                    font-size: 12px;
                }
                QFrame#taskLogPanel {
                    background-color: #F8FBFF;
                    border: 1px solid #C9DAEC;
                    border-radius: 16px;
                }
                QLabel#taskLogTitle {
                    color: #223B5C;
                    font-size: 14px;
                    font-weight: 900;
                    padding-top: 4px;
                }
                QScrollArea#taskLogScroll {
                    background-color: #101B2A;
                    border: 1px solid #263A56;
                    border-radius: 0px;
                }
                QLabel#taskLog {
                    background-color: #101B2A;
                    color: #D9E8F7;
                    padding: 9px 10px;
                    font-family: "D2Coding", "Consolas", "Menlo", monospace;
                    font-size: 11px;
                    line-height: 1.35;
                    border-radius: 0px;
                }
                QLabel#detectorPanelTitle {
                    color: #223B5C;
                    font-size: 17px;
                    font-weight: 900;
                }
                QLabel#detectorPanelSubtitle {
                    color: #6A7E96;
                    font-size: 12px;
                }
                QLabel#detectorImage {
                    background-color: #111827;
                    color: #D8E4F2;
                    border: none;
                    border-radius: 0px;
                    font-size: 15px;
                    font-weight: 700;
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
                QPushButton#homeControlButton {
                    background-color: #FFFFFF;
                    color: #2563B8;
                    border: 1px solid #BFD4EE;
                    border-radius: 12px;
                    padding: 10px 12px;
                    font-weight: 800;
                }
                QPushButton#homeControlButton:hover {
                    background-color: #EEF6FF;
                }
                QPushButton#exitControlButton {
                    background-color: #FFF5F5;
                    color: #C24141;
                    border: 1px solid #F2C6C6;
                    border-radius: 12px;
                    padding: 10px 12px;
                    font-weight: 800;
                }
                QPushButton#exitControlButton:hover {
                    background-color: #FFECEC;
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
            self._detector_connection_status.setStyleSheet(
                self._connection_style('대기 중')
            )
            self._gui_connection_status.setStyleSheet(
                self._connection_style('연결됨')
            )
            self._detector_status.setStyleSheet(
                self._detector_status_style('대기 중')
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

        @staticmethod
        def _gripper_status_style(enabled):
            if enabled:
                color = '#2563B8'
                border = '#BFD4EE'
                background = '#F5FAFF'
            else:
                color = '#7A8794'
                border = '#D5E2F0'
                background = '#F7FAFD'
            return f'''
                background-color: {background};
                border: 1px solid {border};
                border-radius: 10px;
                padding: 8px 10px;
                color: {color};
                font-size: 12px;
                font-weight: 700;
            '''
