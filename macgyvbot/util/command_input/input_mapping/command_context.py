"""Small in-memory context for command interpretation.

The context is intentionally bounded and volatile.  It helps the parser resolve
short phrases such as "아까 가져온 거 정리해" without persisting stale robot
state across process restarts.
"""

from collections import deque


SUCCESS_STATES = {'done', 'completed', 'success', 'returned'}


class CommandContext:
    def __init__(self, max_inputs=5, max_commands=3):
        self._recent_inputs = deque(maxlen=max_inputs)
        self._recent_commands = deque(maxlen=max_commands)
        self.last_tool = ''
        self.last_action = ''
        self.last_successful_tool = ''
        self.last_successful_action = ''
        self.current_robot_status = ''
        self.current_robot_message = ''

    def record_user_input(self, text):
        text = (text or '').strip()
        if text:
            self._recent_inputs.append(text)

    def record_accepted_command(self, command):
        if not command:
            return

        tool_name = command.get('tool_name', 'unknown')
        action = command.get('action', 'unknown')

        if tool_name and tool_name != 'unknown':
            self.last_tool = tool_name
        if action and action != 'unknown':
            self.last_action = action

        self._recent_commands.append(
            {
                'tool_name': tool_name,
                'action': action,
                'target_mode': command.get('target_mode', 'unknown'),
                'raw_text': command.get('raw_text', ''),
            }
        )

    def update_robot_status(self, status):
        if not status:
            return

        state = status.get('status', status.get('state', 'unknown'))
        tool_name = status.get('tool_name', '') or self.last_tool
        message = status.get('message', '')

        self.current_robot_status = state
        self.current_robot_message = message

        if state in SUCCESS_STATES and tool_name and tool_name != 'unknown':
            self.last_successful_tool = tool_name
            self.last_successful_action = self.last_action

    def resolve_previous_tool(self):
        return self.last_successful_tool or self.last_tool or ''

    def robot_status_message(self):
        if self.current_robot_message:
            return self.current_robot_message
        if self.current_robot_status:
            return f'현재 로봇 상태는 {self.current_robot_status}입니다.'
        return '아직 로봇 작업 상태를 받은 적이 없습니다.'

    def prompt_summary(self):
        lines = []
        previous_tool = self.resolve_previous_tool()

        if previous_tool:
            lines.append(f'- 최근 참조 가능한 공구: {previous_tool}')
        if self.last_action:
            lines.append(f'- 최근 명령 동작: {self.last_action}')
        if self.current_robot_status:
            lines.append(f'- 현재 로봇 상태: {self.current_robot_status}')
        if self.current_robot_message:
            lines.append(f'- 최근 로봇 메시지: {self.current_robot_message}')
        if self._recent_inputs:
            lines.append('- 최근 사용자 입력: ' + ' / '.join(self._recent_inputs))

        if not lines:
            return '- 아직 기억할 작업 맥락이 없습니다.'

        return '\n'.join(lines)
