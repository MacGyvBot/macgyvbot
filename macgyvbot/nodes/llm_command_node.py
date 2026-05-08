"""Hybrid natural language command interpreter for macgyvbot.

이 노드는 STT가 만든 텍스트(`/stt_text`)를 구독하고, 먼저 빠르고 예측 가능한
alias/fuzzy matching을 시도한다. 그래도 공구를 확정하지 못하면 LLM에게
자연어 명령을 제한된 JSON 명령으로 바꾸게 한다.

중요한 설계 원칙:
- LLM은 로봇을 직접 제어하지 않는다.
- LLM은 tool_name/action/confidence JSON만 만든다.
- 노드는 JSON을 검증한 뒤, 허용된 공구와 행동일 때만 ROS topic을 발행한다.

기본 LLM backend는 로컬 Ollama HTTP API이다.
예:
    ollama pull qwen2.5:0.5b
    ollama serve
"""

import json
import re
from urllib import error, request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from macgyvbot.voice_command.command_parser import find_action, find_tool


ALLOWED_TOOLS = {
    'screwdriver': '드라이버. 나사를 조이거나 푸는 공구.',
    'pliers': '플라이어 또는 펜치. 물체를 집거나 잡는 공구.',
    'hammer': '망치. 못을 박거나 두드리는 공구.',
    'tape_measure': '줄자. 길이나 치수를 재는 공구.',
    'unknown': '공구를 확정할 수 없음.',
}

ALLOWED_ACTIONS = {
    'bring': '사용자가 공구를 가져오거나 달라고 요청함.',
    'release': '사용자가 잡고 있는 공구를 놓으라고 요청함.',
    'stop': '사용자가 정지 또는 중단을 요청함.',
    'unknown': '행동을 확정할 수 없음.',
}

YES_WORDS = {
    '네',
    '예',
    '응',
    '어',
    '맞아',
    '맞습니다',
    '그래',
    '오케이',
    'ok',
    'yes',
    'ㅇㅇ',
}

NO_WORDS = {
    '아니',
    '아니요',
    '아냐',
    '틀려',
    '취소',
    'cancel',
    'no',
    'ㄴㄴ',
}


class LlmCommandNode(Node):
    def __init__(self):
        super().__init__('llm_command_node')

        self.declare_parameter('input_topic', '/stt_text')
        self.declare_parameter('target_label_topic', '/target_label')
        self.declare_parameter('tool_command_topic', '/tool_command')
        self.declare_parameter('command_feedback_topic', '/command_feedback')
        self.declare_parameter(
            'ollama_url',
            'http://localhost:11434/api/generate',
        )
        self.declare_parameter('model', 'qwen2.5:0.5b')
        self.declare_parameter('timeout_sec', 8.0)
        self.declare_parameter('min_confidence', 0.55)
        self.declare_parameter('use_local_parser', True)
        self.declare_parameter('use_llm_fallback', True)

        input_topic = self.get_parameter('input_topic').value
        target_label_topic = self.get_parameter('target_label_topic').value
        tool_command_topic = self.get_parameter('tool_command_topic').value
        command_feedback_topic = self.get_parameter(
            'command_feedback_topic'
        ).value
        self._ollama_url = self.get_parameter('ollama_url').value
        self._model = self.get_parameter('model').value
        self._timeout_sec = float(self.get_parameter('timeout_sec').value)
        self._min_confidence = float(
            self.get_parameter('min_confidence').value
        )
        self._use_local_parser = bool(
            self.get_parameter('use_local_parser').value
        )
        self._use_llm_fallback = bool(
            self.get_parameter('use_llm_fallback').value
        )
        self._pending_command = None

        self._target_label_pub = self.create_publisher(
            String,
            target_label_topic,
            10,
        )
        self._tool_command_pub = self.create_publisher(
            String,
            tool_command_topic,
            10,
        )
        self._feedback_pub = self.create_publisher(
            String,
            command_feedback_topic,
            10,
        )
        self.create_subscription(String, input_topic, self._text_cb, 10)

        self.get_logger().info('하이브리드 명령 해석 노드 준비 완료')
        self.get_logger().info(f'입력 topic: {input_topic}')
        self.get_logger().info(
            f'출력 topic: {tool_command_topic}, {target_label_topic}, '
            f'{command_feedback_topic}'
        )
        self.get_logger().info(
            f'local parser: {"on" if self._use_local_parser else "off"}, '
            f'LLM fallback: {"on" if self._use_llm_fallback else "off"}'
        )
        self.get_logger().info(f'Ollama model: {self._model}')

    def _text_cb(self, msg):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'명령 해석 요청: "{text}"')

        if self._handle_pending_confirmation(text):
            return

        if self._use_local_parser:
            command = self._parse_locally(text)
            if command is not None:
                self._publish_command(command)
                return

        if not self._use_llm_fallback:
            self.get_logger().warn(
                'local parser 실패, LLM fallback이 꺼져 있어 무시합니다.'
            )
            self._publish_feedback(
                status='rejected',
                raw_text=text,
                reason='local_parser_failed',
                message='공구를 확정하지 못했습니다. 다시 말해주세요.',
            )
            return

        self.get_logger().info(f'LLM fallback 요청: "{text}"')
        parsed = self._parse_with_llm(text)
        if parsed is None:
            self._publish_feedback(
                status='rejected',
                raw_text=text,
                reason='llm_failed',
                message='자연어 해석에 실패했습니다. 다시 말해주세요.',
            )
            return

        command = self._validate_command(parsed, text)
        if command is None:
            return

        self._publish_command(command)

    def _parse_locally(self, text):
        tool_name, match_method, match_score, matched_keyword = find_tool(text)
        action = find_action(text)

        if not tool_name:
            self.get_logger().info('local parser가 공구를 확정하지 못했습니다.')
            return None

        command = {
            'tool_name': tool_name,
            'action': action,
            'raw_text': text,
            'match_method': match_method,
            'match_score': match_score,
            'confidence': match_score,
        }

        if match_method == 'fuzzy':
            command['matched_keyword'] = matched_keyword

        self.get_logger().info(
            f'local parser 명령 확정: tool={tool_name}, action={action}, '
            f'method={match_method}, score={match_score:.2f}'
        )
        return command

    def _publish_command(self, command):
        command['status'] = 'accepted'
        command_msg = String()
        command_msg.data = json.dumps(command, ensure_ascii=False)
        self._tool_command_pub.publish(command_msg)
        self._publish_feedback(
            status='accepted',
            raw_text=command.get('raw_text', ''),
            reason='command_accepted',
            message=self._build_accepted_message(command),
            command=command,
        )

        if command['action'] == 'bring':
            target_msg = String()
            target_msg.data = command['tool_name']
            self._target_label_pub.publish(target_msg)
            self.get_logger().info(
                f'명령 발행: tool={command["tool_name"]}, '
                f'action={command["action"]}, '
                f'method={command["match_method"]}, '
                f'confidence={command["confidence"]:.2f} '
                '-> /target_label 발행'
            )
            return

        self.get_logger().info(
            f'명령 발행: tool={command["tool_name"]}, '
            f'action={command["action"]}, method={command["match_method"]}, '
            f'confidence={command["confidence"]:.2f} '
            '-> /tool_command만 발행'
        )

    def _handle_pending_confirmation(self, text):
        if self._pending_command is None:
            return False

        if self._is_yes(text):
            command = self._pending_command
            command['raw_text'] = command.get('raw_text', '')
            command['confirmed_by'] = text
            self._pending_command = None
            self._publish_command(command)
            return True

        if self._is_no(text):
            command = self._pending_command
            self._pending_command = None
            self._publish_feedback(
                status='cancelled',
                raw_text=text,
                reason='confirmation_no',
                message='알겠습니다. 이전 추정 명령은 실행하지 않겠습니다.',
                command=command,
            )
            return True

        self._pending_command = None
        self._publish_feedback(
            status='cancelled',
            raw_text=text,
            reason='new_command_received',
            message='이전 확인 질문은 취소하고 새 명령으로 이해해볼게요.',
        )
        return False

    def _is_yes(self, text):
        normalized = self._normalize_answer(text)
        return any(word in normalized for word in YES_WORDS)

    def _is_no(self, text):
        normalized = self._normalize_answer(text)
        return any(word in normalized for word in NO_WORDS)

    def _normalize_answer(self, text):
        return (text or '').strip().lower().replace(' ', '')

    def _request_confirmation(self, raw_text, command, reason, question):
        command['status'] = 'pending_confirmation'
        self._pending_command = command
        self._publish_feedback(
            status='pending_confirmation',
            raw_text=raw_text,
            reason=reason,
            message=question,
            command=command,
        )

    def _build_accepted_message(self, command):
        tool_name = command.get('tool_name', 'unknown')
        action = command.get('action', 'unknown')

        if action == 'bring':
            return f'{tool_name}를 가져오라는 뜻으로 이해했습니다.'
        if action == 'release':
            return f'{tool_name}를 놓으라는 뜻으로 이해했습니다.'
        if action == 'stop':
            return '정지 명령으로 이해했습니다.'
        return '명령을 올바른 입력으로 판단했습니다.'

    def _build_confirmation_question(self, command):
        tool_name = command.get('tool_name', 'unknown')
        action = command.get('action', 'unknown')

        if tool_name != 'unknown' and action == 'bring':
            return f'{tool_name}를 가져오라는 뜻이 맞나요? 네 또는 아니오로 답해주세요.'
        if tool_name != 'unknown' and action == 'release':
            return f'{tool_name}를 놓으라는 뜻이 맞나요? 네 또는 아니오로 답해주세요.'
        if tool_name != 'unknown':
            return f'{tool_name}를 가져오라는 뜻인가요? 네 또는 아니오로 답해주세요.'
        return '명령을 확신하기 어렵습니다. 다시 입력해주세요.'

    def _publish_feedback(
        self,
        status,
        raw_text,
        reason,
        message,
        command=None,
    ):
        payload = {
            'status': status,
            'raw_text': raw_text,
            'reason': reason,
            'message': message,
        }
        if command is not None:
            payload['command'] = command

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._feedback_pub.publish(msg)

    def _parse_with_llm(self, text):
        prompt = self._build_prompt(text)
        payload = {
            'model': self._model,
            'prompt': prompt,
            'stream': False,
            'options': {
                'temperature': 0.0,
            },
        }
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        req = request.Request(
            self._ollama_url,
            data=data,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )

        try:
            with request.urlopen(req, timeout=self._timeout_sec) as response:
                body = response.read().decode('utf-8')
        except error.URLError as exc:
            self.get_logger().error(f'Ollama 요청 실패: {exc}')
            return None
        except TimeoutError:
            self.get_logger().error('Ollama 요청 timeout')
            return None

        try:
            ollama_result = json.loads(body)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Ollama 응답 JSON 파싱 실패: {exc}')
            return None

        response_text = ollama_result.get('response', '')
        return self._extract_json(response_text)

    def _build_prompt(self, text):
        return f"""
너는 ROS2 로봇팔 프로젝트 macgyvbot의 자연어 명령 해석기다.
사용자 문장을 허용된 JSON 명령 하나로만 변환한다.

허용 tool_name:
- screwdriver: {ALLOWED_TOOLS['screwdriver']}
- pliers: {ALLOWED_TOOLS['pliers']}
- hammer: {ALLOWED_TOOLS['hammer']}
- tape_measure: {ALLOWED_TOOLS['tape_measure']}
- unknown: {ALLOWED_TOOLS['unknown']}

허용 action:
- bring: {ALLOWED_ACTIONS['bring']}
- release: {ALLOWED_ACTIONS['release']}
- stop: {ALLOWED_ACTIONS['stop']}
- unknown: {ALLOWED_ACTIONS['unknown']}

규칙:
- 반드시 JSON만 출력한다.
- 다른 설명 문장을 출력하지 않는다.
- tool_name은 허용 목록 중 하나만 사용한다.
- action은 허용 목록 중 하나만 사용한다.
- confidence는 0.0부터 1.0 사이 숫자다.
- "그 조이는 거", "나사 돌리는 거"는 screwdriver에 가깝다.
- "못 박는 거", "두드리는 거"는 hammer에 가깝다.
- "길이 재는 거", "치수 재는 거"는 tape_measure에 가깝다.
- "집는 거", "잡는 거", "펜치 같은 거"는 pliers에 가깝다.

예시:
입력: 드라이버 가져다줘
출력: {{"tool_name":"screwdriver","action":"bring","confidence":0.95}}

입력: 그 조이는 거 가져와
출력: {{"tool_name":"screwdriver","action":"bring","confidence":0.80}}

입력: 길이 재는 거 줘
출력: {{"tool_name":"tape_measure","action":"bring","confidence":0.82}}

입력: 멈춰
출력: {{"tool_name":"unknown","action":"stop","confidence":0.90}}

입력: {text}
출력:
""".strip()

    def _extract_json(self, response_text):
        cleaned = response_text.strip()
        if cleaned.startswith('```'):
            cleaned = re.sub(r'^```(?:json)?', '', cleaned).strip()
            cleaned = re.sub(r'```$', '', cleaned).strip()

        match = re.search(r'\{.*\}', cleaned, flags=re.DOTALL)
        if not match:
            self.get_logger().error(
                f'LLM 응답에서 JSON을 찾지 못했습니다: {response_text}'
            )
            return None

        try:
            return json.loads(match.group(0))
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'LLM JSON 파싱 실패: {exc}, raw={response_text}'
            )
            return None

    def _validate_command(self, parsed, raw_text):
        tool_name = str(parsed.get('tool_name', 'unknown')).strip()
        action = str(parsed.get('action', 'unknown')).strip()

        try:
            confidence = float(parsed.get('confidence', 0.0))
        except (TypeError, ValueError):
            confidence = 0.0

        if tool_name not in ALLOWED_TOOLS:
            self.get_logger().warn(f'허용되지 않은 tool_name: {tool_name}')
            tool_name = 'unknown'

        if action not in ALLOWED_ACTIONS:
            self.get_logger().warn(f'허용되지 않은 action: {action}')
            action = 'unknown'

        confidence = max(0.0, min(1.0, confidence))

        candidate_command = {
            'tool_name': tool_name,
            'action': action,
            'raw_text': raw_text,
            'match_method': 'llm',
            'match_score': confidence,
            'confidence': confidence,
        }

        if confidence < self._min_confidence:
            self.get_logger().warn(
                f'LLM confidence가 낮아 명령을 무시합니다: '
                f'{confidence:.2f} < {self._min_confidence:.2f}'
            )
            if tool_name != 'unknown' and action != 'unknown':
                self._request_confirmation(
                    raw_text=raw_text,
                    command=candidate_command,
                    reason='low_confidence',
                    question=self._build_confirmation_question(
                        candidate_command
                    ),
                )
                return None

            self._publish_feedback(
                status='rejected',
                raw_text=raw_text,
                reason='low_confidence',
                message='명령 확신도가 낮습니다. 공구 이름을 더 명확히 말해주세요.',
                command=candidate_command,
            )
            return None

        if action == 'unknown':
            self.get_logger().warn('행동을 확정하지 못해 명령을 무시합니다.')
            if tool_name != 'unknown':
                candidate_command['action'] = 'bring'
                self._request_confirmation(
                    raw_text=raw_text,
                    command=candidate_command,
                    reason='unknown_action',
                    question=self._build_confirmation_question(
                        candidate_command
                    ),
                )
                return None

            self._publish_feedback(
                status='rejected',
                raw_text=raw_text,
                reason='unknown_action',
                message='무엇을 해야 하는지 확정하지 못했습니다. 다시 입력해주세요.',
                command=candidate_command,
            )
            return None

        if action == 'bring' and tool_name == 'unknown':
            self.get_logger().warn('bring 명령이지만 공구를 확정하지 못해 무시합니다.')
            self._publish_feedback(
                status='rejected',
                raw_text=raw_text,
                reason='unknown_tool',
                message=(
                    '어떤 공구를 가져올지 확실하지 않습니다. '
                    '드라이버, 플라이어, 망치, 줄자 중에서 다시 말해주세요.'
                ),
                command=candidate_command,
            )
            return None

        return candidate_command


def main(args=None):
    rclpy.init(args=args)
    node = LlmCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
