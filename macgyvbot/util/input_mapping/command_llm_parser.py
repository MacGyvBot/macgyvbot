"""Hybrid local+LLM command parser for macgyvbot voice commands."""

import json
import re
from urllib import error, request

from macgyvbot.util.input_mapping.command_hard_parser import (
    find_action,
    find_tool,
)


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


class CommandLlmParser:
    def __init__(
        self,
        ollama_url,
        model,
        timeout_sec,
        min_confidence,
        use_local_parser,
        use_llm_fallback,
        logger=None,
    ):
        self._ollama_url = ollama_url
        self._model = model
        self._timeout_sec = float(timeout_sec)
        self._min_confidence = float(min_confidence)
        self._use_local_parser = bool(use_local_parser)
        self._use_llm_fallback = bool(use_llm_fallback)
        self._pending_command = None
        self._logger = logger

    def interpret(self, text):
        text = (text or '').strip()
        if not text:
            return {'command': None, 'feedbacks': []}

        feedbacks = []

        pending_result = self._handle_pending_confirmation(text)
        if pending_result is not None:
            return pending_result

        if self._use_local_parser:
            command = self._parse_locally(text)
            if command is not None:
                return {
                    'command': self._mark_accepted(command),
                    'feedbacks': [
                        self._feedback(
                            status='accepted',
                            raw_text=command.get('raw_text', ''),
                            reason='command_accepted',
                            message=self._build_accepted_message(command),
                            command=command,
                        )
                    ],
                }

        if not self._use_llm_fallback:
            self._warn('local parser 실패, LLM fallback이 꺼져 있어 무시합니다.')
            return {
                'command': None,
                'feedbacks': [
                    self._feedback(
                        status='rejected',
                        raw_text=text,
                        reason='local_parser_failed',
                        message='공구를 확정하지 못했습니다. 다시 말해주세요.',
                    )
                ],
            }

        self._info(f'LLM fallback 요청: "{text}"')
        parsed = self._parse_with_llm(text)
        if parsed is None:
            return {
                'command': None,
                'feedbacks': [
                    self._feedback(
                        status='rejected',
                        raw_text=text,
                        reason='llm_failed',
                        message='자연어 해석에 실패했습니다. 다시 말해주세요.',
                    )
                ],
            }

        validation = self._validate_command(parsed, text)
        if validation['command'] is not None:
            accepted_command = self._mark_accepted(validation['command'])
            feedbacks.append(
                self._feedback(
                    status='accepted',
                    raw_text=accepted_command.get('raw_text', ''),
                    reason='command_accepted',
                    message=self._build_accepted_message(accepted_command),
                    command=accepted_command,
                )
            )
            return {'command': accepted_command, 'feedbacks': feedbacks}

        if validation.get('feedback') is not None:
            feedbacks.append(validation['feedback'])

        return {'command': None, 'feedbacks': feedbacks}

    def _mark_accepted(self, command):
        command['status'] = 'accepted'
        return command

    def _parse_locally(self, text):
        tool_name, match_method, match_score, matched_keyword = find_tool(text)
        action = find_action(text)

        if not tool_name:
            self._info('local parser가 공구를 확정하지 못했습니다.')
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

        self._info(
            f'local parser 명령 확정: tool={tool_name}, action={action}, '
            f'method={match_method}, score={match_score:.2f}'
        )
        return command

    def _handle_pending_confirmation(self, text):
        if self._pending_command is None:
            return None

        if self._is_yes(text):
            command = self._pending_command
            command['raw_text'] = command.get('raw_text', '')
            command['confirmed_by'] = text
            self._pending_command = None
            accepted_command = self._mark_accepted(command)
            return {
                'command': accepted_command,
                'feedbacks': [
                    self._feedback(
                        status='accepted',
                        raw_text=accepted_command.get('raw_text', ''),
                        reason='command_accepted',
                        message=self._build_accepted_message(accepted_command),
                        command=accepted_command,
                    )
                ],
            }

        if self._is_no(text):
            command = self._pending_command
            self._pending_command = None
            return {
                'command': None,
                'feedbacks': [
                    self._feedback(
                        status='cancelled',
                        raw_text=text,
                        reason='confirmation_no',
                        message='알겠습니다. 이전 추정 명령은 실행하지 않겠습니다.',
                        command=command,
                    )
                ],
            }

        self._pending_command = None
        return {
            'command': None,
            'feedbacks': [
                self._feedback(
                    status='cancelled',
                    raw_text=text,
                    reason='new_command_received',
                    message='이전 확인 질문은 취소하고 새 명령으로 이해해볼게요.',
                )
            ],
        }

    def _is_yes(self, text):
        normalized = self._normalize_answer(text)
        return any(word in normalized for word in YES_WORDS)

    def _is_no(self, text):
        normalized = self._normalize_answer(text)
        return any(word in normalized for word in NO_WORDS)

    def _normalize_answer(self, text):
        return (text or '').strip().lower().replace(' ', '')

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

    def _feedback(self, status, raw_text, reason, message, command=None):
        payload = {
            'status': status,
            'raw_text': raw_text,
            'reason': reason,
            'message': message,
        }
        if command is not None:
            payload['command'] = command
        return payload

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
            self._error(f'Ollama 요청 실패: {exc}')
            return None
        except TimeoutError:
            self._error('Ollama 요청 timeout')
            return None

        try:
            ollama_result = json.loads(body)
        except json.JSONDecodeError as exc:
            self._error(f'Ollama 응답 JSON 파싱 실패: {exc}')
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
            self._error(f'LLM 응답에서 JSON을 찾지 못했습니다: {response_text}')
            return None

        try:
            return json.loads(match.group(0))
        except json.JSONDecodeError as exc:
            self._error(f'LLM JSON 파싱 실패: {exc}, raw={response_text}')
            return None

    def _validate_command(self, parsed, raw_text):
        tool_name = str(parsed.get('tool_name', 'unknown')).strip()
        action = str(parsed.get('action', 'unknown')).strip()

        try:
            confidence = float(parsed.get('confidence', 0.0))
        except (TypeError, ValueError):
            confidence = 0.0

        if tool_name not in ALLOWED_TOOLS:
            self._warn(f'허용되지 않은 tool_name: {tool_name}')
            tool_name = 'unknown'

        if action not in ALLOWED_ACTIONS:
            self._warn(f'허용되지 않은 action: {action}')
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
            self._warn(
                f'LLM confidence가 낮아 명령을 무시합니다: '
                f'{confidence:.2f} < {self._min_confidence:.2f}'
            )
            if tool_name != 'unknown' and action != 'unknown':
                self._pending_command = candidate_command
                return {
                    'command': None,
                    'feedback': self._feedback(
                        status='pending_confirmation',
                        raw_text=raw_text,
                        reason='low_confidence',
                        message=self._build_confirmation_question(
                            candidate_command
                        ),
                        command=candidate_command,
                    ),
                }

            return {
                'command': None,
                'feedback': self._feedback(
                    status='rejected',
                    raw_text=raw_text,
                    reason='low_confidence',
                    message='명령 확신도가 낮습니다. 공구 이름을 더 명확히 말해주세요.',
                    command=candidate_command,
                ),
            }

        if action == 'unknown':
            self._warn('행동을 확정하지 못해 명령을 무시합니다.')
            if tool_name != 'unknown':
                candidate_command['action'] = 'bring'
                self._pending_command = candidate_command
                return {
                    'command': None,
                    'feedback': self._feedback(
                        status='pending_confirmation',
                        raw_text=raw_text,
                        reason='unknown_action',
                        message=self._build_confirmation_question(
                            candidate_command
                        ),
                        command=candidate_command,
                    ),
                }

            return {
                'command': None,
                'feedback': self._feedback(
                    status='rejected',
                    raw_text=raw_text,
                    reason='unknown_action',
                    message='무엇을 해야 하는지 확정하지 못했습니다. 다시 입력해주세요.',
                    command=candidate_command,
                ),
            }

        if action == 'bring' and tool_name == 'unknown':
            self._warn('bring 명령이지만 공구를 확정하지 못해 무시합니다.')
            return {
                'command': None,
                'feedback': self._feedback(
                    status='rejected',
                    raw_text=raw_text,
                    reason='unknown_tool',
                    message=(
                        '어떤 공구를 가져올지 확실하지 않습니다. '
                        '드라이버, 플라이어, 망치, 줄자 중에서 다시 말해주세요.'
                    ),
                    command=candidate_command,
                ),
            }

        return {'command': candidate_command, 'feedback': None}

    def _info(self, message):
        if self._logger is not None:
            self._logger('info', message)

    def _warn(self, message):
        if self._logger is not None:
            self._logger('warn', message)

    def _error(self, message):
        if self._logger is not None:
            self._logger('error', message)
