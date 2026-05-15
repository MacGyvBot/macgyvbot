"""Hybrid local+LLM command parser for macgyvbot voice commands."""

import json
import re
from urllib import error, request

from macgyvbot.command_input.input_mapping.command_hard_parser import (
    find_action,
    find_tool,
    normalize_text,
)
from macgyvbot.command_input.input_mapping.command_context import CommandContext
from macgyvbot.command_input.input_mapping.command_vocabulary import (
    ALLOWED_ACTIONS,
    ALLOWED_INTENTS,
    ALLOWED_TARGET_MODES,
    ALLOWED_TOOLS,
    DEICTIC_WORDS,
    NO_WORDS,
    STOP_KEYWORDS,
    YES_WORDS,
)

LOCAL_FUZZY_ACCEPT_THRESHOLD = 0.85
CONFIRMATION_CONFIDENCE_THRESHOLD = 0.70


class CommandLlmParser:
    def __init__(
        self,
        ollama_url,
        model,
        timeout_sec,
        min_confidence,
        use_local_parser,
        use_llm_fallback,
        parser_mode='hybrid',
        context=None,
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
        self._parser_mode = self._normalize_parser_mode(parser_mode)
        self._context = context if context is not None else CommandContext()

    def interpret(self, text):
        text = (text or '').strip()
        if not text:
            return {'command': None, 'feedbacks': []}

        self._context.record_user_input(text)

        pending_result = self._handle_pending_confirmation(text)
        if pending_result is not None:
            return pending_result

        if self._has_stop_intent(text):
            return self._accepted_result(
                {
                    'tool_name': 'unknown',
                    'action': 'stop',
                    'target_mode': 'unknown',
                    'raw_text': text,
                    'match_method': 'stop_keyword',
                    'match_score': 1.0,
                    'confidence': 1.0,
                }
            )

        llm_rejected_result = None
        if self._parser_mode == 'llm_primary' and self._use_llm_fallback:
            llm_result = self._interpret_with_llm(text, request_label='LLM primary 요청')
            if llm_result is not None:
                if not self._is_rejected_result(llm_result):
                    return llm_result
                llm_rejected_result = llm_result

            self._info('LLM primary가 확정하지 못해 local parser 보조를 시도합니다.')

        local_conversation_result = self._parse_local_conversation(text)
        if local_conversation_result is not None:
            return local_conversation_result

        if self._use_local_parser:
            command = self._parse_locally(text)
            if command is not None:
                return self._accepted_result(command)

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

        if self._parser_mode == 'llm_primary' and llm_rejected_result is not None:
            return llm_rejected_result

        llm_result = self._interpret_with_llm(text, request_label='LLM fallback 요청')
        if llm_result is not None:
            return llm_result

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

    def update_robot_status(self, status):
        self._context.update_robot_status(status)

    def _normalize_parser_mode(self, parser_mode):
        parser_mode = str(parser_mode or 'hybrid').strip().lower()
        if parser_mode not in ('hybrid', 'llm_primary'):
            self._warn(f'알 수 없는 parser_mode={parser_mode}, hybrid로 실행합니다.')
            return 'hybrid'
        return parser_mode

    def _is_rejected_result(self, result):
        if result.get('command') is not None:
            return False

        feedbacks = result.get('feedbacks', [])
        if not feedbacks:
            return True

        return all(
            feedback.get('status') == 'rejected'
            for feedback in feedbacks
        )

    def _accepted_result(self, command):
        accepted_command = self._mark_accepted(command)
        self._context.record_accepted_command(accepted_command)
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

    def _interpret_with_llm(self, text, request_label):
        self._info(f'{request_label}: "{text}"')
        parsed = self._parse_with_llm(text)
        if parsed is None:
            return None

        conversation_result = self._handle_conversation_intent(parsed, text)
        if conversation_result is not None:
            return conversation_result

        validation = self._validate_command(parsed, text)
        if validation['command'] is not None:
            return self._accepted_result(validation['command'])

        feedbacks = []
        if validation.get('feedback') is not None:
            feedbacks.append(validation['feedback'])

        return {'command': None, 'feedbacks': feedbacks}

    def _handle_conversation_intent(self, parsed, raw_text):
        intent = str(parsed.get('intent', 'command')).strip()
        if intent not in ALLOWED_INTENTS:
            intent = 'command'

        if intent == 'command':
            return None

        if intent == 'status_query':
            message = self._context.robot_status_message()
            return {
                'command': None,
                'feedbacks': [
                    self._feedback(
                        status='assistant_response',
                        raw_text=raw_text,
                        reason='status_query',
                        message=message,
                    )
                ],
            }

        if intent == 'smalltalk':
            message = parsed.get('assistant_message') or '네, 필요한 공구가 있으면 바로 말해주세요.'
            return {
                'command': None,
                'feedbacks': [
                    self._feedback(
                        status='assistant_response',
                        raw_text=raw_text,
                        reason='smalltalk',
                        message=message,
                    )
                ],
            }

        return None

    def _mark_accepted(self, command):
        command['status'] = 'accepted'
        return command

    def _parse_locally(self, text):
        effective_text = self._effective_command_text(text)
        tool_name, match_method, match_score, matched_keyword = find_tool(effective_text)
        action = find_action(effective_text)

        if action == 'stop':
            command = {
                'tool_name': 'unknown',
                'action': 'stop',
                'target_mode': 'unknown',
                'raw_text': text,
                'match_method': 'stop_keyword',
                'match_score': 1.0,
                'confidence': 1.0,
            }
            self._info('local parser가 정지 명령으로 확정했습니다.')
            return command

        previous_tool = self._context.resolve_previous_tool()
        previous_command = self._context.resolve_previous_command()
        if action == 'unknown' and self._has_repeat_reference(text) and previous_command:
            command = {
                'tool_name': previous_command.get('tool_name', 'unknown'),
                'action': previous_command.get('action', 'unknown'),
                'target_mode': previous_command.get('target_mode', 'unknown'),
                'raw_text': text,
                'match_method': 'local_context',
                'match_score': 1.0,
                'confidence': 1.0,
                'context_used': 'previous_command',
            }
            self._info('local parser가 이전 명령 반복으로 확정했습니다.')
            return command

        if action == 'return' and self._has_previous_reference(text):
            if previous_tool:
                command = {
                    'tool_name': previous_tool,
                    'action': 'return',
                    'target_mode': 'named',
                    'raw_text': text,
                    'match_method': 'local_context',
                    'match_score': 1.0,
                    'confidence': 1.0,
                    'context_used': 'previous_tool',
                }
                self._info(
                    f'local parser가 이전 context 기반 return 명령으로 확정했습니다: '
                    f'tool={previous_tool}'
                )
                return command

            self._info('이전 공구 참조가 있지만 context에 공구가 없어 LLM으로 넘깁니다.')
            return None

        if action == 'bring' and self._has_previous_reference(text) and previous_tool:
            command = {
                'tool_name': previous_tool,
                'action': 'bring',
                'target_mode': 'named',
                'raw_text': text,
                'match_method': 'local_context',
                'match_score': 1.0,
                'confidence': 1.0,
                'context_used': 'previous_tool',
            }
            self._info(
                f'local parser가 이전 context 기반 bring 명령으로 확정했습니다: '
                f'tool={previous_tool}'
            )
            return command

        if action == 'stop':
            command = {
                'tool_name': 'unknown',
                'action': 'stop',
                'target_mode': 'unknown',
                'raw_text': text,
                'match_method': 'stop_keyword',
                'match_score': 1.0,
                'confidence': 1.0,
            }
            self._info('local parser가 정지 명령으로 확정했습니다.')
            return command

        if not tool_name:
            if action == 'return' and self._has_deictic_target(text):
                command = {
                    'tool_name': 'unknown',
                    'action': 'return',
                    'target_mode': 'deictic',
                    'raw_text': text,
                    'match_method': 'local_deictic',
                    'match_score': 1.0,
                    'confidence': 1.0,
                }
                self._info('local parser가 지시어 기반 return 명령으로 확정했습니다.')
                return command

            self._info('local parser가 공구를 확정하지 못했습니다.')
            return None

        if (
            match_method == 'fuzzy'
            and match_score < LOCAL_FUZZY_ACCEPT_THRESHOLD
        ):
            self._info(
                f'local fuzzy parser 결과가 낮아 LLM fallback으로 넘깁니다: '
                f'tool={tool_name}, keyword={matched_keyword}, score={match_score:.2f}'
            )
            return None

        if action == 'unknown':
            self._info('local parser가 행동을 확정하지 못해 LLM으로 넘깁니다.')
            return None

        command = {
            'tool_name': tool_name,
            'action': action,
            'target_mode': 'named',
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

    def _has_deictic_target(self, text):
        normalized = self._normalize_answer(text)
        return any(word in normalized for word in DEICTIC_WORDS)

    def _handle_pending_confirmation(self, text):
        if self._pending_command is None:
            return None

        if self._is_yes(text):
            command = self._pending_command
            command['raw_text'] = command.get('raw_text', '')
            command['confirmed_by'] = text
            self._pending_command = None
            return self._accepted_result(command)

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
        if action == 'return':
            if command.get('target_mode') == 'deictic':
                return '가리킨 공구를 정리하라는 뜻으로 이해했습니다.'
            return f'{tool_name}를 정리하라는 뜻으로 이해했습니다.'
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
        if action == 'return' and command.get('target_mode') == 'deictic':
            return '지금 가리킨 공구를 정리하라는 뜻이 맞나요? 네 또는 아니오로 답해주세요.'
        if tool_name != 'unknown' and action == 'return':
            return f'{tool_name}를 정리하라는 뜻이 맞나요? 네 또는 아니오로 답해주세요.'
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
            'format': 'json',
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
        if not response_text:
            self._error(f'Ollama 응답에 response가 없습니다: {body}')
            return None
        return self._extract_json(response_text)

    def _build_prompt(self, text):
        context_summary = self._context.prompt_summary()
        return f"""
너는 ROS2 로봇팔 프로젝트 macgyvbot의 자연어 명령 해석기다.
사용자 문장을 허용된 JSON 하나로만 변환한다.
로봇 작업 명령이 아닌 짧은 대화와 상태 질문은 명령으로 바꾸지 않는다.

현재 대화/작업 맥락:
{context_summary}

허용 intent:
- command: {ALLOWED_INTENTS['command']}
- status_query: {ALLOWED_INTENTS['status_query']}
- smalltalk: {ALLOWED_INTENTS['smalltalk']}
- unknown: {ALLOWED_INTENTS['unknown']}

허용 tool_name:
- screwdriver: {ALLOWED_TOOLS['screwdriver']}
- pliers: {ALLOWED_TOOLS['pliers']}
- hammer: {ALLOWED_TOOLS['hammer']}
- tape_measure: {ALLOWED_TOOLS['tape_measure']}
- unknown: {ALLOWED_TOOLS['unknown']}

허용 action:
- bring: {ALLOWED_ACTIONS['bring']}
- return: {ALLOWED_ACTIONS['return']}
- release: {ALLOWED_ACTIONS['release']}
- stop: {ALLOWED_ACTIONS['stop']}
- unknown: {ALLOWED_ACTIONS['unknown']}

허용 target_mode:
- named: {ALLOWED_TARGET_MODES['named']}
- deictic: {ALLOWED_TARGET_MODES['deictic']}
- unknown: {ALLOWED_TARGET_MODES['unknown']}

규칙:
- 반드시 JSON만 출력한다.
- 다른 설명 문장을 출력하지 않는다.
- intent는 command, status_query, smalltalk, unknown 중 하나다.
- tool_name은 허용 목록 중 하나만 사용한다.
- action은 허용 목록 중 하나만 사용한다.
- target_mode는 허용 목록 중 하나만 사용한다.
- confidence는 0.0부터 1.0 사이 숫자다.
- needs_confirmation은 true 또는 false다.
- context_used는 none, previous_tool, robot_status 중 하나다.
- "드라이버", "나사 돌리는 거", "나사 조이는 거", "돌려서 쓰는 거"는 screwdriver에 가깝다.
- "못 박는 거", "두드리는 거", "치는 거", "때리는 거"는 hammer에 가깝다.
- "길이 재는 거", "치수 재는 거"는 tape_measure에 가깝다.
- "집는 거", "잡는 거", "펜치 같은 거"는 pliers에 가깝다.
- 드릴, 렌치, 스패너는 현재 프로젝트 대상 공구가 아니므로 tool_name unknown으로 둔다.
- "나사 돌리는 거", "못 두드리는 거"처럼 기능 표현이 있으면 target_mode는 named다.
- "이거", "그거", "저거"처럼 공구명이나 기능 표현이 없는 지시어는 target_mode를 deictic으로 둔다.
- deictic은 return action에서만 허용한다. deictic bring은 tool_name을 추측하지 말고 unknown으로 둔다.
- "아까 가져온 거", "방금 가져온 거", "전에 준 거"처럼 이전 공구를 가리키면 현재 맥락의 최근 참조 가능한 공구를 tool_name으로 쓰고 context_used를 previous_tool로 둔다.
- 최근 참조 가능한 공구가 없으면 이전 공구를 추측하지 말고 tool_name unknown으로 둔다.
- "정리해", "가져다놔", "가져가", "제자리에 둬", "서랍에 넣어", "반납해"는 return action이다.
- "지금 뭐해", "어디까지 했어", "현재 상태 알려줘"는 status_query intent다.
- "고마워", "안녕", "알겠어" 같은 대화는 smalltalk intent다.
- status_query와 smalltalk는 tool_name unknown, action unknown, target_mode unknown으로 둔다.
- status_query와 smalltalk는 assistant_message에 사용자가 볼 짧은 한국어 응답을 넣는다.
- "멈춰", "정지", "중지", "중단", "스탑", "stop", "pause"처럼 명확한 정지 표현만 stop action이다.
- 이해하지 못한 문장을 stop으로 추측하지 말고 action unknown으로 둔다.

예시:
입력: 드라이버 가져다줘
출력: {{"intent":"command","tool_name":"screwdriver","action":"bring","target_mode":"named","confidence":0.95,"needs_confirmation":false,"context_used":"none"}}

입력: 그 조이는 거 가져와
출력: {{"intent":"command","tool_name":"screwdriver","action":"bring","target_mode":"named","confidence":0.80,"needs_confirmation":false,"context_used":"none"}}

입력: 그거 가져와
출력: {{"intent":"command","tool_name":"unknown","action":"bring","target_mode":"deictic","confidence":0.70,"needs_confirmation":true,"context_used":"none"}}

입력: 이거 정리해
출력: {{"intent":"command","tool_name":"unknown","action":"return","target_mode":"deictic","confidence":0.88,"needs_confirmation":false,"context_used":"none"}}

입력: 아까 가져온 거 정리해
조건: 현재 맥락의 최근 참조 가능한 공구가 screwdriver인 경우
출력: {{"intent":"command","tool_name":"screwdriver","action":"return","target_mode":"named","confidence":0.86,"needs_confirmation":false,"context_used":"previous_tool"}}

입력: 나사 돌리는 거 정리해
출력: {{"intent":"command","tool_name":"screwdriver","action":"return","target_mode":"named","confidence":0.88,"needs_confirmation":false,"context_used":"none"}}

입력: 못 두드리는 거 정리해
출력: {{"intent":"command","tool_name":"hammer","action":"return","target_mode":"named","confidence":0.88,"needs_confirmation":false,"context_used":"none"}}

입력: 드라이버 가져다놔
출력: {{"intent":"command","tool_name":"screwdriver","action":"return","target_mode":"named","confidence":0.92,"needs_confirmation":false,"context_used":"none"}}

입력: 길이 재는 거 줘
출력: {{"intent":"command","tool_name":"tape_measure","action":"bring","target_mode":"named","confidence":0.82,"needs_confirmation":false,"context_used":"none"}}

입력: 멈춰
출력: {{"intent":"command","tool_name":"unknown","action":"stop","target_mode":"unknown","confidence":0.90,"needs_confirmation":false,"context_used":"none"}}

입력: 지금 뭐 하는 중이야?
출력: {{"intent":"status_query","tool_name":"unknown","action":"unknown","target_mode":"unknown","confidence":0.90,"needs_confirmation":false,"context_used":"robot_status","assistant_message":"현재 로봇 상태를 확인해드릴게요."}}

입력: 고마워
출력: {{"intent":"smalltalk","tool_name":"unknown","action":"unknown","target_mode":"unknown","confidence":0.90,"needs_confirmation":false,"context_used":"none","assistant_message":"네, 필요한 공구가 있으면 언제든 말해주세요."}}

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
        effective_text = self._effective_command_text(raw_text)
        action = self._adjust_action(
            effective_text,
            str(parsed.get('action', 'unknown')).strip(),
        )
        target_mode = str(parsed.get('target_mode', 'unknown')).strip()
        context_used = str(parsed.get('context_used', 'none')).strip()
        needs_confirmation = self._as_bool(
            parsed.get('needs_confirmation', False)
        )

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

        if target_mode not in ALLOWED_TARGET_MODES:
            self._warn(f'허용되지 않은 target_mode: {target_mode}')
            target_mode = 'unknown'

        if context_used not in ('none', 'previous_tool', 'robot_status'):
            context_used = 'none'

        previous_tool = self._context.resolve_previous_tool()
        previous_command = self._context.resolve_previous_command()

        if action == 'unknown' and self._has_repeat_reference(raw_text) and previous_command:
            tool_name = previous_command.get('tool_name', tool_name)
            action = previous_command.get('action', action)
            target_mode = previous_command.get('target_mode', 'named')
            context_used = 'previous_command'
            confidence = max(confidence, self._min_confidence + 0.10)

        if (
            self._has_previous_reference(raw_text)
            and previous_tool
            and action in ('bring', 'return')
        ):
            tool_name = previous_tool
            target_mode = 'named'
            context_used = 'previous_tool'
            confidence = max(confidence, self._min_confidence + 0.10)

        deterministic_tool = self._resolve_deterministic_tool(effective_text)
        if deterministic_tool:
            tool_name = deterministic_tool
            target_mode = 'named'
            confidence = max(confidence, self._min_confidence + 0.10)

        if (
            action == 'return'
            and self._has_previous_reference(raw_text)
            and not previous_tool
            and not self._has_explicit_tool_clue(raw_text)
        ):
            tool_name = 'unknown'
            target_mode = 'unknown'
            context_used = 'none'
            confidence = min(confidence, self._min_confidence - 0.05)

        if (
            self._has_deictic_target(effective_text)
            and not self._has_explicit_tool_clue(effective_text)
            and not self._has_previous_reference(raw_text)
        ):
            tool_name = 'unknown'
            target_mode = 'deictic'
            confidence = max(confidence, self._min_confidence + 0.05)

        if (
            action == 'return'
            and tool_name == 'unknown'
            and previous_tool
            and (context_used == 'previous_tool' or self._has_previous_reference(raw_text))
        ):
            tool_name = previous_tool
            target_mode = 'named'
            context_used = 'previous_tool'
            confidence = max(confidence, self._min_confidence + 0.05)

        confidence = max(0.0, min(1.0, confidence))
        tool_name, target_mode, confidence = self._adjust_ambiguous_command(
            effective_text,
            tool_name,
            target_mode,
            confidence,
        )

        candidate_command = {
            'tool_name': tool_name,
            'action': action,
            'target_mode': target_mode,
            'raw_text': raw_text,
            'match_method': 'llm',
            'match_score': confidence,
            'confidence': confidence,
        }
        if context_used != 'none':
            candidate_command['context_used'] = context_used

        if self._contains_unsupported_tool(effective_text):
            return {
                'command': None,
                'feedback': self._feedback(
                    status='rejected',
                    raw_text=raw_text,
                    reason='unsupported_tool',
                    message=(
                        '현재 사용할 수 있는 공구는 드라이버, 플라이어, 망치, 줄자입니다. '
                        '그중 하나로 다시 말해주세요.'
                    ),
                    command=candidate_command,
                ),
            }

        if action == 'bring' and target_mode == 'deictic':
            self._warn('deictic bring 명령은 지원하지 않아 재입력을 요청합니다.')
            return {
                'command': None,
                'feedback': self._feedback(
                    status='rejected',
                    raw_text=raw_text,
                    reason='deictic_bring_not_supported',
                    message=(
                        '가져오기 명령은 공구 이름이 필요합니다. '
                        '어떤 공구를 가져올지 말해주세요.'
                    ),
                    command=candidate_command,
                ),
            }

        requires_confirmation = confidence < self._min_confidence or (
            needs_confirmation
            and not self._is_confident_named_command(
                action,
                tool_name,
                target_mode,
                confidence,
            )
            and not self._is_safe_deictic_return(
                action,
                target_mode,
                confidence,
            )
        )

        if requires_confirmation:
            self._warn(
                f'확인이 필요한 LLM 명령입니다: confidence={confidence:.2f}, '
                f'needs_confirmation={needs_confirmation}'
            )
            if action != 'unknown' and (
                tool_name != 'unknown' or target_mode == 'deictic'
            ):
                self._pending_command = candidate_command
                return {
                    'command': None,
                    'feedback': self._feedback(
                        status='pending_confirmation',
                        raw_text=raw_text,
                        reason='needs_confirmation'
                        if needs_confirmation
                        else 'low_confidence',
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

        if action == 'stop' and not self._has_stop_intent(raw_text):
            self._warn('명확한 정지 표현이 없어 stop 명령을 무시합니다.')
            candidate_command['action'] = 'unknown'
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

        if action == 'bring' and target_mode == 'deictic':
            self._warn('deictic bring 명령은 지원하지 않아 재입력을 요청합니다.')
            return {
                'command': None,
                'feedback': self._feedback(
                    status='rejected',
                    raw_text=raw_text,
                    reason='deictic_bring_not_supported',
                    message=(
                        '가져오기 명령은 공구 이름이 필요합니다. '
                        '어떤 공구를 가져올지 말해주세요.'
                    ),
                    command=candidate_command,
                ),
            }

        if (
            action in ('bring', 'return')
            and tool_name == 'unknown'
            and target_mode != 'deictic'
        ):
            self._warn(f'{action} 명령이지만 공구를 확정하지 못해 무시합니다.')
            return {
                'command': None,
                'feedback': self._feedback(
                    status='rejected',
                    raw_text=raw_text,
                    reason='unknown_tool',
                    message=(
                        '어떤 공구인지 확실하지 않습니다. '
                        '드라이버, 플라이어, 망치, 줄자 중에서 다시 말해주세요.'
                    ),
                    command=candidate_command,
                ),
            }

        return {'command': candidate_command, 'feedback': None}

    def _parse_local_conversation(self, text):
        normalized = self._normalize_answer(text)

        if any(
            token in normalized
            for token in ('지금뭐', '뭐하는중', '어디까지', '현재상태', '상태알려')
        ):
            return {
                'command': None,
                'feedbacks': [
                    self._feedback(
                        status='assistant_response',
                        raw_text=text,
                        reason='status_query',
                        message=self._context.robot_status_message(),
                    )
                ],
            }

        if any(
            token in normalized
            for token in ('고마워', '고맙', '고생했어', '수고했어', '안녕', '알겠어')
        ):
            return {
                'command': None,
                'feedbacks': [
                    self._feedback(
                        status='assistant_response',
                        raw_text=text,
                        reason='smalltalk',
                        message='네, 필요한 공구가 있으면 언제든 말해주세요.',
                    )
                ],
            }

        return None

    def _adjust_action(self, raw_text, action):
        normalized = self._normalize_answer(raw_text)
        if self._has_stop_intent(raw_text):
            return 'stop'
        if action == 'stop':
            return 'unknown'
        return_tokens = (
            '가져다가놔',
            '가져다가놓',
            '가져다놔',
            '가져다놓',
            '가져가',
            '가져가줘',
            '가져가세요',
            '갖다놔',
            '갖다놓',
            '치워',
            '치워줘',
            '정리',
            '제자리',
            '되돌려',
            '돌려놔',
            '돌려놓',
            '반납',
            '서랍에넣',
            '넣어',
            '넣어줘',
            '넣어주세요',
            '보관',
        )
        if any(token in normalized for token in return_tokens):
            return 'return'

        bring_tokens = (
            '가져다줘',
            '가져와',
            '가져',
            '갖다줘',
            '달라',
            '줘',
            '주세요',
            '집어줘',
        )
        if any(token in normalized for token in bring_tokens):
            return 'bring'

        return action

    @staticmethod
    def _has_stop_intent(raw_text):
        normalized = normalize_text(raw_text)
        return any(normalize_text(keyword) in normalized for keyword in STOP_KEYWORDS)

    def _has_previous_reference(self, raw_text):
        normalized = self._normalize_answer(raw_text)
        return any(
            token in normalized
            for token in (
                '아까',
                '방금',
                '좀전',
                '전에',
                '이전에',
                '같은거',
                '가져온거',
                '가져왔던거',
                '준거',
                '줬던거',
            )
        )

    def _has_repeat_reference(self, raw_text):
        normalized = self._normalize_answer(raw_text)
        has_previous = any(
            token in normalized
            for token in ('아까', '방금', '좀전', '전에', '이전', '같은거')
        )
        has_repeat = any(
            token in normalized
            for token in ('다시해', '다시', '또', '한번더', '하나더', '반복')
        )
        return has_previous and has_repeat

    def _has_explicit_tool_clue(self, raw_text):
        normalized = self._normalize_answer(raw_text)
        return any(
            token in normalized
            for token in (
                '드라이버',
                '드라이바',
                '나사',
                '돌리',
                '조이',
                '플라이어',
                '펜치',
                '뺀치',
                '집',
                '잡',
                '망치',
                '해머',
                '두드리',
                '못',
                '줄자',
                '길이',
                '치수',
                '측정',
            )
        )

    @staticmethod
    def _is_confident_named_command(action, tool_name, target_mode, confidence):
        return (
            action in ('bring', 'return')
            and tool_name != 'unknown'
            and target_mode == 'named'
            and confidence >= CONFIRMATION_CONFIDENCE_THRESHOLD
        )

    @staticmethod
    def _is_safe_deictic_return(action, target_mode, confidence):
        return (
            action == 'return'
            and target_mode == 'deictic'
            and confidence >= CONFIRMATION_CONFIDENCE_THRESHOLD
        )

    @staticmethod
    def _as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('true', '1', 'yes', 'y')
        return bool(value)

    def _adjust_ambiguous_command(
        self,
        raw_text,
        tool_name,
        target_mode,
        confidence,
    ):
        normalized = self._normalize_answer(raw_text)
        has_deictic_target = any(word in normalized for word in DEICTIC_WORDS)

        has_turn_expression = any(
            word in normalized
            for word in ('돌리', '조이', '풀어', '푸는', '풀')
        )
        has_impact_expression = any(
            word in normalized
            for word in ('박', '두드리', '치는거', '쳐', '때리', '망치')
        )
        has_measure_expression = any(
            word in normalized
            for word in ('길이', '치수', '재', '측정', '센치', 'cm')
        )
        has_grip_expression = any(
            word in normalized
            for word in ('집', '잡', '찝', '펜치', '플라이어', '니퍼')
        )
        has_named_tool = any(
            word in normalized
            for word in (
                '드라이버',
                '드라이바',
                '망치',
                '해머',
                '햄머',
                '플라이어',
                '펜치',
                '뺀치',
                '줄자',
                '테이프',
            )
        )
        has_tool_clue = any(
            (
                has_turn_expression,
                has_impact_expression,
                has_measure_expression,
                has_grip_expression,
                has_named_tool,
            )
        )

        if has_deictic_target and not has_tool_clue and tool_name == 'unknown':
            return 'unknown', 'deictic', confidence

        inferred_tool = self._infer_tool_from_function_words(
            has_turn_expression=has_turn_expression,
            has_impact_expression=has_impact_expression,
            has_measure_expression=has_measure_expression,
            has_grip_expression=has_grip_expression,
        )
        if inferred_tool is not None:
            if tool_name != inferred_tool:
                self._warn(
                    f'문장 기능 표현 기준으로 tool_name을 보정합니다: '
                    f'{tool_name} -> {inferred_tool}'
                )
            return inferred_tool, 'named', confidence

        if has_tool_clue and tool_name != 'unknown':
            return tool_name, 'named', confidence

        if target_mode == 'unknown':
            target_mode = 'named' if tool_name != 'unknown' else 'unknown'

        if (
            tool_name == 'hammer'
            and has_turn_expression
            and not has_impact_expression
        ):
            self._warn('돌리는/조이는 표현을 hammer가 아닌 screwdriver로 보정합니다.')
            confidence = min(confidence, max(0.0, self._min_confidence - 0.05))
            return 'screwdriver', 'named', confidence

        return tool_name, target_mode, confidence

    @staticmethod
    def _infer_tool_from_function_words(
        has_turn_expression,
        has_impact_expression,
        has_measure_expression,
        has_grip_expression,
    ):
        if has_impact_expression:
            return 'hammer'
        if has_measure_expression:
            return 'tape_measure'
        if has_grip_expression:
            return 'pliers'
        if has_turn_expression:
            return 'screwdriver'
        return None

    def _effective_command_text(self, raw_text):
        normalized = raw_text or ''
        if '말고' in normalized:
            return normalized.rsplit('말고', 1)[-1].strip()
        return normalized

    def _contains_unsupported_tool(self, raw_text):
        normalized = self._normalize_answer(raw_text)
        return any(
            token in normalized
            for token in ('드릴', '렌치', '스패너', '스페너')
        )

    def _resolve_deterministic_tool(self, raw_text):
        tool_name, match_method, match_score, _ = find_tool(raw_text)
        if match_method == 'alias':
            return tool_name
        if match_method == 'fuzzy' and match_score >= LOCAL_FUZZY_ACCEPT_THRESHOLD:
            return tool_name
        return ''

    def _info(self, message):
        if self._logger is not None:
            self._logger('info', message)

    def _warn(self, message):
        if self._logger is not None:
            self._logger('warn', message)

    def _error(self, message):
        if self._logger is not None:
            self._logger('error', message)
