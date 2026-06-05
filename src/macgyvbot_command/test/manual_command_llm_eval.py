#!/usr/bin/env python3
"""
Manual live evaluation for CommandLlmParser.

This script intentionally calls a local Ollama server and is not meant to be a
regular CI test.  It helps check natural Korean command handling without ROS:

    python3 test/manual_command_llm_eval.py --model gemma3:1b
"""

import argparse
import json

from macgyvbot_command.input_mapping.command_context import CommandContext
from macgyvbot_command.input_mapping.command_llm_parser import (
    CommandLlmParser,
)


CASES = [
    ('드라이버 가져다줘', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('드라이버 줘', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('나사 돌리는 거 가져와', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('나사 조이는 거 줘', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('돌려서 쓰는 공구 가져다줘', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('망치 가져다줘', {'command': {'tool_name': 'hammer', 'action': 'bring'}}),
    ('못 박는 거 가져와', {'command': {'tool_name': 'hammer', 'action': 'bring'}}),
    ('두드리는 거 줘', {'command': {'tool_name': 'hammer', 'action': 'bring'}}),
    ('플라이어 가져와', {'command': {'tool_name': 'pliers', 'action': 'bring'}}),
    ('펜치 가져다줘', {'command': {'tool_name': 'pliers', 'action': 'bring'}}),
    ('집는 공구 줘', {'command': {'tool_name': 'pliers', 'action': 'bring'}}),
    ('줄자 가져와', {'command': {'tool_name': 'tape_measure', 'action': 'bring'}}),
    ('길이 재는 거 줘', {'command': {'tool_name': 'tape_measure', 'action': 'bring'}}),
    ('치수 재는 공구 가져와', {'command': {'tool_name': 'tape_measure', 'action': 'bring'}}),
    ('렌치 가져다줘', {'command': {'tool_name': 'wrench', 'action': 'bring'}}),
    ('스패너 줘', {'command': {'tool_name': 'wrench', 'action': 'bring'}}),
    ('볼트 조이는 거 가져와', {'command': {'tool_name': 'wrench', 'action': 'bring'}}),
    ('너트 푸는 공구 줘', {'command': {'tool_name': 'wrench', 'action': 'bring'}}),
    ('드라이버 정리해', {'command': {'tool_name': 'screwdriver', 'action': 'return'}}),
    ('드라이버 가져다놔', {'command': {'tool_name': 'screwdriver', 'action': 'return'}}),
    ('나사 돌리는 거 정리해', {'command': {'tool_name': 'screwdriver', 'action': 'return'}}),
    ('망치 정리해', {'command': {'tool_name': 'hammer', 'action': 'return'}}),
    ('못 두드리는 거 가져가', {'command': {'tool_name': 'hammer', 'action': 'return'}}),
    ('플라이어 제자리에 둬', {'command': {'tool_name': 'pliers', 'action': 'return'}}),
    ('줄자 서랍에 넣어줘', {'command': {'tool_name': 'tape_measure', 'action': 'return'}}),
    ('렌치 정리해', {'command': {'tool_name': 'wrench', 'action': 'return'}}),
    ('스패너 서랍에 넣어줘', {'command': {'tool_name': 'wrench', 'action': 'return'}}),
    (
        '이거 정리해',
        {
            'command': {
                'tool_name': 'unknown',
                'action': 'return',
                'target_mode': 'deictic',
            },
        },
    ),
    (
        '저거 가져다놔',
        {
            'command': {
                'tool_name': 'unknown',
                'action': 'return',
                'target_mode': 'deictic',
            },
        },
    ),
    ('그거 가져와', {'no_command_reason': 'deictic_bring_not_supported'}),
    ('이거 가져다줘', {'no_command_reason': 'deictic_bring_not_supported'}),
    ('아까 가져온 거 정리해', {'command': {'tool_name': 'screwdriver', 'action': 'return'}}),
    ('방금 준 거 다시 정리해', {'command': {'tool_name': 'screwdriver', 'action': 'return'}}),
    ('아까 그거 가져와', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('같은 거 하나 더 가져와', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('아까 했던 거 다시 해', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('좀 전에 가져온 공구 반납해', {'command': {'tool_name': 'screwdriver', 'action': 'return'}}),
    ('지금 뭐 하는 중이야?', {'feedback_status': 'assistant_response'}),
    ('어디까지 했어?', {'feedback_status': 'assistant_response'}),
    ('현재 상태 알려줘', {'feedback_status': 'assistant_response'}),
    ('고마워', {'feedback_status': 'assistant_response'}),
    ('고생했어', {'feedback_status': 'assistant_response'}),
    ('안녕', {'feedback_status': 'assistant_response'}),
    ('알겠어', {'feedback_status': 'assistant_response'}),
    ('멈춰', {'command': {'action': 'pause'}}),
    ('정지', {'command': {'action': 'pause'}}),
    ('작업 중단해', {'command': {'action': 'pause'}}),
    ('스탑', {'command': {'action': 'pause'}}),
    ('재개', {'command': {'action': 'resume'}}),
    ('다시 시작해', {'command': {'action': 'resume'}}),
    ('계속해', {'command': {'action': 'resume'}}),
    ('취소', {'command': {'action': 'cancel'}}),
    ('이번 작업 취소해', {'command': {'action': 'cancel'}}),
    ('작업 취소', {'command': {'action': 'cancel'}}),
    ('홈위치로 가', {'command': {'action': 'home'}}),
    ('복귀해', {'command': {'action': 'home'}}),
    ('원위치로 돌아가', {'command': {'action': 'home'}}),
    ('초기 위치로 이동해', {'command': {'action': 'home'}}),
    ('종료', {'command': {'action': 'exit'}}),
    ('끝내줘', {'command': {'action': 'exit'}}),
    ('프로그램 꺼줘', {'command': {'action': 'exit'}}),
    ('드릴 가져와', {'no_command': True}),
    ('커피 가져와', {'no_command': True}),
    ('잘못 말했어 망치 말고 줄자 줘', {'command': {'tool_name': 'tape_measure', 'action': 'bring'}}),
    ('드라이버 말고 망치 가져와', {'command': {'tool_name': 'hammer', 'action': 'bring'}}),
    ('플라이어 말고 줄자 정리해', {'command': {'tool_name': 'tape_measure', 'action': 'return'}}),
    ('나사 조이는 거 말고 못 박는 거 가져와', {'command': {'tool_name': 'hammer', 'action': 'bring'}}),
    ('십자 드라이버 부탁해', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('일자드라이버 좀 가져와', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('나사 풀 공구 주세요', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('장도리 가져와', {'command': {'tool_name': 'hammer', 'action': 'bring'}}),
    ('고무 망치 주세요', {'command': {'tool_name': 'hammer', 'action': 'bring'}}),
    ('뺀치 가져다 줄래', {'command': {'tool_name': 'pliers', 'action': 'bring'}}),
    ('니퍼를 가져다줘', {'command': {'tool_name': 'pliers', 'action': 'bring'}}),
    ('측정 줄자 부탁해', {'command': {'tool_name': 'tape_measure', 'action': 'bring'}}),
    ('테이프 자 가져와', {'command': {'tool_name': 'tape_measure', 'action': 'bring'}}),
    ('몽키스패너 가져와', {'command': {'tool_name': 'wrench', 'action': 'bring'}}),
    ('육각 렌치 가져다줘', {'command': {'tool_name': 'wrench', 'action': 'bring'}}),
    ('망치 말고 드라이버 가져와', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('십자 드라이버 반납해', {'command': {'tool_name': 'screwdriver', 'action': 'return'}}),
    ('장도리 치워줘', {'command': {'tool_name': 'hammer', 'action': 'return'}}),
    ('펜치 보관해', {'command': {'tool_name': 'pliers', 'action': 'return'}}),
    ('테이프 자 서랍에 넣어', {'command': {'tool_name': 'tape_measure', 'action': 'return'}}),
    ('볼트 푸는 거 보관해', {'command': {'tool_name': 'wrench', 'action': 'return'}}),
    (
        '이 공구 제자리에 놓아줘',
        {'command': {'tool_name': 'unknown', 'action': 'return', 'target_mode': 'deictic'}},
    ),
    (
        '저 공구를 보관해줘',
        {'command': {'tool_name': 'unknown', 'action': 'return', 'target_mode': 'deictic'}},
    ),
    ('놔', {'command': {'action': 'release'}}),
    ('놓으세요', {'command': {'action': 'release'}}),
    ('그리퍼 열어', {'command': {'action': 'release'}}),
    ('집게 오픈해', {'command': {'action': 'release'}}),
    ('당장 멈춰', {'command': {'action': 'pause'}}),
    ('작업을 중지해', {'command': {'action': 'pause'}}),
    ('pause', {'command': {'action': 'pause'}}),
    ('이어 해', {'command': {'action': 'resume'}}),
    ('resume', {'command': {'action': 'resume'}}),
    ('계속 진행해', {'command': {'action': 'resume'}}),
    ('cancel', {'command': {'action': 'cancel'}}),
    ('홈으로 돌아가', {'command': {'action': 'home'}}),
    ('초기 자세로 돌아가', {'command': {'action': 'home'}}),
    ('기본 위치로 가줘', {'command': {'action': 'home'}}),
    ('home으로 이동해', {'command': {'action': 'home'}}),
    ('앱 꺼줘', {'command': {'action': 'exit'}}),
    ('quit', {'command': {'action': 'exit'}}),
    ('프로그램 닫아줘', {'command': {'action': 'exit'}}),
    ('카메라 꺼줘', {'no_command': True}),
    ('종료하지 마', {'feedback_status': 'assistant_response'}),
    ('종료됐어?', {'feedback_status': 'assistant_response'}),
    ('앱 꺼주지 마', {'feedback_status': 'assistant_response'}),
    ('복귀하지 마', {'feedback_status': 'assistant_response'}),
    ('지금 홈위치야?', {'feedback_status': 'assistant_response'}),
    ('복귀 중이야?', {'feedback_status': 'assistant_response'}),
    ('정지하지 마', {'feedback_status': 'assistant_response'}),
    ('현재 정지 상태야?', {'feedback_status': 'assistant_response'}),
    ('재개하지 마', {'feedback_status': 'assistant_response'}),
    ('지금 재개된 상태야?', {'feedback_status': 'assistant_response'}),
    ('취소하지 마', {'feedback_status': 'assistant_response'}),
    ('취소됐어?', {'feedback_status': 'assistant_response'}),
    ('놓지 마', {'feedback_status': 'assistant_response'}),
    ('그리퍼 열지 마', {'feedback_status': 'assistant_response'}),
    ('지금 홈 복귀가 완료됐어?', {'feedback_status': 'assistant_response'}),
    ('종료 요청 처리 중이야?', {'feedback_status': 'assistant_response'}),
    ('정지 이후에는 어떻게 돼?', {'feedback_status': 'assistant_response'}),
    ('드라비어 가져다줘', {'command': {'tool_name': 'screwdriver', 'action': 'bring'}}),
    ('플라이어어 가져다줘', {'command': {'tool_name': 'pliers', 'action': 'bring'}}),
]


def make_parser(args):
    context = CommandContext()
    parser = CommandLlmParser(
        ollama_url=args.ollama_url,
        model=args.model,
        timeout_sec=args.timeout_sec,
        min_confidence=args.min_confidence,
        use_local_parser=True,
        use_llm_fallback=True,
        parser_mode=args.parser_mode,
        context=context,
        logger=None,
    )

    # Deterministic seed for "아까/방금/같은 거" style context tests.
    context.record_accepted_command(
        {
            'tool_name': 'screwdriver',
            'action': 'bring',
            'target_mode': 'named',
            'raw_text': '드라이버 가져다줘',
        }
    )
    context.update_robot_status(
        {
            'status': 'done',
            'tool_name': 'screwdriver',
            'message': 'screwdriver 전달 완료',
        }
    )
    return parser


def first_feedback(result):
    feedbacks = result.get('feedbacks') or []
    return feedbacks[0] if feedbacks else {}


def command_matches(command, expected):
    for key, value in expected.items():
        if command.get(key) != value:
            return False
    return True


def evaluate(result, expected):
    command = result.get('command')
    feedback = first_feedback(result)

    if 'command' in expected:
        return command is not None and command_matches(command, expected['command'])

    if expected.get('no_command'):
        return command is None

    if 'no_command_reason' in expected:
        return (
            command is None
            and feedback.get('reason') == expected['no_command_reason']
        )

    if 'feedback_status' in expected:
        return (
            command is None
            and feedback.get('status') == expected['feedback_status']
        )

    return False


def compact_result(result):
    command = result.get('command')
    feedback = first_feedback(result)
    if command is not None:
        return {
            'command': {
                key: command.get(key)
                for key in (
                    'tool_name',
                    'action',
                    'target_mode',
                    'match_method',
                    'confidence',
                    'context_used',
                )
            }
        }
    return {
        'feedback': {
            key: feedback.get(key)
            for key in ('status', 'reason', 'message')
        }
    }


def main():
    parser_arg = argparse.ArgumentParser()
    parser_arg.add_argument('--model', default='gemma3:1b')
    parser_arg.add_argument(
        '--ollama-url',
        default='http://127.0.0.1:11434/api/generate',
    )
    parser_arg.add_argument('--timeout-sec', type=float, default=30.0)
    parser_arg.add_argument('--min-confidence', type=float, default=0.65)
    parser_arg.add_argument('--parser-mode', default='llm_primary')
    parser_arg.add_argument('--limit', type=int, default=0)
    args = parser_arg.parse_args()

    cases = CASES[:args.limit] if args.limit > 0 else CASES

    passed = 0
    for index, (text, expected) in enumerate(cases, 1):
        parser = make_parser(args)
        result = parser.interpret(text)
        ok = evaluate(result, expected)
        passed += int(ok)
        status = 'PASS' if ok else 'FAIL'
        print(f'[{index:02d}] {status} {text}')
        if not ok:
            print('  expected:', expected)
            print('  actual  :', json.dumps(compact_result(result), ensure_ascii=False))

    total = len(cases)
    print(f'\nsummary: {passed}/{total} passed')
    return 0 if passed == total else 1


if __name__ == '__main__':
    raise SystemExit(main())
