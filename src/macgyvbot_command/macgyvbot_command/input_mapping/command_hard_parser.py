"""Lightweight command parsing helpers for macgyvbot.

이 파일은 LLM 전에 먼저 시도하는 빠른 해석 레이어다.
- alias dictionary: 명확한 공구 별칭 매칭
- fuzzy matching: STT 오인식 보정
- action keyword: bring/return/release 분류

LLM은 이 파서가 공구를 확정하지 못했을 때 fallback으로 사용한다.
"""

from difflib import SequenceMatcher

from macgyvbot_command.input_mapping.command_vocabulary import (
    BRING_KEYWORDS,
    CANCEL_KEYWORDS,
    EXIT_KEYWORDS,
    HOME_KEYWORDS,
    RELEASE_KEYWORDS,
    RESUME_KEYWORDS,
    RETURN_KEYWORDS,
    STOP_KEYWORDS,
    TOOL_KEYWORDS,
)

FUZZY_MATCH_THRESHOLD = 0.50
MIN_FUZZY_KEYWORD_LENGTH = 3
SHORT_CONTROL_MAX_LENGTH = 14

_SHORT_CONTROL_KEYWORDS = (
    ('pause', STOP_KEYWORDS),
    ('resume', RESUME_KEYWORDS),
    ('cancel', CANCEL_KEYWORDS),
    ('home', HOME_KEYWORDS),
)
_SAFE_CONTROL_SUFFIXES = (
    '',
    '요',
    '해',
    '해요',
    '해줘',
    '해주세요',
    '하자',
    '하라',
    '줘',
    '주세요',
    '가',
    '가요',
    '가줘',
    '가주세요',
    '로가',
    '로가요',
    '로가줘',
    '로가주세요',
    '이동',
    '이동해',
    '이동해줘',
    '돌아가',
    '돌아가줘',
)
_NEGATED_CONTROL_TOKENS = (
    '하지마',
    '하지말',
    '멈추지마',
    '멈추지말',
    '정지하지마',
    '정지하지말',
    '중지하지마',
    '중지하지말',
    '복귀하지마',
    '복귀하지말',
)


def normalize_text(text):
    return (text or '').lower().replace(' ', '')


def iter_tool_aliases():
    for tool_name, keywords in TOOL_KEYWORDS.items():
        for keyword in keywords:
            yield tool_name, keyword


def similarity(left, right):
    return SequenceMatcher(None, left, right).ratio()


def find_tool_by_alias(text):
    normalized = normalize_text(text)
    for tool_name, keyword in iter_tool_aliases():
        if normalize_text(keyword) in normalized:
            return tool_name
    return ''


def find_tool_by_fuzzy(text):
    normalized = normalize_text(text)
    if not normalized:
        return '', 0.0, ''

    best_tool = ''
    best_score = 0.0
    best_keyword = ''

    for tool_name, keyword in iter_tool_aliases():
        normalized_keyword = normalize_text(keyword)
        if len(normalized_keyword) < MIN_FUZZY_KEYWORD_LENGTH:
            continue

        score = similarity(normalized, normalized_keyword)

        if len(normalized) > len(normalized_keyword):
            max_start = len(normalized) - len(normalized_keyword) + 1
            for start in range(0, max_start):
                part = normalized[start:start + len(normalized_keyword)]
                score = max(score, similarity(part, normalized_keyword))

        if score > best_score:
            best_tool = tool_name
            best_score = score
            best_keyword = keyword

    if best_score < FUZZY_MATCH_THRESHOLD:
        return '', best_score, best_keyword

    return best_tool, best_score, best_keyword


def find_tool(text):
    tool_name = find_tool_by_alias(text)
    if tool_name:
        return tool_name, 'alias', 1.0, ''

    tool_name, score, keyword = find_tool_by_fuzzy(text)
    if tool_name:
        return tool_name, 'fuzzy', score, keyword

    return '', 'none', score, keyword


def find_action(text):
    normalized = normalize_text(text)

    for keyword in STOP_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'pause'

    for keyword in RESUME_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'resume'

    for keyword in CANCEL_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'cancel'

    for keyword in EXIT_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'exit'

    for keyword in HOME_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'home'

    # "가져다가 놔", "정리해"는 사용자에게 전달이 아니라 보관 위치로 반납하는 명령이다.
    for keyword in RETURN_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'return'

    for keyword in RELEASE_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'release'

    for keyword in BRING_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'bring'

    return 'unknown'


def find_short_control_action(text):
    """Return a safe immediate control action for short utterances.

    This intentionally matches only compact, explicit control phrases so
    emergency-style commands such as "멈춰" do not wait for the LLM path, while
    longer or negated sentences still go through the full parser guards.
    """
    normalized = normalize_text(text)
    if not normalized or len(normalized) > SHORT_CONTROL_MAX_LENGTH:
        return ''

    if any(token in normalized for token in _NEGATED_CONTROL_TOKENS):
        return ''

    for action, keywords in _SHORT_CONTROL_KEYWORDS:
        if _matches_control_keyword(normalized, keywords):
            return action

    return ''


def _matches_control_keyword(normalized_text, keywords):
    for keyword in keywords:
        normalized_keyword = normalize_text(keyword)
        if not normalized_keyword:
            continue
        if normalized_text == normalized_keyword:
            return True
        if not normalized_text.startswith(normalized_keyword):
            continue
        suffix = normalized_text[len(normalized_keyword):]
        if suffix in _SAFE_CONTROL_SUFFIXES:
            return True
    return False
