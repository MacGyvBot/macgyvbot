"""Lightweight command parsing helpers for macgyvbot.

이 파일은 LLM 전에 먼저 시도하는 빠른 해석 레이어다.
- alias dictionary: 명확한 공구 별칭 매칭
- fuzzy matching: STT 오인식 보정
- action keyword: bring/return/release 분류

LLM은 이 파서가 공구를 확정하지 못했을 때 fallback으로 사용한다.
"""

from difflib import SequenceMatcher

from macgyvbot.command_input.input_mapping.command_vocabulary import (
    BRING_KEYWORDS,
    RELEASE_KEYWORDS,
    RETURN_KEYWORDS,
    STOP_KEYWORDS,
    TOOL_KEYWORDS,
)

FUZZY_MATCH_THRESHOLD = 0.50
MIN_FUZZY_KEYWORD_LENGTH = 3

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
            return 'stop'

    # "가져다가 놔", "정리해"는 사용자에게 전달이 아니라 보관 위치로 반납하는 명령이다.
    for keyword in RETURN_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'return'

    for keyword in BRING_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'bring'

    for keyword in RELEASE_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'release'

    return 'unknown'
