"""Lightweight command parsing helpers for macgyvbot.

이 파일은 LLM 전에 먼저 시도하는 빠른 해석 레이어다.
- alias dictionary: 명확한 공구 별칭 매칭
- fuzzy matching: STT 오인식 보정
- action keyword: bring/release 분류

LLM은 이 파서가 공구를 확정하지 못했을 때 fallback으로 사용한다.
"""

from difflib import SequenceMatcher


TOOL_KEYWORDS = {
    'screwdriver': [
        '드라이버',
        '드라이바',
        '스크루드라이버',
        '십자드라이버',
        '십자 드라이버',
        '일자드라이버',
        '일자 드라이버',
        '나사드라이버',
        '나사 드라이버',
    ],
    'pliers': [
        '플라이어',
        '플라이어스',
        '펜치',
        '뺀치',
        '니퍼',
        '집게공구',
        '집게 공구',
    ],
    'hammer': [
        '망치',
        '해머',
        '햄머',
        '장도리',
        '고무망치',
        '고무 망치',
    ],
    'tape_measure': [
        '줄자',
        '테이프메저',
        '테이프 메저',
        '테이프자',
        '테이프 자',
        '측정줄자',
        '측정 줄자',
    ],
}

FUZZY_MATCH_THRESHOLD = 0.50

BRING_KEYWORDS = [
    '가져다',
    '가져와',
    '가져',
    '갖다',
    '달라',
    '줘',
    '주세요',
    '집어',
    '잡아',
]

RELEASE_KEYWORDS = [
    '놔',
    '놓아',
    '놓으',
    '내려놔',
    '내려놓',
    '두고',
]


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
        score = similarity(normalized, normalized_keyword)

        if len(normalized) > len(normalized_keyword):
            for start in range(0, len(normalized) - len(normalized_keyword) + 1):
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

    # "가져다 놓아"는 사용자가 원하는 전달 동작이므로 bring을 우선한다.
    for keyword in BRING_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'bring'

    for keyword in RELEASE_KEYWORDS:
        if normalize_text(keyword) in normalized:
            return 'release'

    return 'bring'
