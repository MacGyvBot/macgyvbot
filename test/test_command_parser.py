from macgyvbot.voice_command.command_parser import find_action


def test_find_action_return_keywords():
    assert find_action("드라이버 반납해") == "return"
    assert find_action("플라이어 서랍에 넣어줘") == "return"
    assert find_action("망치 제자리에 둬") == "return"


def test_find_action_bring_still_works():
    assert find_action("드라이버 가져와") == "bring"
