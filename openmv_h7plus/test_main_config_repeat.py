import ast
from pathlib import Path


SOURCE = Path(__file__).with_name('main.py').read_text(encoding='utf-8')
MODULE = ast.parse(SOURCE)
FUNCTIONS = {}
for node in MODULE.body:
    if isinstance(node, ast.FunctionDef) and node.name == '_maybe_repeat_config':
        function_module = ast.Module(body=[node], type_ignores=[])
        code = compile(function_module, 'main.py', 'exec')
        namespace = {}
        exec(code, namespace)
        FUNCTIONS[node.name] = namespace[node.name]


class FakeDetector:
    def __init__(self, answers):
        self.answers = list(answers)
        self.calls = []

    def should_repeat_config(self, now_ms):
        self.calls.append(now_ms)
        if self.answers:
            return self.answers.pop(0)
        return False


class FakeProtocol:
    def __init__(self, fail=False):
        self.fail = fail
        self.calls = 0

    def send_config(self):
        self.calls += 1
        if self.fail:
            raise RuntimeError('send failed')
        return True


maybe_repeat_config = FUNCTIONS['_maybe_repeat_config']


def test_repeat_config_sends_once_when_period_reached():
    detector = FakeDetector([False, True, False])
    protocol = FakeProtocol()
    assert maybe_repeat_config(detector, protocol, 1000) is False
    assert maybe_repeat_config(detector, protocol, 10000) is True
    assert maybe_repeat_config(detector, protocol, 10001) is False
    assert protocol.calls == 1


def test_repeat_config_does_not_send_before_period():
    detector = FakeDetector([False, False, False])
    protocol = FakeProtocol()
    assert maybe_repeat_config(detector, protocol, 1000) is False
    assert maybe_repeat_config(detector, protocol, 5000) is False
    assert maybe_repeat_config(detector, protocol, 9999) is False
    assert protocol.calls == 0


def test_repeat_config_failure_does_not_escape_loop():
    detector = FakeDetector([True, True, False])
    protocol = FakeProtocol(fail=True)
    assert maybe_repeat_config(detector, protocol, 10000) is False
    assert maybe_repeat_config(detector, protocol, 20000) is False
    assert maybe_repeat_config(detector, protocol, 20001) is False
    assert protocol.calls == 2
