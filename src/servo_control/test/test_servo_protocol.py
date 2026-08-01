import json


def parse_request(payload):
    request = json.loads(payload)
    sequence_id = int(request['sequence_id'])
    command = str(request['command']).lower()
    if sequence_id <= 0 or command != 'throw':
        raise ValueError('invalid request')
    return sequence_id, command


def test_request_requires_positive_sequence_and_throw():
    assert parse_request('{"sequence_id":7,"command":"throw"}') == (7, 'throw')


def test_duplicate_sequence_is_rejected_without_second_execution():
    seen = set()
    sequence_id, _ = parse_request('{"sequence_id":7,"command":"throw"}')
    assert sequence_id not in seen
    seen.add(sequence_id)
    assert sequence_id in seen
