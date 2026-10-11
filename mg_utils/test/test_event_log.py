import pytest

from mg_utils.event_log import EventLog, format_event, format_value, parse_event


class _Logger:
    def __init__(self):
        self.records = []

    def info(self, msg):
        self.records.append(('info', msg))

    def warning(self, msg):
        self.records.append(('warning', msg))

    def error(self, msg):
        self.records.append(('error', msg))


def test_format_simple_event():
    assert format_event('sequencer.state_changed', from_='IDLE', to='NAVIGATING', index=3) == \
        'evt=sequencer.state_changed from=IDLE to=NAVIGATING index=3'


def test_format_value_types():
    assert format_value(True) == 'true'
    assert format_value(False) == 'false'
    assert format_value(1.5) == '1.5'
    assert format_value(0.1 + 0.2) == '0.3'
    assert format_value(7) == '7'


@pytest.mark.parametrize('value, expected', [
    ('', '""'),
    ('read timeout', '"read timeout"'),
    ('a=b', '"a=b"'),
    ('say "hi"', '"say \\"hi\\""'),
    ('back\\slash', '"back\\\\slash"'),
    ('line1\nline2', '"line1\\nline2"'),
])
def test_format_value_quotes_and_escapes(value, expected):
    assert format_value(value) == expected


def test_none_fields_are_omitted():
    assert format_event('node.started', port=None, name='motor') == 'evt=node.started name=motor'


@pytest.mark.parametrize('name', ['started', 'Node.started', 'node.Started', 'node.', '.started', 'a.b.c'])
def test_invalid_event_name(name):
    with pytest.raises(ValueError):
        format_event(name)


@pytest.mark.parametrize('key', ['Index', 'a-b', '1a', 'evt'])
def test_invalid_key(key):
    with pytest.raises(ValueError):
        format_event('node.started', **{key: 1})


def test_parse_roundtrip():
    fields = {
        'reason': 'read timeout',
        'quote': 'say "hi"',
        'path': 'a\\b',
        'multi': 'x\ny',
        'empty': '',
        'eq': 'k=v',
        'index': 3,
        'ok': True,
    }
    body = format_event('driver.disconnected', **fields)
    name, parsed = parse_event(body)
    assert name == 'driver.disconnected'
    assert parsed == {k: format_value(v) if not isinstance(v, str) else v for k, v in fields.items()}


def test_parse_ignores_non_events():
    assert parse_event('Serial device connected') is None
    assert parse_event('  evt=node.started') is None
    assert parse_event('FSM: IDLE -> NAVIGATING') is None


def test_parse_event_without_fields():
    assert parse_event('evt=node.started') == ('node.started', {})


def test_event_log_levels():
    logger = _Logger()
    log = EventLog(logger)
    log.info('driver.connected', device='motor')
    log.warn('driver.reconnect_attempt', count=2)
    log.error('driver.disconnected', reason='read timeout')
    assert logger.records == [
        ('info', 'evt=driver.connected device=motor'),
        ('warning', 'evt=driver.reconnect_attempt count=2'),
        ('error', 'evt=driver.disconnected reason="read timeout"'),
    ]
