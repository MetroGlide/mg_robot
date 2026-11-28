"""状態ランプの意味の表 (config/activity_lamps.yaml) の読み込みと検査のテスト"""
import os

import pytest

from mg_waypoint_navigation.waypoint_sequencer import activity as act
from mg_waypoint_navigation.waypoint_sequencer.activity_lamps import (
    MOTIONS,
    RESPONSES,
    load_lamp_table,
    parse_lamp_table,
)

TABLE_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'activity_lamps.yaml')


def lamps(**entries):
    return {'lamps': {code: dict(entry) for code, entry in entries.items()}}


class TestShippedTable:
    def test_covers_every_activity_code(self):
        table = load_lamp_table(TABLE_PATH, act.PRIORITY)
        assert set(table) == set(act.PRIORITY)

    def test_every_entry_has_a_description(self):
        for code, meaning in load_lamp_table(TABLE_PATH, act.PRIORITY).items():
            assert meaning.description, code

    def test_red_is_only_for_stops_needing_a_person(self):
        table = load_lamp_table(TABLE_PATH, act.PRIORITY)
        acts = sorted(code for code, m in table.items() if m.response == 'act')
        assert acts == [act.ERROR, act.NO_PROGRESS, act.SENSOR_TIMEOUT_STOP]

    def test_start_is_only_for_start_waits(self):
        # MMI の LED1 の橙の点滅 (trigger_acceptable) と同じ条件の活動だけ
        table = load_lamp_table(TABLE_PATH, act.PRIORITY)
        starts = sorted(code for code, m in table.items() if m.response == 'start')
        assert starts == [act.READY_TO_START, act.TRIGGER_WAIT]

    def test_moving_activities_are_not_stops(self):
        table = load_lamp_table(TABLE_PATH, act.PRIORITY)
        for code in (act.NAVIGATING, act.MANUAL_GOAL, act.EXTERNAL_GOAL,
                     act.LOCALIZATION_SLOWDOWN, act.OBSTACLE_SLOWDOWN):
            assert table[code].motion == 'moving', code


class TestParse:
    def test_values_match_the_message_constants(self):
        assert RESPONSES == {'normal', 'watch', 'act', 'operator', 'start', 'none'}
        assert MOTIONS == {'moving', 'waiting', 'handling'}

    def test_reads_entries(self):
        table = parse_lamp_table(
            lamps(a={'response': 'normal', 'motion': 'moving', 'description': 'A'}), ['a'])
        assert (table['a'].response, table['a'].motion, table['a'].description) == \
            ('normal', 'moving', 'A')

    @pytest.mark.parametrize('raw', [
        None,
        {'lamps': []},
        lamps(a={'response': 'unknown', 'motion': 'moving'}),
        lamps(a={'response': 'normal', 'motion': 'blink'}),
    ])
    def test_rejects_invalid(self, raw):
        with pytest.raises(ValueError):
            parse_lamp_table(raw, ['a'])

    def test_rejects_missing_and_extra_codes(self):
        entry = {'response': 'normal', 'motion': 'moving'}
        with pytest.raises(ValueError):
            parse_lamp_table(lamps(a=entry), ['a', 'b'])
        with pytest.raises(ValueError):
            parse_lamp_table(lamps(a=entry, c=entry), ['a'])
