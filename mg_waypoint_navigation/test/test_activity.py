"""「いまの活動」の合成 (ROS 非依存) のテスト"""
import dataclasses

import pytest

from mg_waypoint_navigation.waypoint_sequencer import activity as act
from mg_waypoint_navigation.waypoint_sequencer.activity import (
    Activity,
    ActivityDebouncer,
    ActivityInputs,
    ActivitySettings,
    CollisionInput,
    LocalizationInput,
    compose_activity,
)
from mg_waypoint_navigation.waypoint_sequencer.states import SequencerState

STALL_WARN_SEC = 30.0
SETTINGS = ActivitySettings(stall_warn_sec=STALL_WARN_SEC)


def make_inputs(**overrides) -> ActivityInputs:
    base = ActivityInputs(
        state=SequencerState.NAVIGATING,
        current_index=3,
        total_waypoints=10,
        countdown_ms_remaining=0,
        pause_requesters=(),
        distance_remaining=12.3,
        current_action='',
        waiting_trigger=False,
        manual_goal_active=False,
        external_goal_active=False,
        collision=None,
        running_recovery_nodes=frozenset(),
        stalled_sec=0.0,
    )
    return dataclasses.replace(base, **overrides)


def compose(**overrides):
    return compose_activity(make_inputs(**overrides), SETTINGS)


class TestSequencerStates:
    def test_idle(self):
        result = compose(state=SequencerState.IDLE)
        assert result.code == act.IDLE

    def test_trigger_wait(self):
        result = compose(state=SequencerState.IDLE, waiting_trigger=True)
        assert result.code == act.TRIGGER_WAIT
        assert (result.index, result.total) == (3, 10)

    def test_departure_wait(self):
        result = compose(state=SequencerState.ON_STARTING, countdown_ms_remaining=2000)
        assert result.code == act.DEPARTURE_WAIT
        assert result.remaining == pytest.approx(2.0)

    def test_navigating(self):
        result = compose()
        assert result.code == act.NAVIGATING
        assert (result.index, result.total) == (3, 10)
        assert result.remaining == pytest.approx(12.3)

    def test_paused_with_requesters(self):
        result = compose(
            state=SequencerState.SUSPENDED, pause_requesters=('web', 'scenario'))
        assert result.code == act.PAUSED
        assert result.detail == 'web, scenario'

    def test_goal_reached(self):
        assert compose(state=SequencerState.GOAL_REACHED).code == act.GOAL_REACHED

    def test_error(self):
        result = compose(state=SequencerState.ERROR)
        assert result.code == act.ERROR


class TestArrivalActions:
    @pytest.mark.parametrize('action,text', [
        ('load_map', '地図切り替えを実行中'),
        ('wait', '待機中'),
        ('amcl_reset', '自己位置をリセット中'),
        ('set_navigation_mode', '走行モードを切り替え中'),
    ])
    def test_known_action(self, action, text):
        result = compose(state=SequencerState.ON_ARRIVING, current_action=action)
        assert result.code == act.ACTION
        assert result.text == text
        assert result.detail == action

    def test_unknown_action_keeps_type_in_detail(self):
        result = compose(state=SequencerState.ON_ARRIVING, current_action='custom')
        assert result.code == act.ACTION
        assert result.detail == 'custom'


class TestDriving:
    def test_obstacle_stop(self):
        result = compose(collision=CollisionInput(act.COLLISION_STOP, 'stop_front'))
        assert result.code == act.OBSTACLE_STOP
        assert result.detail == 'stop_front'

    def test_sensor_timeout_stop(self):
        result = compose(
            collision=CollisionInput(act.COLLISION_STOP, 'source_timeout:scan'))
        assert result.code == act.SENSOR_TIMEOUT_STOP
        assert result.detail == 'scan'

    def test_obstacle_slowdown(self):
        result = compose(collision=CollisionInput(act.COLLISION_SLOWDOWN, 'slow'))
        assert result.code == act.OBSTACLE_SLOWDOWN

    def test_approach_is_also_obstacle_slowdown(self):
        result = compose(collision=CollisionInput(act.COLLISION_APPROACH, 'approach'))
        assert result.code == act.OBSTACLE_SLOWDOWN
        assert result.detail == 'approach'

    def test_do_nothing_is_navigating(self):
        assert compose(collision=CollisionInput(0, '')).code == act.NAVIGATING

    def test_no_progress_boundary(self):
        assert compose(stalled_sec=STALL_WARN_SEC - 0.1).code == act.NAVIGATING
        assert compose(stalled_sec=STALL_WARN_SEC).code == act.NO_PROGRESS

    def test_stop_takes_priority_over_no_progress(self):
        result = compose(
            stalled_sec=100.0, collision=CollisionInput(act.COLLISION_STOP, 'p'))
        assert result.code == act.OBSTACLE_STOP

    def test_recovery_takes_priority_over_stop(self):
        result = compose(
            running_recovery_nodes=frozenset({'SpinLeftAndReplan'}),
            collision=CollisionInput(act.COLLISION_STOP, 'p'))
        assert result.code == act.RECOVERY
        assert result.text == 'リカバリー: 左へ回転'

    def test_manual_goal(self):
        result = compose(state=SequencerState.IDLE, manual_goal_active=True)
        assert result.code == act.MANUAL_GOAL
        assert (result.index, result.total) == (-1, -1)

    def test_manual_goal_in_error_state(self):
        result = compose(state=SequencerState.ERROR, manual_goal_active=True)
        assert result.code == act.MANUAL_GOAL

    def test_external_goal(self):
        result = compose(state=SequencerState.IDLE, external_goal_active=True)
        assert result.code == act.EXTERNAL_GOAL
        assert (result.index, result.total, result.remaining) == (-1, -1, -1.0)

    def test_external_goal_replaces_waiting_states(self):
        for state in (SequencerState.GOAL_REACHED, SequencerState.IDLE):
            assert compose(state=state, external_goal_active=True).code == act.EXTERNAL_GOAL
        result = compose(
            state=SequencerState.IDLE, waiting_trigger=True, external_goal_active=True)
        assert result.code == act.EXTERNAL_GOAL

    def test_sequencer_states_take_priority_over_external_goal(self):
        for state, code in (
                (SequencerState.SUSPENDED, act.PAUSED),
                (SequencerState.ON_STARTING, act.DEPARTURE_WAIT),
                (SequencerState.ON_ARRIVING, act.ACTION),
                (SequencerState.ERROR, act.ERROR)):
            assert compose(state=state, external_goal_active=True).code == code

    def test_external_goal_shows_obstacles_and_recovery(self):
        stop = compose(
            state=SequencerState.IDLE, external_goal_active=True,
            collision=CollisionInput(act.COLLISION_STOP, 'stop_front'))
        assert (stop.code, stop.detail) == (act.OBSTACLE_STOP, 'stop_front')
        assert (stop.index, stop.total) == (-1, -1)
        slowdown = compose(
            state=SequencerState.IDLE, external_goal_active=True,
            collision=CollisionInput(act.COLLISION_SLOWDOWN, 'slow'))
        assert slowdown.code == act.OBSTACLE_SLOWDOWN
        recovery = compose(
            state=SequencerState.IDLE, external_goal_active=True,
            running_recovery_nodes=frozenset({'BackUp'}))
        assert recovery.code == act.RECOVERY

    def test_detour_rejected_replaces_recovery(self):
        result = compose(
            rejected_detour=act.DetourInput(152.0, 45.0),
            running_recovery_nodes=frozenset({'BackUpAndReplan'}))
        assert result.code == act.DETOUR_REJECTED
        assert result.detail == '経路 152 m / 上限 45 m'
        assert (result.index, result.total) == (3, 10)

    def test_detour_rejected_is_not_shown_outside_recovery(self):
        # 棄却しても、今の経路で走れているときは、通常の表示のまま
        result = compose(rejected_detour=act.DetourInput(152.0, 45.0))
        assert result.code == act.NAVIGATING

    def test_detour_rejected_does_not_hide_stop_and_no_progress(self):
        stop = compose(
            rejected_detour=act.DetourInput(152.0, 45.0),
            collision=CollisionInput(act.COLLISION_STOP, 'p'))
        assert stop.code == act.OBSTACLE_STOP
        stalled = compose(rejected_detour=act.DetourInput(152.0, 45.0), stalled_sec=100.0)
        assert stalled.code == act.NO_PROGRESS

    def test_detour_rejected_on_manual_and_external_goal(self):
        nodes = frozenset({'Replan'})
        manual = compose(
            state=SequencerState.IDLE, manual_goal_active=True,
            running_recovery_nodes=nodes, rejected_detour=act.DetourInput(152.0, 45.0))
        assert (manual.code, manual.index, manual.total) == (act.DETOUR_REJECTED, -1, -1)
        external = compose(
            state=SequencerState.IDLE, external_goal_active=True,
            running_recovery_nodes=nodes, rejected_detour=act.DetourInput(152.0, 45.0))
        assert external.code == act.DETOUR_REJECTED

    def test_detour_rejected_is_ignored_in_sequencer_states(self):
        result = compose(
            state=SequencerState.SUSPENDED,
            running_recovery_nodes=frozenset({'Replan'}),
            rejected_detour=act.DetourInput(152.0, 45.0))
        assert result.code == act.PAUSED

    def test_paused_takes_priority_over_stop(self):
        result = compose(
            state=SequencerState.SUSPENDED,
            collision=CollisionInput(act.COLLISION_STOP, 'p'))
        assert result.code == act.PAUSED


class TestRecoveryText:
    @pytest.mark.parametrize('nodes,text', [
        ({'SpinLeftAndReplan', 'Spin'}, 'リカバリー: 左へ回転'),
        ({'SpinRightAndReplan'}, 'リカバリー: 右へ回転'),
        ({'BackUpAndReplan', 'BackUp'}, 'リカバリー: 後退'),
        ({'EscapeKeepout'}, 'リカバリー: 進入禁止の領域から脱出'),
        ({'Replan'}, 'リカバリー: 経路を引き直し'),
        ({'ClearAndReplan', 'Wait'}, 'リカバリー: 障害物の記録を消して引き直し'),
        ({'ClearLocalCostmapAndWait'}, 'リカバリー: 障害物の記録を消して待機'),
        ({'Spin'}, 'リカバリー: 回転'),
        ({'BackUp'}, 'リカバリー: 後退'),
        ({'Wait'}, 'リカバリー: 待機'),
    ])
    def test_text(self, nodes, text):
        result = compose(running_recovery_nodes=frozenset(nodes))
        assert result.code == act.RECOVERY
        assert result.text == text


def localization(**overrides) -> LocalizationInput:
    return dataclasses.replace(LocalizationInput(state=0), **overrides)


class TestPause:
    def test_pause_is_split_by_requester(self):
        for requesters, code in (
                (('web_ui',), act.PAUSED),
                (('scenario_test', 'unknown'), act.PAUSED),
                (('driver_watchdog',), act.WATCHDOG_PAUSE),
                (('localization',), act.LOCALIZATION_HOLD),
                (('web_ui', 'driver_watchdog'), act.WATCHDOG_PAUSE),
                (('driver_watchdog', 'localization'), act.LOCALIZATION_HOLD)):
            result = compose(state=SequencerState.SUSPENDED, pause_requesters=requesters)
            assert result.code == code, requesters
            assert result.detail == ', '.join(requesters)
            assert (result.index, result.total) == (3, 10)

    def test_requester_names_come_from_settings(self):
        settings = ActivitySettings(
            stall_warn_sec=STALL_WARN_SEC, localization_requesters=frozenset({'loc_x'}),
            watchdog_requesters=frozenset({'dog_x'}))
        inputs = make_inputs(state=SequencerState.SUSPENDED, pause_requesters=('loc_x',))
        assert compose_activity(inputs, settings).code == act.LOCALIZATION_HOLD
        inputs = make_inputs(state=SequencerState.SUSPENDED, pause_requesters=('localization',))
        assert compose_activity(inputs, settings).code == act.PAUSED

    def test_pause_hides_localization_and_obstacles(self):
        result = compose(
            state=SequencerState.SUSPENDED, pause_requesters=('localization',),
            localization=localization(state=2, speed_limit=0.3), localization_expected=True,
            collision=CollisionInput(act.COLLISION_STOP, 'p'))
        assert result.code == act.LOCALIZATION_HOLD


class TestWaitingToStart:
    def test_ready_to_start(self):
        result = compose(state=SequencerState.IDLE, trigger_acceptable=True)
        assert result.code == act.READY_TO_START
        assert (result.index, result.total) == (3, 10)

    def test_trigger_wait_takes_priority_over_ready_to_start(self):
        result = compose(state=SequencerState.IDLE, waiting_trigger=True, trigger_acceptable=True)
        assert result.code == act.TRIGGER_WAIT

    def test_not_acceptable_idle_is_idle(self):
        assert compose(state=SequencerState.IDLE, trigger_acceptable=False).code == act.IDLE

    def test_driving_replaces_waiting(self):
        for overrides in ({'manual_goal_active': True}, {'external_goal_active': True}):
            result = compose(
                state=SequencerState.IDLE, trigger_acceptable=True, waiting_trigger=True,
                **overrides)
            assert result.code in (act.MANUAL_GOAL, act.EXTERNAL_GOAL)

    def test_only_idle_waits_for_start(self):
        result = compose(state=SequencerState.GOAL_REACHED, trigger_acceptable=True)
        assert result.code == act.GOAL_REACHED


class TestLocalization:
    def test_slowdown(self):
        result = compose(
            localization=localization(state=1, situation='suspect', speed_limit=0.4),
            localization_expected=True)
        assert result.code == act.LOCALIZATION_SLOWDOWN
        assert result.detail == 'SUSPECT, suspect, 上限 0.40 m/s'
        assert (result.index, result.total) == (3, 10)
        assert result.remaining == pytest.approx(12.3)

    def test_slowdown_without_supervisor_state_name(self):
        # 状況の名前によらず、速度の上限があれば減速 (起動直後の下限値も含む)
        result = compose(
            localization=localization(situation='uninitialized', speed_limit=0.2),
            localization_expected=True)
        assert result.code == act.LOCALIZATION_SLOWDOWN

    @pytest.mark.parametrize('state', [2, 3, 4])
    def test_recovering_drive(self, state):
        result = compose(localization=localization(state=state), localization_expected=True)
        assert result.code == act.LOCALIZATION_RECOVERING_DRIVE

    def test_recovering_takes_priority_over_slowdown_and_no_lidar(self):
        result = compose(
            localization=localization(state=3, speed_limit=0.3, sensor_set='no_lidar'),
            localization_expected=True)
        assert result.code == act.LOCALIZATION_RECOVERING_DRIVE

    def test_no_lidar_drive(self):
        result = compose(
            localization=localization(sensor_set='no_lidar'), localization_expected=True)
        assert result.code == act.NO_LIDAR_DRIVE

    def test_slowdown_takes_priority_over_no_lidar(self):
        result = compose(
            localization=localization(sensor_set='no_lidar', speed_limit=0.3),
            localization_expected=True)
        assert result.code == act.LOCALIZATION_SLOWDOWN

    def test_suspect_without_slowdown_is_navigating(self):
        result = compose(
            localization=localization(state=1, situation='suspect'), localization_expected=True)
        assert result.code == act.NAVIGATING

    def test_unknown_only_when_expected(self):
        assert compose(localization=None, localization_expected=True).code == \
            act.LOCALIZATION_UNKNOWN
        assert compose(localization=None, localization_expected=False).code == act.NAVIGATING

    def test_not_shown_when_not_driving(self):
        for state, code in ((SequencerState.IDLE, act.IDLE),
                            (SequencerState.GOAL_REACHED, act.GOAL_REACHED),
                            (SequencerState.ERROR, act.ERROR),
                            (SequencerState.ON_STARTING, act.DEPARTURE_WAIT)):
            result = compose(
                state=state, localization=localization(state=4, speed_limit=0.3),
                localization_expected=True)
            assert result.code == code

    def test_obstacle_and_recovery_take_priority(self):
        loc = localization(state=3, speed_limit=0.3)
        stop = compose(localization=loc, localization_expected=True,
                       collision=CollisionInput(act.COLLISION_STOP, 'p'))
        assert stop.code == act.OBSTACLE_STOP
        recovery = compose(localization=loc, localization_expected=True,
                           running_recovery_nodes=frozenset({'BackUp'}))
        assert recovery.code == act.RECOVERY
        stalled = compose(localization=loc, localization_expected=True, stalled_sec=100.0)
        assert stalled.code == act.NO_PROGRESS

    def test_localization_takes_priority_over_obstacle_slowdown_and_goal_kind(self):
        loc = localization(speed_limit=0.3)
        assert compose(
            localization=loc, localization_expected=True,
            collision=CollisionInput(act.COLLISION_SLOWDOWN, 'slow')).code == \
            act.LOCALIZATION_SLOWDOWN
        manual = compose(state=SequencerState.IDLE, manual_goal_active=True,
                         localization=loc, localization_expected=True)
        assert (manual.code, manual.index) == (act.LOCALIZATION_SLOWDOWN, -1)
        assert manual.remaining == pytest.approx(12.3)
        external = compose(state=SequencerState.IDLE, external_goal_active=True,
                           localization=loc, localization_expected=True)
        assert (external.code, external.remaining) == (act.LOCALIZATION_SLOWDOWN, -1.0)


class TestPriorityTable:
    def test_rules_and_priority_match(self):
        assert set(act.PRIORITY) == set(act.RULES)
        assert len(act.PRIORITY) == len(set(act.PRIORITY))
        assert act.PRIORITY[-1] == act.IDLE

    def test_every_code_is_reachable(self):
        cases = {
            act.LOCALIZATION_HOLD: dict(state=SequencerState.SUSPENDED,
                                        pause_requesters=('localization',)),
            act.WATCHDOG_PAUSE: dict(state=SequencerState.SUSPENDED,
                                     pause_requesters=('driver_watchdog',)),
            act.PAUSED: dict(state=SequencerState.SUSPENDED),
            act.DEPARTURE_WAIT: dict(state=SequencerState.ON_STARTING),
            act.ACTION: dict(state=SequencerState.ON_ARRIVING),
            act.ERROR: dict(state=SequencerState.ERROR),
            act.DETOUR_REJECTED: dict(running_recovery_nodes=frozenset({'Replan'}),
                                      rejected_detour=act.DetourInput(10.0, 5.0)),
            act.RECOVERY: dict(running_recovery_nodes=frozenset({'Replan'})),
            act.SENSOR_TIMEOUT_STOP: dict(
                collision=CollisionInput(act.COLLISION_STOP, 'source_timeout:scan')),
            act.OBSTACLE_STOP: dict(collision=CollisionInput(act.COLLISION_STOP, 'p')),
            act.NO_PROGRESS: dict(stalled_sec=STALL_WARN_SEC),
            act.LOCALIZATION_RECOVERING_DRIVE: dict(
                localization=localization(state=2), localization_expected=True),
            act.LOCALIZATION_SLOWDOWN: dict(
                localization=localization(speed_limit=0.1), localization_expected=True),
            act.NO_LIDAR_DRIVE: dict(
                localization=localization(sensor_set='no_lidar'), localization_expected=True),
            act.LOCALIZATION_UNKNOWN: dict(localization_expected=True),
            act.OBSTACLE_SLOWDOWN: dict(collision=CollisionInput(act.COLLISION_SLOWDOWN, 's')),
            act.EXTERNAL_GOAL: dict(state=SequencerState.IDLE, external_goal_active=True),
            act.MANUAL_GOAL: dict(state=SequencerState.IDLE, manual_goal_active=True),
            act.NAVIGATING: dict(),
            act.TRIGGER_WAIT: dict(state=SequencerState.IDLE, waiting_trigger=True),
            act.READY_TO_START: dict(state=SequencerState.IDLE, trigger_acceptable=True),
            act.GOAL_REACHED: dict(state=SequencerState.GOAL_REACHED),
            act.IDLE: dict(state=SequencerState.IDLE),
        }
        assert set(cases) == set(act.PRIORITY)
        for code, overrides in cases.items():
            assert compose(**overrides).code == code, code

    def test_validate_rejects_mismatch(self, monkeypatch):
        monkeypatch.setattr(act, 'PRIORITY', act.PRIORITY[1:])
        with pytest.raises(RuntimeError):
            act.validate_priority()
        monkeypatch.setattr(act, 'PRIORITY', (act.IDLE,) + tuple(act.PRIORITY))
        with pytest.raises(RuntimeError):
            act.validate_priority()


def activity(code: str, detail: str = '') -> Activity:
    return Activity(code, code, detail=detail)


class TestActivityDebouncer:
    def test_first_activity_is_taken_at_once(self):
        debouncer = ActivityDebouncer(1.0)
        published, elapsed, change = debouncer.update(activity('a'), 10.0)
        assert (published.code, elapsed) == ('a', 0.0)
        assert (change.from_code, change.activity.code) == ('', 'a')

    def test_short_change_is_dropped(self):
        debouncer = ActivityDebouncer(1.0)
        debouncer.update(activity('a'), 10.0)
        published, elapsed, change = debouncer.update(activity('b'), 10.5)
        assert (published.code, change) == ('a', None)
        assert elapsed == pytest.approx(0.5)
        published, elapsed, change = debouncer.update(activity('a'), 11.0)
        assert (published.code, change) == ('a', None)
        assert elapsed == pytest.approx(1.0)

    def test_lasting_change_is_taken_with_its_start(self):
        debouncer = ActivityDebouncer(1.0)
        debouncer.update(activity('a'), 10.0)
        debouncer.update(activity('b'), 12.0)
        published, elapsed, change = debouncer.update(activity('b'), 13.0)
        assert published.code == 'b'
        assert elapsed == pytest.approx(1.0)
        assert (change.from_code, change.activity.code) == ('a', 'b')
        assert (change.from_sec, change.since_sec) == (pytest.approx(2.0), pytest.approx(1.0))
        _, elapsed, change = debouncer.update(activity('b'), 14.5)
        assert (elapsed, change) == (pytest.approx(2.5), None)

    def test_candidate_restarts_when_code_changes_again(self):
        debouncer = ActivityDebouncer(1.0)
        debouncer.update(activity('a'), 10.0)
        debouncer.update(activity('b'), 10.5)
        assert debouncer.update(activity('c'), 11.0)[0].code == 'a'
        assert debouncer.update(activity('c'), 11.5)[0].code == 'a'
        published, _, change = debouncer.update(activity('c'), 12.0)
        assert (published.code, change.from_code) == ('c', 'a')

    def test_same_code_refreshes_values(self):
        debouncer = ActivityDebouncer(1.0)
        debouncer.update(activity('a', 'x'), 10.0)
        published, _, change = debouncer.update(activity('a', 'y'), 10.5)
        assert (published.detail, change) == ('y', None)

    def test_zero_takes_every_change(self):
        debouncer = ActivityDebouncer(0.0)
        debouncer.update(activity('a'), 10.0)
        published, elapsed, change = debouncer.update(activity('b'), 10.5)
        assert (published.code, elapsed) == ('b', 0.0)
        assert (change.from_code, change.from_sec) == ('a', pytest.approx(0.5))
        assert debouncer.update(activity('a'), 11.0)[2].from_code == 'b'

    def test_stopped_activities_are_taken_at_once(self):
        debouncer = ActivityDebouncer(1.0)
        debouncer.update(activity(act.NAVIGATING), 10.0)
        published, _, change = debouncer.update(activity(act.ACTION), 10.5)
        assert (published.code, change.from_code) == (act.ACTION, act.NAVIGATING)
        # 走行に戻るときは判定にかける
        assert debouncer.update(activity(act.NAVIGATING), 11.0)[0].code == act.ACTION
        assert debouncer.update(activity(act.NAVIGATING), 12.0)[0].code == act.NAVIGATING

    def test_immediate_codes_are_stopped_states(self):
        assert act.IMMEDIATE_CODES <= set(act.PRIORITY)
        assert act.NAVIGATING not in act.IMMEDIATE_CODES
        assert act.OBSTACLE_SLOWDOWN not in act.IMMEDIATE_CODES

