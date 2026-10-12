"""「いまの活動」の合成 (ROS 非依存) のテスト"""
import dataclasses

import pytest

from mg_waypoint_navigation.waypoint_sequencer import activity as act
from mg_waypoint_navigation.waypoint_sequencer.activity import (
    ActivityInputs,
    ActivityTracker,
    CollisionInput,
    compose_activity,
)
from mg_waypoint_navigation.waypoint_sequencer.states import SequencerState

STALL_WARN_SEC = 30.0


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
    return compose_activity(make_inputs(**overrides), STALL_WARN_SEC)


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
        assert result.emphasis == act.NORMAL

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
        assert result.emphasis == act.ALERT


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
        assert result.emphasis == act.ALERT

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
        ({'ClearCostmapsAndWait', 'Wait'}, 'リカバリー: 障害物の記録を消して待機'),
        ({'ClearLocalCostmapAndWait'}, 'リカバリー: 障害物の記録を消して待機'),
        ({'Spin'}, 'リカバリー: 回転'),
        ({'BackUp'}, 'リカバリー: 後退'),
        ({'Wait'}, 'リカバリー: 待機'),
    ])
    def test_text(self, nodes, text):
        result = compose(running_recovery_nodes=frozenset(nodes))
        assert result.code == act.RECOVERY
        assert result.text == text


class TestActivityTracker:
    def test_elapsed_resets_only_when_code_changes(self):
        tracker = ActivityTracker()
        assert tracker.elapsed_sec('a', 10.0) == 0.0
        assert tracker.elapsed_sec('a', 15.0) == pytest.approx(5.0)
        assert tracker.elapsed_sec('b', 16.0) == 0.0
        assert tracker.elapsed_sec('b', 18.0) == pytest.approx(2.0)
        assert tracker.elapsed_sec('a', 19.0) == 0.0
