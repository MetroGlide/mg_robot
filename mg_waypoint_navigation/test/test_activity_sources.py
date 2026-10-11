"""「いまの活動」の入力 (collision_monitor の状態・BT のリカバリー) の保持のテスト"""
from types import SimpleNamespace
from unittest.mock import MagicMock

from mg_waypoint_navigation.waypoint_sequencer.activity import CollisionInput
from mg_waypoint_navigation.waypoint_sequencer.activity_sources import ActivitySources


def make_sources(collision_topic='/collision_monitor_state',
                 bt_log_topic='/behavior_tree_log'):
    node = MagicMock()
    sources = ActivitySources(node, MagicMock(), collision_topic, bt_log_topic)
    return sources, node


def bt_log(*events):
    return SimpleNamespace(event_log=[
        SimpleNamespace(node_name=name, current_status=status)
        for name, status in events])


class TestCollisionState:
    def test_unknown_until_first_message(self):
        sources, _ = make_sources()
        assert sources.collision is None

    def test_keeps_latest(self):
        sources, _ = make_sources()
        sources.on_collision_state(SimpleNamespace(action_type=1, polygon_name='stop'))
        sources.on_collision_state(SimpleNamespace(action_type=0, polygon_name=''))
        assert sources.collision == CollisionInput(0, '')


class TestRecoveryNodes:
    def test_running_and_finished(self):
        sources, _ = make_sources()
        sources.on_bt_log(bt_log(('SpinLeftAndReplan', 'RUNNING')))
        assert sources.recovery_nodes == {'SpinLeftAndReplan'}
        sources.on_bt_log(bt_log(('SpinLeftAndReplan', 'SUCCESS')))
        assert sources.recovery_nodes == frozenset()

    def test_ignores_other_nodes(self):
        sources, _ = make_sources()
        sources.on_bt_log(bt_log(('ComputePathToPose', 'RUNNING'), ('RateController', 'RUNNING')))
        assert sources.recovery_nodes == frozenset()

    def test_follow_path_running_clears_missed_ends(self):
        sources, _ = make_sources()
        sources.on_bt_log(bt_log(('BackUp', 'RUNNING')))
        sources.on_bt_log(bt_log(('FollowPath', 'RUNNING')))
        assert sources.recovery_nodes == frozenset()

    def test_events_are_applied_in_order_within_a_message(self):
        sources, _ = make_sources()
        sources.on_bt_log(bt_log(('FollowPath', 'RUNNING'), ('Spin', 'RUNNING')))
        assert sources.recovery_nodes == {'Spin'}

    def test_clear_recovery(self):
        sources, _ = make_sources()
        sources.on_bt_log(bt_log(('Wait', 'RUNNING')))
        sources.clear_recovery()
        assert sources.recovery_nodes == frozenset()


class TestSubscriptions:
    def test_both_topics_subscribed(self):
        _, node = make_sources()
        assert node.create_subscription.call_count == 2

    def test_empty_topic_disables_input(self):
        _, node = make_sources(collision_topic='', bt_log_topic='')
        node.create_subscription.assert_not_called()
