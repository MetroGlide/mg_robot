"""「いまの活動」の入力 (collision_monitor の状態・BT のリカバリー) の保持のテスト"""
from types import SimpleNamespace
from unittest.mock import MagicMock

from action_msgs.msg import GoalStatus

from mg_waypoint_navigation.waypoint_sequencer.activity import CollisionInput
from mg_waypoint_navigation.waypoint_sequencer.activity_sources import (
    ActivitySources,
    EXTERNAL_GOAL_HOLD_SEC,
)


NAV_STATUS_TOPIC = '/navigate_to_pose/_action/status'


def make_sources(collision_topic='/collision_monitor_state',
                 bt_log_topic='/behavior_tree_log',
                 nav_status_topic=NAV_STATUS_TOPIC):
    node = MagicMock()
    node.count_publishers.return_value = 1
    sources = ActivitySources(
        node, MagicMock(), collision_topic, bt_log_topic, nav_status_topic)
    return sources, node


def nav_status(*statuses):
    return SimpleNamespace(status_list=[SimpleNamespace(status=s) for s in statuses])


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


class TestExternalGoal:
    def test_false_without_status(self):
        sources, _ = make_sources()
        assert not sources.external_goal_active(False, 100.0)

    def test_needs_to_last_hold_sec(self):
        sources, _ = make_sources()
        sources.on_nav_status(nav_status(GoalStatus.STATUS_EXECUTING))
        assert not sources.external_goal_active(False, 100.0)
        assert not sources.external_goal_active(False, 100.0 + EXTERNAL_GOAL_HOLD_SEC - 0.1)
        assert sources.external_goal_active(False, 100.0 + EXTERNAL_GOAL_HOLD_SEC)

    def test_accepted_counts_as_live(self):
        sources, _ = make_sources()
        sources.on_nav_status(nav_status(GoalStatus.STATUS_ACCEPTED))
        sources.external_goal_active(False, 100.0)
        assert sources.external_goal_active(False, 102.0)

    def test_finished_goals_are_not_live(self):
        sources, _ = make_sources()
        sources.on_nav_status(nav_status(
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED))
        sources.external_goal_active(False, 100.0)
        assert not sources.external_goal_active(False, 102.0)

    def test_own_goal_is_not_external(self):
        sources, _ = make_sources()
        sources.on_nav_status(nav_status(GoalStatus.STATUS_EXECUTING))
        sources.external_goal_active(False, 100.0)
        assert not sources.external_goal_active(True, 102.0)

    def test_hold_restarts_after_own_goal_ends(self):
        # 自分のゴールの結果が先に届き、状態トピックの更新が少し遅れる場合
        sources, _ = make_sources()
        sources.on_nav_status(nav_status(GoalStatus.STATUS_EXECUTING))
        sources.external_goal_active(True, 100.0)
        assert not sources.external_goal_active(False, 100.5)
        sources.on_nav_status(nav_status(GoalStatus.STATUS_SUCCEEDED))
        assert not sources.external_goal_active(False, 101.0)
        assert not sources.external_goal_active(False, 105.0)

    def test_ends_when_goal_finishes(self):
        sources, _ = make_sources()
        sources.on_nav_status(nav_status(GoalStatus.STATUS_EXECUTING))
        sources.external_goal_active(False, 100.0)
        assert sources.external_goal_active(False, 102.0)
        sources.on_nav_status(nav_status(GoalStatus.STATUS_SUCCEEDED))
        assert not sources.external_goal_active(False, 102.5)

    def test_stale_status_ignored_when_publisher_is_gone(self):
        sources, node = make_sources()
        sources.on_nav_status(nav_status(GoalStatus.STATUS_EXECUTING))
        sources.external_goal_active(False, 100.0)
        assert sources.external_goal_active(False, 102.0)
        node.count_publishers.return_value = 0
        assert not sources.external_goal_active(False, 103.0)
        node.count_publishers.return_value = 1
        assert not sources.external_goal_active(False, 104.0)

    def test_disabled_without_topic(self):
        sources, node = make_sources(nav_status_topic='')
        sources.on_nav_status(nav_status(GoalStatus.STATUS_EXECUTING))
        assert not sources.external_goal_active(False, 100.0)
        assert not sources.external_goal_active(False, 200.0)
        node.count_publishers.assert_not_called()


class TestSubscriptions:
    def test_all_topics_subscribed(self):
        _, node = make_sources()
        assert node.create_subscription.call_count == 3

    def test_empty_topic_disables_input(self):
        _, node = make_sources(collision_topic='', bt_log_topic='', nav_status_topic='')
        node.create_subscription.assert_not_called()
