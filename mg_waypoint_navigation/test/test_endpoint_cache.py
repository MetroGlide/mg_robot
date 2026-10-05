"""EndpointCache の publisher 生成の単体テスト。ROS2 不要 (conftest が依存をモックする)。"""
from __future__ import annotations

from unittest.mock import MagicMock

from mg_waypoint_navigation.waypoint_sequencer.actions import base
from mg_waypoint_navigation.waypoint_sequencer.actions.base import EndpointCache


def _cache(subscription_counts):
    node = MagicMock()
    publisher = MagicMock()
    publisher.get_subscription_count.side_effect = subscription_counts
    node.create_publisher.return_value = publisher
    return EndpointCache(node), node, publisher


def test_new_publisher_waits_until_a_subscriber_is_found(monkeypatch):
    monkeypatch.setattr(base.time, "sleep", lambda s: None)
    cache, node, publisher = _cache([0, 0, 1])
    assert cache.get_publisher(str, "/t") is publisher
    assert publisher.get_subscription_count.call_count == 3


def test_publisher_is_created_once_and_not_waited_again(monkeypatch):
    monkeypatch.setattr(base.time, "sleep", lambda s: None)
    cache, node, publisher = _cache([1])
    cache.get_publisher(str, "/t")
    cache.get_publisher(str, "/t")
    node.create_publisher.assert_called_once()
    assert publisher.get_subscription_count.call_count == 1


def test_wait_is_bounded_when_nobody_subscribes(monkeypatch):
    times = iter([0.0, 0.5, 1.5])
    monkeypatch.setattr(base.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(base.time, "sleep", lambda s: None)
    cache, node, publisher = _cache(lambda: 0)
    publisher.get_subscription_count.side_effect = None
    publisher.get_subscription_count.return_value = 0
    assert cache.get_publisher(str, "/t") is publisher
