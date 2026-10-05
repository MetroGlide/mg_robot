"""SequencerProgressTracker の判定ロジックの単体テスト。"""
from __future__ import annotations

from mg_scenario_test.sequencer_tracker import Progress, SequencerProgressTracker


def _tracker():
    return SequencerProgressTracker(idle_stall_sec=10.0)


def test_waiting_without_status():
    assert _tracker().evaluate(0, 0.0, 1.0) == Progress.WAITING


def test_stale_goal_reached_is_not_treated_as_reached():
    t = _tracker()
    t.update("GOAL_REACHED", 5, now=0.0)
    t.reset_run()
    t.update("GOAL_REACHED", 5, now=0.1)
    assert t.evaluate(0, 0.0, 0.2) == Progress.WAITING


def test_reached_after_active_state_observed():
    t = _tracker()
    t.update("ON_STARTING", 0, now=0.0)
    t.update("NAVIGATING", 0, now=1.0)
    assert t.evaluate(0, 0.0, 1.0) == Progress.WAITING
    t.update("NAVIGATING", 1, now=2.0)
    assert t.evaluate(0, 0.0, 2.0) == Progress.REACHED


def test_last_waypoint_goal_reached():
    t = _tracker()
    t.update("NAVIGATING", 2, now=0.0)
    t.update("GOAL_REACHED", 3, now=1.0)
    assert t.evaluate(2, 0.0, 1.0) == Progress.REACHED


def test_wait_trigger_idle_after_waypoint_counts_as_reached():
    t = _tracker()
    t.update("NAVIGATING", 0, now=0.0)
    t.update("IDLE", 1, now=1.0)
    assert t.evaluate(0, 0.0, 1.0) == Progress.REACHED


def test_error_state():
    t = _tracker()
    t.update("NAVIGATING", 0, now=0.0)
    t.update("ERROR", 0, now=1.0)
    assert t.evaluate(0, 0.0, 1.0) == Progress.ERROR


def test_idle_stall_detected_after_timeout():
    t = _tracker()
    t.update("NAVIGATING", 0, now=0.0)
    t.update("IDLE", 1, now=1.0)
    assert t.evaluate(1, 2.0, 5.0) == Progress.WAITING
    assert t.evaluate(1, 2.0, 12.5) == Progress.STALLED


def test_suspended_is_not_stall():
    t = _tracker()
    t.update("SUSPENDED", 0, now=0.0)
    assert t.evaluate(0, 0.0, 100.0) == Progress.WAITING


def test_idle_stall_restarts_after_activity():
    t = _tracker()
    t.update("IDLE", 0, now=0.0)
    t.update("ON_STARTING", 0, now=5.0)
    t.update("IDLE", 0, now=6.0)
    assert t.evaluate(0, 0.0, 15.0) == Progress.WAITING
    assert t.evaluate(0, 0.0, 16.5) == Progress.STALLED
