from mg_utils.monitoring.level_tracker import LevelChange, LevelChangeTracker, level_name


def _tracker(grace=30.0):
    return LevelChangeTracker(start_time=0.0, startup_grace_sec=grace)


def test_level_names():
    assert [level_name(i) for i in range(4)] == ['OK', 'WARN', 'ERROR', 'STALE']
    assert level_name(9) == '9'


def test_no_change_returns_none():
    t = _tracker(grace=0.0)
    assert t.update(1.0, 'a', 0, '') is None
    assert t.update(2.0, 'a', 0, '') is None


def test_change_is_reported_once():
    t = _tracker(grace=0.0)
    t.update(1.0, 'a', 0, 'ok')
    assert t.update(2.0, 'a', 2, 'no data') == LevelChange('a', 'OK', 'ERROR', 'no data')
    assert t.update(3.0, 'a', 2, 'no data') is None
    assert t.update(4.0, 'a', 0, 'ok') == LevelChange('a', 'ERROR', 'OK', 'ok')


def test_names_are_tracked_independently():
    t = _tracker(grace=0.0)
    t.update(1.0, 'a', 0, '')
    t.update(1.0, 'b', 0, '')
    assert t.update(2.0, 'a', 1, '') == LevelChange('a', 'OK', 'WARN', '')
    assert t.update(2.0, 'b', 0, '') is None


def test_changes_during_grace_are_remembered_but_not_reported():
    t = _tracker(grace=30.0)
    assert t.update(1.0, 'a', 2, 'starting') is None
    assert t.update(5.0, 'a', 0, 'ok') is None
    # 猶予が終わったあとは、その前に覚えたレベルとの差を返す
    assert t.update(31.0, 'a', 0, 'ok') is None
    assert t.update(32.0, 'a', 2, 'lost') == LevelChange('a', 'OK', 'ERROR', 'lost')


def test_first_seen_after_grace_reports_only_non_ok():
    t = _tracker(grace=10.0)
    assert t.update(20.0, 'new_ok', 0, '') is None
    assert t.update(20.0, 'new_bad', 2, 'x') == LevelChange('new_bad', 'NONE', 'ERROR', 'x')
    assert t.update(21.0, 'new_ok', 3, 'stale') == LevelChange('new_ok', 'OK', 'STALE', 'stale')
