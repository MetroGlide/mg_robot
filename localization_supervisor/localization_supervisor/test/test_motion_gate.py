"""門 (core/motion_gate.py) と、地図との照合で決める規則 (core/map_match.py) のテスト。

部屋の中を円を描いて走るロボットで、推定器 (AMCL)・融合の結果 (EKF)・GNSS の姿勢を作り、門を回す。
照合の依頼は、次の周期の結果として同じスレッドで返す。置き直しは、推定器と融合の結果をその姿勢に移す。
"""
import math

import numpy as np
import pytest

from localization_supervisor.core.geometry import compose
from localization_supervisor.core.grid import scan_points_in_map
from localization_supervisor.core.interfaces import MotionSample, PoseSample
from localization_supervisor.core.map_match import CandidatePose, MapMatchConfig, MapMatcher
from localization_supervisor.core.motion_gate import CONTINUATION, MANUAL, GateState, MotionGate
from localization_supervisor.core.motion_gate_config import parse_gate_config
from localization_supervisor.core.plugins import ConfigError
from localization_supervisor.core.snapshot import DistanceFieldCache, SampleStore
from localization_supervisor.core.status import State, gate_status_view

from supervisor_test_helpers import cov, room_scan, wall_map

PERIOD = 0.2
RADIUS = 4.0
SPEED = 0.5


def truth(t):
    angle = SPEED / RADIUS * t
    return (RADIUS * math.cos(angle), RADIUS * math.sin(angle), angle + math.pi / 2.0)


def config(**gate):
    raw = {
        'version': 1,
        'sources': {
            'amcl': {'kind': 'pose', 'input': {}},
            'ekf': {'kind': 'pose', 'derived_from': ['amcl'], 'input': {}},
            'gnss': {'kind': 'pose', 'input': {}},
            'odom': {'kind': 'motion', 'input': {}},
            'map_scan': {'kind': 'reference', 'input': {}},
        },
        'gate': {'estimator': 'amcl', 'fused': 'ekf', 'motion': 'odom', 'reference': 'map_scan', 'gnss': 'gnss',
                 'max_closed_m': 5.0, 'undecided_hold_sec': 4.0, 'undecided_drive_m': 2.0, **gate},
        'closers': {
            'amcl_jump': {'type': 'pose_jump', 'sources': {'pose': 'amcl', 'motion': 'odom'},
                          'params': {'threshold_m': 0.5, 'threshold_rad': 0.3}},
            'amcl_window': {'type': 'pose_jump', 'sources': {'pose': 'amcl', 'motion': 'odom'},
                            'params': {'threshold_m': 1.0, 'threshold_rad': 0.3, 'window_sec': 10.0}},
            'amcl_gnss': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                          'params': {'metric': 'distance', 'threshold': 1.0, 'b_max_std_m': 0.5},
                          'close_after_sec': 2.0},
        },
        'scan_buffer': {'keyframe_distance_m': 0.3, 'window_distance_m': 3.0},
        'resetters': {'estimator': [{'type': 'amcl'}], 'fused': [{'type': 'robot_localization'}]},
        'gate_service': '/gate',
    }
    return parse_gate_config(raw)


class Sim:
    """円を描いて走るロボット。amcl_offset(t) と ekf_offset は、真の姿勢に右から掛けるずれ。"""

    def __init__(self, **gate):
        self.config = config(**gate)
        self.gate = MotionGate(self.config.gate)
        self.matcher = MapMatcher(self.config.gate.map_match)
        self.store = SampleStore(self.config.gate.kinds, horizon_sec=60.0)
        self.store.set_map('map_scan', wall_map())
        self.t = 0.0
        self.moving = True
        self.odom = (0.0, 0.0, 0.0)
        self.true = truth(0.0)
        self.amcl_offset = (0.0, 0.0, 0.0)
        self.ekf_offset = (0.0, 0.0, 0.0)
        # 推定器を置き直したときの、真の姿勢からのずれ (置き直すまでは None)
        self.amcl_reset = None
        self.gnss = True
        self.gnss_offset = (0.0, 0.0)
        # 推定器の地図の座標でのずれ (ゆっくりしたずれ)
        self.amcl_shift = (0.0, 0.0)
        self.blind = False
        self.pending = None
        self.steps = []

    def amcl(self):
        pose = compose(self.true, self.amcl_offset)
        return (pose[0] + self.amcl_shift[0], pose[1] + self.amcl_shift[1], pose[2])

    def step(self):
        if self.moving:
            self.t_drive = getattr(self, 't_drive', 0.0) + PERIOD
        drive = getattr(self, 't_drive', 0.0)
        self.true = truth(drive)
        self.t += PERIOD
        t = self.t
        self.store.add_motion('odom', MotionSample(t, self.true))
        self.store.add_pose('amcl', PoseSample(t, self.amcl(), cov()))
        self.store.add_pose('ekf', PoseSample(t, compose(self.true, self.ekf_offset), cov()))
        if self.gnss:
            self.store.add_pose('gnss', PoseSample(
                t, (self.true[0] + self.gnss_offset[0], self.true[1] + self.gnss_offset[1], 0.0), cov(0.2)))
        scan = room_scan(t, self.true)
        if self.blind:
            scan.ranges[:] = np.inf
        self.store.set_scan('map_scan', scan)
        step = self.gate.step(self.store.snapshot(t), self.pending)
        self.pending = None
        if step.request is not None:
            self.pending = (step.request, self.matcher.decide(
                step.request.field_for, step.request.points, step.request.candidates))
        if step.reset is not None:
            rel = _relative(self.true, step.reset.pose)
            self.ekf_offset = rel
            if step.reset.targets == 'all':
                self.amcl_reset = rel
                self.amcl_offset = rel
                self.amcl_shift = (0.0, 0.0)
        for event in step.events:
            # イベントの値は 1 つずつの値にする (イベントログの書式)
            assert all(v is None or isinstance(v, (str, int, float, bool)) for v in event.fields.values()), event
        self.steps.append(step)
        return step

    def run(self, seconds):
        return [self.step() for _ in range(int(round(seconds / PERIOD)))]

    def events(self, name):
        return [e for s in self.steps for e in s.events if e.name == name]


def _relative(a, b):
    """a から見た b (a^-1 * b)。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi)


def _error(sim):
    rel = _relative(sim.true, sim.amcl())
    return math.hypot(rel[0], rel[1]), abs(rel[2])


def _confirmed(**gate):
    sim = Sim(**gate)
    sim.run(4.0)
    assert sim.gate.state == GateState.OPEN
    sim.run(13.0)
    return sim


def test_startup_confirms_the_estimator_without_reset():
    sim = Sim()
    sim.run(4.0)
    assert sim.gate.state == GateState.OPEN
    assert sim.events('gate_opened')[0].fields['how'] == 'startup'
    assert not sim.events('pose_reset')
    assert all(s.gate_open for s in sim.steps)


def test_startup_waits_for_the_estimator():
    """推定器がまだ出していない間は、融合の結果だけで決めて置き直さない。"""
    sim = Sim()
    sim.ekf_offset = (0.4, 0.2, 0.05)
    sim.store = SampleStore(sim.config.gate.kinds, horizon_sec=60.0)
    sim.store.set_map('map_scan', wall_map())
    sim.store.add_pose = _skip_amcl(sim.store.add_pose)
    sim.run(4.0)
    assert sim.gate.state == GateState.UNCONFIRMED
    assert not sim.events('map_match') and not sim.events('pose_reset')


def _skip_amcl(add_pose):
    def add(name, sample):
        if name != 'amcl':
            add_pose(name, sample)
    return add


def test_startup_keeps_the_estimator_when_fused_is_off():
    sim = Sim()
    sim.ekf_offset = (0.4, 0.2, 0.05)
    sim.run(4.0)
    assert sim.gate.state == GateState.OPEN
    assert not sim.events('pose_reset')


def test_startup_finds_the_pose_around_gnss_when_estimator_and_fused_are_off():
    """推定器と融合の結果がともにずれた起動では、健全な GNSS の周りで照合した姿勢へ置き直す。"""
    sim = Sim()
    sim.amcl_offset = (1.5, 0.8, 0.1)
    sim.ekf_offset = (1.5, 0.8, 0.1)
    sim.run(4.0)
    assert sim.events('map_match')[0].fields['result'] == 'gnss'
    # 大きい補正なので、止まってから置き直す
    assert sim.gate.state == GateState.CORRECTING
    sim.moving = False
    sim.run(3.0)
    assert sim.events('pose_reset')
    error_xy, error_yaw = _error(sim)
    assert error_xy < 0.3 and error_yaw < 0.1


def test_jump_closes_the_gate_and_resets_to_the_continuation():
    sim = _confirmed()
    sim.amcl_offset = (1.5, 0.0, 0.0)
    sim.ekf_offset = (0.3, 0.0, 0.0)
    step = sim.step()
    assert step.state == GateState.CLOSED and not step.gate_open
    assert 'amcl_jump' in step.closing
    sim.run(6.0)
    match = [e for e in sim.events('map_match') if e.fields['reason'] == 'closed']
    assert match and match[0].fields['result'] in (CONTINUATION, 'ekf')
    resets = sim.events('pose_reset')
    assert resets and resets[0].fields['targets'] == 'all'
    dpos, dyaw = _error(sim)
    assert dpos < 0.3 and dyaw < 0.1
    assert sim.gate.state == GateState.OPEN


def test_large_correction_stops_before_resetting():
    sim = _confirmed()
    sim.amcl_offset = (1.5, 0.0, 0.0)
    sim.ekf_offset = (1.0, 0.0, 0.0)
    sim.run(3.0)
    assert sim.gate.state == GateState.CORRECTING
    assert sim.steps[-1].hold and sim.steps[-1].hold_reason == 'correction'
    assert not sim.events('pose_reset')
    sim.moving = False
    sim.run(3.0)
    reset = sim.events('pose_reset')[0]
    assert reset.fields['how'] == 'stopped' and reset.fields['targets'] == 'all'
    assert not sim.steps[-1].hold
    assert _error(sim)[0] < 0.3


def test_correction_does_not_wait_forever_for_a_stop():
    sim = _confirmed(correction_hold_max_sec=3.0)
    sim.amcl_offset = (1.5, 0.0, 0.0)
    sim.ekf_offset = (1.0, 0.0, 0.0)
    sim.run(6.0)
    assert sim.events('pose_reset')[0].fields['how'] == 'stopped'
    assert sim.gate.state == GateState.OPEN


def test_correct_recovery_of_the_estimator_is_adopted():
    """推定器が誤りから正しい姿勢へ動いたら (正しい直り)、推定器を置き直さず、融合の結果だけを戻す。"""
    sim = _confirmed()
    sim.ekf_offset = (0.8, 0.0, 0.0)
    sim.amcl_offset = (0.8, 0.0, 0.0)
    # 門が閉じないうちに、推定器と融合の結果がそろってずれた (誤った推定器を融合が取り込んだ) 状態を作る
    sim.gate._grace_until = (math.inf, math.inf)
    sim.run(1.0)
    sim.gate._grace_until = (-math.inf, -math.inf)
    sim.amcl_offset = (0.0, 0.0, 0.0)
    sim.run(2.0)
    match = [e for e in sim.events('map_match') if e.fields['reason'] == 'closed'][0]
    assert 'amcl' in match.fields['agree'].split(',')
    sim.moving = False
    sim.run(3.0)
    resets = sim.events('pose_reset')
    assert resets and all(r.fields['targets'] == 'fused' for r in resets)
    assert sim.amcl_reset is None


def test_false_alarm_reopens_after_one_match_without_reset():
    """閉じる理由の部品の誤報 (ずれたのは GNSS) は、照合で推定器を確かめて、置き直さずに開ける。"""
    sim = _confirmed(confirm_distance_m=1000.0)
    sim.ekf_offset = (0.1, 0.0, 0.0)
    sim.gnss_offset = (1.5, 0.0)
    sim.run(4.0)
    assert sim.events('gate_closed')[0].fields['closers'] == 'amcl_gnss'
    assert sim.gate.state == GateState.OPEN
    assert sim.events('gate_opened')[-1].fields['how'] == 'adopted'
    assert not sim.events('pose_reset')


def test_closers_are_ignored_while_in_grace_after_a_reset():
    sim = _confirmed()
    sim.amcl_offset = (1.5, 0.0, 0.0)
    sim.run(5.0)
    assert sim.events('pose_reset')
    closed = len(sim.events('gate_closed'))
    sim.run(12.0)
    assert len(sim.events('gate_closed')) == closed


def test_manual_pose_is_checked_and_becomes_the_confirmed_pose():
    sim = _confirmed()
    true = sim.true
    sim.gate.manual_pose((true[0] + 0.2, true[1] - 0.15, true[2] + 0.04))
    sim.run(0.6)
    manual = sim.events('manual_pose')
    assert manual and manual[0].fields['corrected']
    reset = sim.events('pose_reset')[0]
    assert reset.fields['how'] == MANUAL and reset.fields['targets'] == 'all'
    assert _error(sim)[0] < 0.15
    sim.run(13.0)
    assert not sim.events('gate_closed')


def test_manual_pose_about_a_metre_off_is_corrected_by_the_match():
    """人が与えるおおよその姿勢 (1 m ほどずれ) も、手動の探す範囲の中なら照合で直す。"""
    sim = _confirmed()
    true = sim.true
    sim.gate.manual_pose((true[0] + 0.9, true[1] + 0.6, true[2] - 0.2))
    sim.run(0.6)
    assert sim.events('manual_pose')[0].fields['corrected']
    error_xy, error_yaw = _error(sim)
    assert error_xy < 0.2 and error_yaw < 0.1


def test_manual_pose_is_used_as_given_when_it_cannot_be_matched():
    sim = _confirmed()
    sim.blind = True
    sim.run(8.0)
    sim.moving = False
    sim.run(1.0)
    sim.gate.manual_pose((1.0, 1.0, 0.0))
    sim.run(0.6)
    assert sim.events('manual_pose')[0].fields['corrected'] is False
    reset = sim.events('pose_reset')[-1].fields
    assert (reset['x'], reset['y']) == pytest.approx((1.0, 1.0), abs=0.05)


def test_gnss_disagreement_closes_the_gate_for_slow_drift():
    """動きの合意では気づけないゆっくりしたずれも、GNSS と離れたままなら閉じて、照合で戻す。"""
    sim = _confirmed(confirm_distance_m=1000.0)
    for i in range(1, 400):
        sim.amcl_shift = (0.0, 0.006 * i)
        step = sim.step()
        if 'amcl_gnss' in step.closing or sim.events('gate_closed'):
            break
    closed = sim.events('gate_closed')
    assert closed and closed[0].fields['closers'] == 'amcl_gnss'
    assert all('amcl_jump' not in s.closing and 'amcl_window' not in s.closing for s in sim.steps)
    sim.moving = False
    sim.run(4.0)
    assert sim.events('pose_reset')
    assert _error(sim)[0] < 0.3


def test_periodic_confirmation_moves_the_confirmed_pose():
    sim = _confirmed(confirm_distance_m=2.0)
    sim.run(10.0)
    assert len(sim.events('pose_confirmed')) >= 2
    anchor = sim.gate._anchor
    assert sim.gate._odometer - anchor.odometer < 3.0


def test_undecided_closure_holds_and_releases_by_limits():
    """閉じたまま決まらなければ、上限で止めて照合し、決まらなければ解いて少し走ってから、また止める。"""
    sim = _confirmed()
    sim.blind = True
    sim.run(8.0)
    sim.amcl_offset = (1.5, 0.0, 0.0)
    sim.run(30.0)
    holds = [e for e in sim.events('hold') if e.fields['reason'] == 'undecided']
    starts = [e for e in holds if e.fields['active']]
    ends = [e for e in holds if not e.fields['active']]
    assert len(starts) >= 2 and len(ends) >= 1
    assert sim.gate.state == GateState.CLOSED
    view = gate_status_view(next(s for s in sim.steps if s.hold), sim.config.gate, '')
    assert view.state == State.DEGRADED and view.hold_requested


def test_status_view_follows_the_gate_state():
    sim = _confirmed()
    assert gate_status_view(sim.steps[-1], sim.config.gate, '').state == State.NORMAL
    sim.amcl_offset = (1.5, 0.0, 0.0)
    step = sim.step()
    view = gate_status_view(step, sim.config.gate, 'gate_closed')
    assert view.state == State.ISOLATED and view.isolated_sources == ['amcl']
    assert {c.name for c in view.checks} == {'amcl_jump', 'amcl_window', 'amcl_gnss'}


def test_map_change_returns_to_unconfirmed():
    sim = _confirmed()
    sim.store.set_map('map_scan', wall_map())
    sim.step()
    assert sim.gate.state in (GateState.UNCONFIRMED, GateState.OPEN)
    assert sim.events('gate_map_changed')


# ---------------------------------------------------------------- 照合の規則

def _decide(candidates, **params):
    matcher = MapMatcher(MapMatchConfig(**params))
    true = (1.0, 2.0, 0.3)
    scan = room_scan(0.0, true)
    points = scan_points_in_map(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
                                (0.0, 0.0, 0.0), scan.sensor_pose, 400)
    fields = DistanceFieldCache()
    occupancy = wall_map()
    return matcher.decide(lambda x, y, r: fields.around(occupancy, x, y, r), points, candidates)


def test_same_hypothesis_candidates_are_grouped_after_matching():
    decision = _decide({'a': CandidatePose((1.3, 2.2, 0.3), 0.5, 0.15),
                        'b': CandidatePose((1.0, 2.0, 0.3), 0.5, 0.15)})
    assert decision.best is not None
    assert set(decision.same) == {'a', 'b'}


def test_candidate_off_the_peak_is_not_adopted_as_given():
    """照合で範囲の端まで動いて止まった候補 (山の頂上から外れた入力) は、入力のまま採らない。"""
    decision = _decide({'off': CandidatePose((1.7, 2.0, 0.3), 0.5, 0.15),
                        'on': CandidatePose((1.0, 2.0, 0.3), 0.5, 0.15)}, tolerance_m=0.1)
    assert decision.best == 'on'
    assert 'off' in decision.same and 'off' not in decision.agree


def test_separated_hypotheses_need_a_margin():
    decision = _decide({'true': CandidatePose((1.0, 2.0, 0.3), 0.5, 0.15),
                        'far': CandidatePose((4.0, -3.0, 0.3), 0.5, 0.15)})
    assert decision.best == 'true'
    assert decision.margin is not None and decision.margin > 0.1


def test_wide_window_finds_the_true_pose_beyond_the_narrow_range():
    decision = _decide({'cont': CandidatePose((2.4, 2.9, 0.6), 2.0, 0.55)})
    assert decision.best == 'cont'
    pose = decision.pose
    assert math.hypot(pose[0] - 1.0, pose[1] - 2.0) < 0.15 and abs(pose[2] - 0.3) < 0.06


def test_low_ratio_is_not_decided():
    decision = _decide({'bad': CandidatePose((-6.0, -6.0, 1.5), 0.5, 0.15)}, min_ratio=0.9)
    assert decision.best is None and decision.reason == 'low_ratio'


# ---------------------------------------------------------------- 設定

def test_config_requires_both_resetters():
    with pytest.raises(ConfigError):
        parse_gate_config({**_raw(), 'resetters': {'estimator': [{'type': 'amcl'}]}})


def test_config_checks_closer_sources():
    raw = _raw()
    raw['closers'] = {'x': {'type': 'pose_jump', 'sources': {'pose': 'odom', 'motion': 'odom'}}}
    with pytest.raises(ConfigError):
        parse_gate_config(raw)


def test_config_loads_external_closers():
    raw = _raw()
    raw['closers'] = {'x': {'type': 'supervisor_test_helpers:AlwaysNormal', 'sources': {'pose': 'amcl'}}}
    assert parse_gate_config(raw).gate.closers[0].name == 'x'


def _raw():
    return {
        'version': 1,
        'sources': {'amcl': {'kind': 'pose'}, 'ekf': {'kind': 'pose'}, 'odom': {'kind': 'motion'},
                    'map_scan': {'kind': 'reference'}},
        'gate': {'estimator': 'amcl', 'fused': 'ekf', 'motion': 'odom', 'reference': 'map_scan'},
        'closers': {'j': {'type': 'pose_jump', 'sources': {'pose': 'amcl', 'motion': 'odom'}}},
        'resetters': {'estimator': [{'type': 'amcl'}], 'fused': [{'type': 'robot_localization'}]},
        'gate_service': '/gate',
    }
