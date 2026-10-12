"""nav2_params.yaml の値の整合を検査する。

単純に等しい値は YAML のスカラーのアンカーで共有している。アンカーで表せない関係
(符号の反転、別のファイルとの関係、大小) と、アンカーの使い方の規則をここで検査する。
"""
from pathlib import Path
import re

import pytest
import yaml

NAV2_PARAMS = Path(__file__).resolve().parents[1] / 'params' / 'nav2_params.yaml'
LASER_FILTER = (
    Path(__file__).resolve().parents[2] / 'mg_drivers' / 'params' / 'top_laser_filter.yaml')


@pytest.fixture(scope='module')
def params():
    with open(NAV2_PARAMS, encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def laser_filter():
    with open(LASER_FILTER, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _ros_params(params, node):
    return params[node]['ros__parameters']


def _footprint(params, costmap):
    ros_params = params[costmap][costmap]['ros__parameters']
    return yaml.safe_load(ros_params['footprint'])


def test_anchors_are_scalar_only():
    # リストや辞書のアンカーは、RewrittenYaml (PyYAML の dump) が &id001 / *id001 を出し、
    # rcl のパラメータのパーサが読めず、全 Nav2 サーバーが起動時に落ちる
    with open(NAV2_PARAMS, encoding='utf-8') as f:
        events = list(yaml.parse(f))
    collection_starts = (yaml.SequenceStartEvent, yaml.MappingStartEvent)
    bad = [e for e in events if isinstance(e, collection_starts) and e.anchor is not None]
    assert not bad, f'リストや辞書にアンカーがある: {[e.anchor for e in bad]}'


def test_rewritten_yaml_has_no_aliases():
    # nav2_common.launch.RewrittenYaml と同じ処理 (safe_load -> dump)
    with open(NAV2_PARAMS, encoding='utf-8') as f:
        dumped = yaml.dump(yaml.safe_load(f))
    assert not re.search(r'[&*]id\d+', dumped)


def test_footprint_and_padding_are_shared_by_both_costmaps(params):
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']
    assert local['footprint'] == global_['footprint']
    assert local['footprint_padding'] == global_['footprint_padding']


def test_smoother_max_velocity_matches_cruise_speed(params):
    smoother = _ros_params(params, 'velocity_smoother')
    cruise = _ros_params(params, 'controller_server')['FollowPath']['desired_linear_vel']
    assert smoother['max_velocity'][0] == cruise
    assert smoother['max_velocity'][1] == 0.0


def test_smoother_min_velocity_is_within_max(params):
    smoother = _ros_params(params, 'velocity_smoother')
    assert smoother['min_velocity'][0] <= 0.0
    assert abs(smoother['min_velocity'][0]) <= smoother['max_velocity'][0]
    assert smoother['min_velocity'][2] == -smoother['max_velocity'][2]


def test_smoother_accel_and_decel_signs(params):
    smoother = _ros_params(params, 'velocity_smoother')
    for axis in (0, 2):
        assert smoother['max_accel'][axis] > 0.0
        assert smoother['max_decel'][axis] < 0.0
    assert smoother['max_accel'][1] == 0.0
    assert smoother['max_decel'][1] == 0.0


def test_angular_acceleration_is_the_same_everywhere(params):
    behavior = _ros_params(params, 'behavior_server')['rotational_acc_lim']
    rpp = _ros_params(params, 'controller_server')['FollowPath']['max_angular_accel']
    smoother = _ros_params(params, 'velocity_smoother')
    assert behavior == rpp == smoother['max_accel'][2] == -smoother['max_decel'][2]


def _box_filter(laser_filter):
    chain = laser_filter['top_laser_filter']['ros__parameters']
    return next(
        f['params'] for f in chain.values()
        if isinstance(f, dict) and f.get('type') == 'laser_filters/LaserScanBoxFilter')


def _extent(flat_points):
    """[x1, y1, x2, y2, ...] の外接の矩形 (min_x, max_x, min_y, max_y)。"""
    xs = flat_points[0::2]
    ys = flat_points[1::2]
    return min(xs), max(xs), min(ys), max(ys)


def _polygon(params, name):
    return _ros_params(params, 'collision_monitor')[name]


def test_laser_filter_box_contains_footprint(params, laser_filter):
    box = _box_filter(laser_filter)
    assert box['box_frame'] == 'base_footprint'
    assert box['invert'] is False
    footprint = _footprint(params, 'local_costmap')
    xs = [p[0] for p in footprint]
    ys = [p[1] for p in footprint]
    assert box['min_x'] <= min(xs) and box['max_x'] >= max(xs)
    assert box['min_y'] <= min(ys) and box['max_y'] >= max(ys)


def test_collision_monitor_polygons_are_enabled(params):
    monitor = _ros_params(params, 'collision_monitor')
    assert monitor['polygons'][0] == 'PolygonStop'
    for name in monitor['polygons']:
        assert monitor[name]['enabled'] is True, name


def test_collision_monitor_uses_the_same_scan_as_the_costmaps(params):
    monitor_topic = _ros_params(params, 'collision_monitor')['scan_top']['topic']
    topics = {monitor_topic.lstrip('/')}
    for costmap in ('local_costmap', 'global_costmap'):
        layer = params[costmap][costmap]['ros__parameters']['top_obstacle_layer']
        topics.add(layer['scan']['topic'].lstrip('/'))
    assert topics == {'scan_filtered/scan_top_lidar'}


def test_collision_monitor_stops_when_the_scan_stops(params):
    monitor = _ros_params(params, 'collision_monitor')
    assert monitor['scan_top']['stop_on_timeout'] is True
    # LiDAR は 10 Hz で、rosbag の実測の最大の間隔は 0.2 秒。その 2.5 倍以上は待つ
    assert monitor['source_timeout'] >= 0.5


def test_stop_polygon_starts_outside_the_laser_filter_box(params, laser_filter):
    box = _box_filter(laser_filter)
    min_x, _, min_y, max_y = _extent(_polygon(params, 'PolygonStop')['points'])
    assert min_x >= box['max_x']
    assert min_y <= box['min_y'] and max_y >= box['max_y']


def test_stop_polygon_depth_covers_the_braking_distance(params):
    smoother = _ros_params(params, 'velocity_smoother')
    speed = smoother['max_velocity'][0]
    braking = speed ** 2 / (2 * -smoother['max_decel'][0])
    # scan と制御の遅れ (0.1 秒) ぶんを足す。設計値であり、実機の制動距離は別に測る
    min_x, max_x, _, _ = _extent(_polygon(params, 'PolygonStop')['points'])
    assert max_x - min_x >= braking + speed * 0.1


def test_approach_polygon_is_the_laser_filter_box(params, laser_filter):
    box = _box_filter(laser_filter)
    approach = _polygon(params, 'PolygonApproach')
    assert 'footprint_topic' not in approach
    assert _extent(approach['points']) == (
        box['min_x'], box['max_x'], box['min_y'], box['max_y'])


def test_slowdown_polygon_contains_stop_and_approach(params):
    min_x, max_x, min_y, max_y = _extent(_polygon(params, 'PolygonSlowdown')['points'])
    for name in ('PolygonStop', 'PolygonApproach'):
        inner = _extent(_polygon(params, name)['points'])
        assert min_x <= inner[0] and max_x >= inner[1], name
        assert min_y <= inner[2] and max_y >= inner[3], name


def test_rpp_reacts_before_the_approach_polygon(params):
    rpp = _ros_params(params, 'controller_server')['FollowPath']
    approach = _polygon(params, 'PolygonApproach')
    assert approach['time_before_collision'] < rpp['max_allowed_time_to_collision_up_to_carrot']
