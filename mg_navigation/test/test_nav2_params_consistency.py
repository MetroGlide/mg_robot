"""nav2_params.yaml の値の整合を検査する。

単純に等しい値は YAML のスカラーのアンカーで共有している。アンカーで表せない関係
(符号の反転、別のファイルとの関係、大小) と、アンカーの使い方の規則をここで検査する。
"""
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

NAV2_PARAMS = Path(__file__).resolve().parents[1] / 'params' / 'nav2_params.yaml'
LASER_FILTER = (
    Path(__file__).resolve().parents[2] / 'mg_drivers' / 'params' / 'top_laser_filter.yaml')
OBSTACLE_DETECTION = (
    Path(__file__).resolve().parents[2] / 'mg_drivers' / 'params' / 'obstacle_detection.yaml')
URDF_DIR = Path(__file__).resolve().parents[2] / 'mg_description' / 'urdf'

# 実機の点群の視野 [rad]。RealSense D435i の点群は、rs_launch の既定 (pointcloud.stream_filter: 2、
# allow_no_texture_points: false) でカラーの視野 (約 69 x 42 度) に制限される
CLOUD_HORIZONTAL_FOV = math.radians(69.0)
CLOUD_VERTICAL_FOV = math.radians(42.0)
# 深度カメラが測れる最短の距離 [m]
DEPTH_MIN_RANGE = 0.2
# STVL の観測源の名前
STVL_SOURCES = {'local_costmap': 'obstacle_points', 'global_costmap': 'pointcloud'}


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


def test_progress_checker_waits_longer_than_the_controller(params):
    # 同じだと、障害物の前で待つ途中で障害物が去っても、進みきる前に「進捗なし」で失敗する
    controller = _ros_params(params, 'controller_server')
    checker = controller[controller['progress_checker_plugin']]
    assert checker['movement_time_allowance'] > controller['failure_tolerance']


def test_spin_speed_range_is_valid(params):
    behavior = _ros_params(params, 'behavior_server')
    assert 0.0 < behavior['min_rotational_vel'] <= behavior['max_rotational_vel']


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


def test_collision_monitor_evaluates_stop_first(params):
    # polygon の enabled は運用で切り替える (すべて無効にしてもよい) ので検査しない。
    # STOP が決まった時点で以降の polygon は評価されないため、STOP を先頭に置く
    monitor = _ros_params(params, 'collision_monitor')
    assert monitor['polygons'][0] == 'PolygonStop'
    for name in monitor['polygons']:
        assert isinstance(monitor[name]['enabled'], bool), name


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
    # ちょうど等しい設計値が浮動小数の丸めで落ちないように、1e-9 の許容を持たせる
    assert max_x - min_x >= braking + speed * 0.1 - 1e-9


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


def test_rpp_lookahead_scales_with_speed_at_cruise(params):
    rpp = _ros_params(params, 'controller_server')['FollowPath']
    assert rpp['use_velocity_scaled_lookahead_dist'] is True
    # 巡航速度での先読みが min にも max にも張り付かないこと (張り付くと速度連動が効かない)
    lookahead = rpp['desired_linear_vel'] * rpp['lookahead_time']
    assert rpp['min_lookahead_dist'] < lookahead < rpp['max_lookahead_dist']
    # 速度連動を使う間は参照されない
    assert 'lookahead_dist' not in rpp


def test_rpp_inflation_cost_scaling_factor_matches_the_local_inflation_layer(params):
    rpp = _ros_params(params, 'controller_server')['FollowPath']
    local = params['local_costmap']['local_costmap']['ros__parameters']
    assert rpp['inflation_cost_scaling_factor'] == (
        local['inflation_layer']['cost_scaling_factor'])


def _distance_to_segment_line(point, a, b):
    """点 point から、点 a と b を通る直線までの距離。"""
    (px, py), (ax, ay), (bx, by) = point, a, b
    return abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / math.hypot(bx - ax, by - ay)


def test_rpp_cost_scaling_dist_exceeds_the_inscribed_radius(params):
    # コストから求めた障害物までの距離は内接半径以上になるので、cost_scaling_dist が
    # 内接半径以下だとコストによる減速が一切効かない。内接半径は、原点から (padding を含む)
    # フットプリントの最も近い辺までの距離
    local = params['local_costmap']['local_costmap']['ros__parameters']
    footprint = _footprint(params, 'local_costmap')
    inscribed = min(
        _distance_to_segment_line((0.0, 0.0), footprint[i], footprint[(i + 1) % len(footprint)])
        for i in range(len(footprint))) + local['footprint_padding']
    rpp = _ros_params(params, 'controller_server')['FollowPath']
    assert rpp['use_cost_regulated_linear_velocity_scaling'] is True
    assert rpp['cost_scaling_dist'] > inscribed
    # 距離を求められるのは、膨張の範囲内だけ
    assert rpp['cost_scaling_dist'] <= local['inflation_layer']['inflation_radius']


def _stvl_source(params, costmap):
    layer = params[costmap][costmap]['ros__parameters']['obstacle_stvl_layer']
    return layer[STVL_SOURCES[costmap]]


def _rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _apply(rot, vec):
    return [sum(rot[i][j] * vec[j] for j in range(3)) for i in range(3)]


def _floats(text):
    return [float(v) for v in text.split()]


def _camera_pose_in_base_footprint():
    """URDF から、base_footprint で見たカメラ (camera_link_base) の位置と姿勢 (回転行列)。"""
    base = ET.parse(URDF_DIR / 'robot_base.urdf.xacro').getroot()
    top = ET.parse(URDF_DIR / 'mg_description.urdf.xacro').getroot()
    base_link_z = _floats(
        next(j for j in base.iter('joint') if j.get('name') == 'base_link_joint')
        .find('origin').get('xyz'))
    top_frame = _floats(
        next(j for j in top.iter('joint') if j.get('name') == 'top_frame_link_joint')
        .find('origin').get('xyz'))
    camera = next(e for e in top.iter() if e.tag.endswith('realsense_d435i'))
    assert camera.get('attach_to') == 'top_frame_link'
    position = [b + t + c for b, t, c in zip(base_link_z, top_frame, _floats(camera.get('xyz')))]
    return position, _rotation(*_floats(camera.get('rpy')))


def _frustum_corners(source):
    """光学座標系 (x 右、y 下、z 前) での、錐台の手前と奥の面の 8 つの角。"""
    tan_h = math.tan(source['horizontal_fov_angle'] / 2.0)
    tan_v = math.tan(source['vertical_fov_angle'] / 2.0)
    return [
        [depth * tan_h * sx, depth * tan_v * sy, depth]
        for depth in (source['min_z'], source['max_z'])
        for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)]


def _optical_to_link(point):
    # 光学座標系 (x 右、y 下、z 前) -> カメラの座標系 (x 前、y 左、z 上)
    return [point[2], -point[0], -point[1]]


@pytest.mark.parametrize('costmap', sorted(STVL_SOURCES))
def test_stvl_clears_with_the_camera_frustum(params, costmap):
    # 錐台は原点の座標系の +Z を向く。点群 (地面の座標系) のままだと真上を向いて消去しない
    source = _stvl_source(params, costmap)
    assert source['clearing'] is True
    assert source['sensor_frame'] == 'camera_depth_optical_frame'
    assert source.get('model_type', 0) == 0
    # 0 だと錐台の中でも古くならず、消去しない
    assert source['decay_acceleration'] > 0.0


@pytest.mark.parametrize('costmap', sorted(STVL_SOURCES))
def test_stvl_frustum_is_within_the_camera_view(params, costmap):
    # 錐台が実際の視野より広いと、見えていない障害物 (近くの低い物の下側など) を消す
    source = _stvl_source(params, costmap)
    assert source['horizontal_fov_angle'] <= CLOUD_HORIZONTAL_FOV
    assert source['vertical_fov_angle'] <= CLOUD_VERTICAL_FOV
    assert DEPTH_MIN_RANGE <= source['min_z'] < source['max_z']


@pytest.mark.parametrize('costmap', sorted(STVL_SOURCES))
def test_stvl_frustum_is_within_the_markable_region(params, costmap):
    # 錐台が検出の ROI やマーク範囲からはみ出すと、検出していない (マークし直せない) 障害物を消す
    source = _stvl_source(params, costmap)
    with open(OBSTACLE_DETECTION, encoding='utf-8') as f:
        roi = yaml.safe_load(f)['obstacle_detection_3d_node']['ros__parameters']
    assert roi['target_frame'] == 'base_footprint'
    position, rotation = _camera_pose_in_base_footprint()
    # 光学座標系の原点とカメラの座標系の原点のずれ (数 cm) の余裕
    margin = 0.05
    for corner in _frustum_corners(source):
        assert math.dist(corner, [0.0, 0.0, 0.0]) <= source['obstacle_range']
        x, y, _ = [p + d for p, d in zip(position, _apply(rotation, _optical_to_link(corner)))]
        assert roi['cropbox_x_min'] + margin <= x <= roi['cropbox_x_max'] - margin
        assert roi['cropbox_y_min'] + margin <= y <= roi['cropbox_y_max'] - margin
