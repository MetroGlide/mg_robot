"""自己位置推定の監督ノード。

AMCL・GNSS・EKF・スキャン (と地図) を見比べて AMCL のずれを検知し、ずれたら AMCL の出力を EKF から
切り離して、EKF の姿勢で AMCL と EKF を初期化し直す。判定と状態遷移は ROS に依存しないモジュールにある
(checks.py / scan_check.py / state_machine.py)。

  検知 (1 Hz)
    gnss : 精度の良い GNSS と AMCL の位置が合わない
    jump : 連続する AMCL の推定が示す map->odom が、オドメトリの示す動きから飛んだ
    scan : EKF の姿勢でスキャンを地図に重ねても合わない
    diff : AMCL と EKF の位置が離れている
  行動
    切り離し: /amcl_publish_controller_node/change_publish_state (SetBool) で AMCL の出力を止める
    復旧    : 候補 (EKF の姿勢、精度の良い GNSS の位置 + EKF の向き) のうちスキャンが地図に最もよく合う姿勢で
              /initialpose (AMCL) と /set_pose (EKF) を初期化し直し、収束を確認したら AMCL を戻す
"""
import math
from collections import deque
from typing import Deque, List, Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from mg_msgs.msg import LocalizationStatus
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

from .checks import Pose2, amcl_gnss_d2, compose, implied_map_odom, pose_jump
from .scan_check import (
    LocalDistanceField, OccupancyMap, build_local_distance_field, match_ratio, scan_points_in_map)
from .state_machine import Action, Checks, MachineConfig, State, SupervisorMachine

UNAVAILABLE = -1.0


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform_to_pose(transform) -> Pose2:
    t = transform.transform
    return (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))


class LocalizationSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('localization_supervisor_node')
        p = self._declare_parameters()
        self._p = p
        self._machine = SupervisorMachine(MachineConfig(
            gnss_suspect_ticks=p['gnss_suspect_ticks'], gnss_isolate_ticks=p['gnss_isolate_ticks'],
            scan_suspect_ticks=p['scan_suspect_ticks'], scan_isolate_ticks=p['scan_isolate_ticks'],
            diff_suspect_ticks=p['diff_suspect_ticks'], diff_isolate_ticks=p['diff_isolate_ticks'],
            corroborate_ticks=p['corroborate_ticks'], clear_ticks=p['clear_ticks'],
            isolate_hold_sec=p['isolate_hold_sec'], recover_ok_ticks=p['recover_ok_ticks'],
            recover_timeout_sec=p['recover_timeout_sec'], max_reinit_attempts=p['max_reinit_attempts'],
            degraded_retry_sec=p['degraded_retry_sec']))

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 入力
        self._amcl: Optional[Tuple[float, Pose2, Tuple[float, float]]] = None    # (受信時刻, 姿勢, 分散 x, y)
        self._amcl_stamp: Optional[float] = None
        self._amcl_stamp_processed: Optional[float] = None
        self._prev_map_odom: Optional[Pose2] = None
        self._gps: Optional[Tuple[float, float, float, float, str]] = None       # (時刻, x, y, 分散, child_frame)
        self._scan: Optional[LaserScan] = None
        self._map: Optional[OccupancyMap] = None
        self._field: Optional[LocalDistanceField] = None
        self._field_center: Optional[Tuple[float, float]] = None
        self._base_lidar: Optional[Pose2] = None
        self._ignore_jump_until = 0.0
        # 直近の EKF の map->odom (時刻, 姿勢)。AMCL が飛んだ後の復旧で、飛ぶ前の状態に巻き戻すために持つ
        self._map_odom_history: Deque[Tuple[float, Pose2]] = deque()
        self._suspect_since: Optional[float] = None

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose_origin', self._on_amcl, 10)
        self.create_subscription(Odometry, '/odom/gps', self._on_gps, 10)
        self.create_subscription(LaserScan, p['scan_topic'], self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        # 他のノード (GNSS による初期化など) が AMCL を初期化し直したら、その直後の飛びは異常としない
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._on_initialpose, 10)

        # 出力
        self._status_pub = self.create_publisher(LocalizationStatus, '/localization/status', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self._set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/set_pose', 1)
        self._speed_limit_pub = self.create_publisher(SpeedLimit, p['speed_limit_topic'], 1)
        self._gate_client = self.create_client(SetBool, p['amcl_gate_service'])
        self._applied_attach = True
        self._gate_pending = False

        self._values = {'d2': UNAVAILABLE, 'jump': UNAVAILABLE, 'diff': UNAVAILABLE, 'ratio': UNAVAILABLE}
        self._last_event = ''
        self.create_timer(p['period_sec'], self._on_timer)
        self.get_logger().info('localization_supervisor_node started')

    def _declare_parameters(self) -> dict:
        defaults = {
            'period_sec': 1.0,
            'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_footprint',
            'scan_topic': '/scan_top_lidar',
            'amcl_gate_service': '/amcl_publish_controller_node/change_publish_state',
            'speed_limit_topic': '/speed_limit',
            # 判定のしきい値
            'gnss_max_sigma_m': 0.5,             # これ以下の精度の GNSS だけを AMCL との比較に使う
            'gnss_max_age_sec': 3.0,
            'gnss_d2_threshold': 16.0,           # 2 自由度のマハラノビス距離の二乗
            'gnss_extra_std_m': 0.3,             # 時刻のずれなどによる位置の不確かさ
            'amcl_max_age_sec': 3.0,
            'jump_threshold_m': 1.0,
            'jump_threshold_rad': 0.35,
            'ignore_jump_after_init_sec': 5.0,
            'scan_radius_m': 25.0,
            'scan_tolerance_m': 0.3,
            'scan_ratio_threshold': 0.5,         # 一致した点の割合がこれ未満なら異常
            'scan_max_points': 180,
            'diff_threshold_m': 2.0,
            # 状態遷移
            'gnss_suspect_ticks': 3, 'gnss_isolate_ticks': 6,
            'scan_suspect_ticks': 5, 'scan_isolate_ticks': 10,
            'diff_suspect_ticks': 5, 'diff_isolate_ticks': 10,
            'corroborate_ticks': 2, 'clear_ticks': 5,
            'isolate_hold_sec': 3.0, 'recover_ok_ticks': 5, 'recover_timeout_sec': 30.0,
            'max_reinit_attempts': 3, 'degraded_retry_sec': 60.0,
            # 復旧
            'history_sec': 60.0,                 # 巻き戻し用に map->odom を残す時間
            'rollback_margin_sec': 3.0,          # 異常が始まる何秒前の状態まで戻すか
            'reinit_gnss_max_sigma_m': 0.5,
            'reinit_position_std_m': 0.5,
            'reinit_yaw_std_rad': 0.3,
            # DEGRADED のときの動作: warn (通知のみ) | slow (速度を制限する)
            'action_on_degraded': 'warn',
            'degraded_speed_limit_percent': 30.0,
        }
        return {name: self.declare_parameter(name, value).value for name, value in defaults.items()}

    # ---------------------------------------------------------------- 入力

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quaternion(msg.pose.pose.orientation))
        cov = msg.pose.covariance
        self._amcl = (self._now(), pose, (cov[0], cov[7]))
        self._amcl_stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

    def _on_gps(self, msg: Odometry) -> None:
        self._gps = (self._now(), msg.pose.pose.position.x, msg.pose.pose.position.y,
                     msg.pose.covariance[0], msg.child_frame_id)

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self._map = OccupancyMap(data, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)
        self._field = None
        self.get_logger().info(f'map received: {msg.info.width}x{msg.info.height}')

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        self._ignore_jump_until = self._now() + self._p['ignore_jump_after_init_sec']
        self._prev_map_odom = None

    def _lookup_pose(self, target: str, source: str, stamp: Optional[float] = None) -> Optional[Pose2]:
        """target->source の姿勢。stamp が無い、または取れないときは最新。"""
        when = Time(nanoseconds=int(stamp * 1e9)) if stamp is not None else Time()
        try:
            return transform_to_pose(self._tf_buffer.lookup_transform(target, source, when))
        except tf2_ros.TransformException:
            if stamp is None:
                return None
        try:
            return transform_to_pose(self._tf_buffer.lookup_transform(target, source, Time()))
        except tf2_ros.TransformException:
            return None

    # ---------------------------------------------------------------- 判定

    def _check_jump(self, now: float) -> Optional[bool]:
        """新しい AMCL の推定が届いたときだけ判定する。"""
        p = self._p
        if self._amcl is None or self._amcl_stamp is None or self._amcl_stamp == self._amcl_stamp_processed:
            return None
        self._amcl_stamp_processed = self._amcl_stamp
        odom_base = self._lookup_pose(p['odom_frame'], p['base_frame'], self._amcl_stamp)
        if odom_base is None:
            return None
        map_odom = implied_map_odom(self._amcl[1], odom_base)
        previous, self._prev_map_odom = self._prev_map_odom, map_odom
        if previous is None:
            return None
        dpos, dyaw = pose_jump(previous, map_odom)
        self._values['jump'] = dpos
        if now < self._ignore_jump_until:
            return False
        return dpos > p['jump_threshold_m'] or dyaw > p['jump_threshold_rad']

    def _fresh_amcl(self, now: float) -> bool:
        return self._amcl is not None and now - self._amcl[0] <= self._p['amcl_max_age_sec']

    def _check_gnss(self, now: float) -> Optional[bool]:
        p = self._p
        if not self._fresh_amcl(now) or self._gps is None:
            return None
        t, x, y, var, _ = self._gps
        if now - t > p['gnss_max_age_sec'] or var > p['gnss_max_sigma_m'] ** 2:
            return None
        # AMCL と GNSS は届いた時刻がずれるので、その間にロボットが動く分と時刻のずれの不確かさを足す
        gap = abs(self._amcl[0] - t)
        extra = p['gnss_extra_std_m'] ** 2 + (1.0 * gap) ** 2
        d2 = amcl_gnss_d2(self._amcl[1][:2], self._amcl[2], (x, y), var, extra)
        self._values['d2'] = d2
        return d2 > p['gnss_d2_threshold']

    def _check_diff(self, now: float) -> Optional[bool]:
        p = self._p
        if not self._fresh_amcl(now):
            return None
        ekf = self._lookup_pose(p['map_frame'], p['base_frame'], self._amcl_stamp)
        if ekf is None:
            return None
        diff = math.hypot(self._amcl[1][0] - ekf[0], self._amcl[1][1] - ekf[1])
        self._values['diff'] = diff
        return diff > p['diff_threshold_m']

    def _scan_ratio_at(self, pose: Pose2) -> Optional[float]:
        """pose (map->base) でスキャンを地図に重ねたときに一致した点の割合。判断できなければ None。"""
        p = self._p
        scan = self._scan
        if scan is None or self._map is None:
            return None
        if self._base_lidar is None:
            self._base_lidar = self._lookup_pose(p['base_frame'], scan.header.frame_id)
            if self._base_lidar is None:
                return None
        moved = (self._field_center is None
                 or math.hypot(pose[0] - self._field_center[0], pose[1] - self._field_center[1]) > 5.0)
        if self._field is None or moved:
            self._field = build_local_distance_field(self._map, pose[0], pose[1], p['scan_radius_m'])
            self._field_center = (pose[0], pose[1])
        if self._field is None:
            return None
        points = scan_points_in_map(
            np.asarray(scan.ranges, dtype=float), scan.angle_min, scan.angle_increment,
            scan.range_min, min(scan.range_max, p['scan_radius_m']), pose, self._base_lidar,
            p['scan_max_points'])
        result = match_ratio(self._field, points, p['scan_tolerance_m'])
        return None if result is None else result[0]

    def _check_scan(self) -> Optional[bool]:
        p = self._p
        scan = self._scan
        stamp = None if scan is None else Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
        ekf = self._lookup_pose(p['map_frame'], p['base_frame'], stamp)
        if ekf is None:
            return None
        ratio = self._scan_ratio_at(ekf)
        if ratio is None:
            return None
        self._values['ratio'] = ratio
        return ratio < p['scan_ratio_threshold']

    # ---------------------------------------------------------------- 行動

    def _call_gate(self, attach: bool) -> None:
        if self._gate_pending:
            return
        if not self._gate_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = attach
        self._gate_pending = True
        future = self._gate_client.call_async(request)
        future.add_done_callback(lambda f, want=attach: self._on_gate_done(f, want))

    def _on_gate_done(self, future, attach: bool) -> None:
        self._gate_pending = False
        result = future.result()
        if result is not None and result.success:
            self._applied_attach = attach
            self.get_logger().info(f'AMCL output {"attached" if attach else "detached"}')
        else:
            self.get_logger().warning('failed to change AMCL output state; will retry')

    def _candidate_poses(self, now: float) -> List[Tuple[str, Pose2, float]]:
        """復旧の候補 (名前, 姿勢, 位置の標準偏差)。"""
        p = self._p
        candidates: List[Tuple[str, Pose2, float]] = []
        ekf = self._lookup_pose(p['map_frame'], p['base_frame'])
        if ekf is not None:
            candidates.append(('ekf', ekf, p['reinit_position_std_m']))
        # 異常が始まる前の map->odom に、今のオドメトリの動きを足した姿勢 (AMCL が飛んで EKF が引きずられた場合)
        rollback = self._rollback_map_odom()
        odom_base = self._lookup_pose(p['odom_frame'], p['base_frame'])
        if rollback is not None and odom_base is not None:
            candidates.append(('rollback', compose(rollback, odom_base), p['reinit_position_std_m']))
        if self._gps is not None and ekf is not None:
            t, x, y, var, _ = self._gps
            if now - t <= p['gnss_max_age_sec'] and var <= p['reinit_gnss_max_sigma_m'] ** 2:
                candidates.append(('gnss', (x, y, ekf[2]), max(math.sqrt(var), 0.1)))
        return candidates

    def _record_map_odom(self, now: float) -> None:
        """正常なとき (SUSPECT の前) の map->odom を残す。古いものは捨てる。"""
        if self._machine.state != State.NORMAL:
            return
        map_odom = self._lookup_pose(self._p['map_frame'], self._p['odom_frame'])
        if map_odom is None:
            return
        self._map_odom_history.append((now, map_odom))
        while self._map_odom_history and now - self._map_odom_history[0][0] > self._p['history_sec']:
            self._map_odom_history.popleft()

    def _rollback_map_odom(self) -> Optional[Pose2]:
        """異常が始まる rollback_margin_sec 前の、最後の map->odom。"""
        if self._suspect_since is None:
            return None
        limit = self._suspect_since - self._p['rollback_margin_sec']
        chosen = None
        for t, pose in self._map_odom_history:
            if t <= limit:
                chosen = pose
        return chosen

    def _reinit(self, now: float) -> str:
        p = self._p
        candidates = self._candidate_poses(now)
        if not candidates:
            return 'reinit skipped: no pose available'
        best = candidates[0]
        best_ratio = self._scan_ratio_at(best[1])
        for candidate in candidates[1:]:
            ratio = self._scan_ratio_at(candidate[1])
            # スキャンで比べられれば一致の良い方。比べられなければ GNSS を優先する
            if ratio is None or best_ratio is None or ratio > best_ratio:
                best, best_ratio = candidate, ratio
        name, pose, std = best
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = p['map_frame']
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        msg.pose.covariance[0] = std * std
        msg.pose.covariance[7] = std * std
        msg.pose.covariance[35] = p['reinit_yaw_std_rad'] ** 2
        self._ignore_jump_until = now + p['ignore_jump_after_init_sec']
        self._prev_map_odom = None
        self._initialpose_pub.publish(msg)
        self._set_pose_pub.publish(msg)
        ratio_text = 'n/a' if best_ratio is None else f'{best_ratio:.2f}'
        return f'reinit from {name} (scan match {ratio_text})'

    def _apply_degraded(self, degraded: bool) -> None:
        if self._p['action_on_degraded'] != 'slow':
            return
        msg = SpeedLimit()
        msg.percentage = True
        msg.speed_limit = self._p['degraded_speed_limit_percent'] if degraded else 100.0
        self._speed_limit_pub.publish(msg)

    # ---------------------------------------------------------------- 周期

    def _on_timer(self) -> None:
        now = self._now()
        if now == 0.0:
            return
        self._values = {'d2': UNAVAILABLE, 'jump': UNAVAILABLE, 'diff': UNAVAILABLE, 'ratio': UNAVAILABLE}
        checks = Checks(
            gnss=self._check_gnss(now), jump=self._check_jump(now),
            scan=self._check_scan(), diff=self._check_diff(now))
        state_before = self._machine.state
        decision = self._machine.step(now, checks)
        if state_before == State.NORMAL and decision.state == State.SUSPECT:
            self._suspect_since = now
        self._record_map_odom(now)

        events = [decision.event] if decision.event else []
        for action in decision.actions:
            if action == Action.REINIT:
                events.append(self._reinit(now))
            elif action == Action.NOTIFY_DEGRADED:
                self.get_logger().error('localization degraded: AMCL could not be recovered')
                self._apply_degraded(True)
            elif action == Action.ATTACH:
                self._apply_degraded(False)
        if events:
            self._last_event = '; '.join(events)
            self.get_logger().info(f'state={State(decision.state).name} {self._last_event}')

        # AMCL の出力の状態を、望ましい状態 (状態機械が決める) に合わせる。失敗したら次の周期で再試行する
        if self._applied_attach != self._machine.attached:
            self._call_gate(self._machine.attached)
        self._publish_status(decision.state)

    def _publish_status(self, state: State) -> None:
        msg = LocalizationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = int(state)
        msg.amcl_attached = self._applied_attach
        msg.amcl_gnss_d2 = float(self._values['d2'])
        msg.amcl_jump_m = float(self._values['jump'])
        msg.amcl_ekf_diff_m = float(self._values['diff'])
        msg.scan_match_ratio = float(self._values['ratio'])
        msg.last_event = self._last_event
        self._status_pub.publish(msg)

        status = DiagnosticStatus()
        status.name = 'localization_supervisor'
        status.hardware_id = 'mg01'
        status.level = {State.NORMAL: DiagnosticStatus.OK, State.DEGRADED: DiagnosticStatus.ERROR}.get(
            state, DiagnosticStatus.WARN)
        status.message = State(state).name
        status.values = [KeyValue(key='amcl_attached', value=str(self._applied_attach)),
                         KeyValue(key='last_event', value=self._last_event)]
        diag = DiagnosticArray()
        diag.header.stamp = msg.header.stamp
        diag.status = [status]
        self._diag_pub.publish(diag)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
