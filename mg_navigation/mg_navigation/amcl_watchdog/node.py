"""AMCL watchdog node: AMCL の共分散を監視し、精度低下時に GNSS で再初期化する。

監督ノード (localization_supervisor) と同じ形で、自己位置推定の状態を /localization/status と
診断 `amcl_watchdog_node: localization` に出す (NORMAL / SUSPECT / RECOVERING / DEGRADED)。
状態の変化と再初期化を、イベントログ (localization.*) に出す。
"""
import time
import threading

import rclpy
import rclpy.duration
from diagnostic_updater import Updater
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from localization_supervisor.core.state_machine import State
from localization_supervisor.ros.diagnostics import LocalizationTask
from localization_supervisor_msgs.msg import LocalizationStatus
from std_srvs.srv import Trigger

from mg_utils.event_log import EventLog

from .health import watchdog_state
from .metrics import compute


class AmclWatchdogNode(Node):
    def __init__(self):
        super().__init__('amcl_watchdog_node')

        # --- パラメータ ---
        self._metric       = self.declare_parameter('metric', 'trace_xy').value
        self._threshold    = float(self.declare_parameter('threshold', 2.0).value)
        self._consec_count = int(self.declare_parameter('consecutive_count', 3).value)
        self._backoff      = float(self.declare_parameter('recovery_backoff_sec', 20.0).value)
        self._max_retries  = int(self.declare_parameter('max_retries', 3).value)
        self._svc_timeout  = float(self.declare_parameter('initializer.call_timeout_sec', 5.0).value)
        self._min_interval = float(self.declare_parameter('min_interval_between_events', 0.0).value)

        # --- 状態変数 ---
        self._consec       = 0           # 連続超過カウンタ
        self._last_event   = 0.0         # 最後に異常検知した時刻 (フラッピング抑止)
        self._last_recovery = 0.0        # 最後にリカバリ開始した時刻 (バックオフ)
        self._in_recovery  = False
        self._recovery_failed = False    # 再初期化に失敗し、まだ戻っていない
        self._event_text   = ''          # 直近のできごと (配信用の文字列)
        self._state        = State.NORMAL
        self._lock         = threading.Lock()
        self._events       = EventLog(self.get_logger())

        # --- ROS インターフェース ---
        self._reinit_client = self.create_client(Trigger, 'request_reinit')
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, 10)
        self._status_pub = self.create_publisher(LocalizationStatus, '/localization/status', 10)
        self._updater = Updater(self)
        self._updater.setHardwareID('localization')
        self._updater.add(LocalizationTask(self._diagnostic_snapshot))
        self.create_timer(1.0, self._on_timer)

        self._events.info(
            'node.started', name='amcl_watchdog_node', metric=self._metric,
            threshold=self._threshold, consecutive_count=self._consec_count)

    # ------------------------------------------------------------------ #
    # 状態の配信
    # ------------------------------------------------------------------ #

    def _diagnostic_snapshot(self):
        # 判定値は共分散の指標だけ。監督ノードの判定値 (GNSS・スキャンなど) は使えない
        return (int(self._state), True, self._event_text, {})

    def _on_timer(self) -> None:
        state = watchdog_state(self._in_recovery, self._recovery_failed, self._consec)
        if state != self._state:
            fields = dict(from_=self._state.name, to=state.name, reason=self._event_text)
            if state == State.NORMAL:
                self._events.info('localization.state_changed', **fields)
            elif state == State.DEGRADED:
                self._events.error('localization.state_changed', **fields)
            else:
                self._events.warn('localization.state_changed', **fields)
            self._state = state

        msg = LocalizationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = int(self._state)
        msg.amcl_attached = True
        msg.amcl_gnss_d2 = -1.0
        msg.amcl_jump_m = -1.0
        msg.amcl_ekf_diff_m = -1.0
        msg.scan_match_ratio = -1.0
        msg.scan_match_gain = -1.0
        msg.last_event = self._event_text
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # コールバック
    # ------------------------------------------------------------------ #

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            if self._in_recovery:
                return

        try:
            value = compute(self._metric, msg.pose.covariance)
        except ValueError as e:
            self.get_logger().warning(f"metric error: {e}")
            return

        self.get_logger().debug(f"{self._metric}={value:.4f}")

        if value <= self._threshold:
            self._recovery_failed = False

        if not self._is_anomaly(value):
            return

        now = time.monotonic()
        if now - self._last_recovery < self._backoff:
            self.get_logger().debug("recovery suppressed (backoff)")
            return

        threading.Thread(
            target=self._recovery_thread, args=(value,), daemon=True
        ).start()

    # ------------------------------------------------------------------ #
    # 異常判定（状態を持つカウンタ）
    # ------------------------------------------------------------------ #

    def _is_anomaly(self, value: float) -> bool:
        now = time.monotonic()
        if value > self._threshold:
            self._consec += 1
        else:
            self._consec = 0
            return False

        if self._consec < self._consec_count:
            return False
        if now - self._last_event < self._min_interval:
            return False

        self._last_event = now
        self._consec = 0
        return True

    # ------------------------------------------------------------------ #
    # リカバリ（バックグラウンドスレッド）
    # ------------------------------------------------------------------ #

    def _recovery_thread(self, metric_value: float) -> None:
        with self._lock:
            self._in_recovery = True
        try:
            self._do_recovery(metric_value)
        finally:
            with self._lock:
                self._in_recovery = False

    def _do_recovery(self, metric_value: float) -> None:
        self._event_text = (
            f"anomaly: {self._metric}={metric_value:.4f} (threshold={self._threshold})")
        msg = ""
        for attempt in range(1, self._max_retries + 1):
            self._events.warn(
                "localization.reinit_requested", source="gnss", attempt=attempt,
                max_attempts=self._max_retries, metric=self._metric,
                value=round(metric_value, 4), threshold=self._threshold)
            ok, msg = self._call_reinit()
            if ok:
                self._events.info(
                    "localization.reinit_result", success=True, attempt=attempt, message=msg)
                self._event_text = f"reinit succeeded: {msg}"
                self._last_recovery = time.monotonic()
                return
            self._events.error(
                "localization.reinit_result", success=False, attempt=attempt, message=msg)
            if attempt < self._max_retries:
                time.sleep(1.0)
        self._event_text = f"reinit failed after {self._max_retries} attempts: {msg}"
        self._recovery_failed = True

    def _call_reinit(self) -> tuple[bool, str]:
        """Trigger サービスを呼び出し (ok, message) を返す。"""
        if not self._reinit_client.wait_for_service(timeout_sec=1.0):
            return False, "service not available"

        future = self._reinit_client.call_async(Trigger.Request())
        start = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=self._svc_timeout)
        while not future.done():
            if (self.get_clock().now() - start) > timeout:
                return False, "timed out"
            time.sleep(0.05)

        result = future.result()
        if result is None:
            return False, "call failed"
        return bool(result.success), result.message


def main(args=None):
    rclpy.init(args=args)
    node = AmclWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down amcl_watchdog")
    finally:
        node.destroy_node()
        rclpy.shutdown()
