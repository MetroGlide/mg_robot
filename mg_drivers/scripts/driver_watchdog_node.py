#!/usr/bin/env python3
"""ドライバの生存を監視し、止まったら走行を一時停止して、必要ならプロセスを終了させる。

監視するのは、LiDAR のスキャン・オドメトリ・モータドライバの接続状態などのトピック。
途絶 (またはモータドライバが切断を報告) したら、次のことをする。

- /diagnostics に `driver_watchdog/<id>` を ERROR で出す
- シーケンサの pause_request に、requester_id 付きで一時停止を出し続ける (戻ったら解除する)
- restart_node を指定したソースは、途絶が続いたらそのノードのプロセスに SIGTERM を送る。
  launch の respawn が起動し直す (走行中に止まる rplidar_ros のように、自分では復旧しない
  上流のドライバのため)

判定の計算は driver_watchdog_core.py にある。
"""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mg_msgs.msg import PauseRequest
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from driver_watchdog_core import (
    ProcessTerminator,
    RestartPolicy,
    SourceMonitor,
    TickGuard,
)

# ソースの type と、購読するメッセージ型
SOURCE_TYPES = {
    'scan': LaserScan,
    'odom': Odometry,
    'connected': Bool,
}


class Source:
    """監視する 1 つのトピック"""

    def __init__(self, source_id, topic, restart_node, monitor, policy):
        self.id = source_id
        self.topic = topic
        self.restart_node = restart_node
        self.monitor = monitor
        self.policy = policy
        self.restarts = 0
        self.faulted = False


class DriverWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__('driver_watchdog_node')
        check_period = self.declare_parameter('check_period_sec', 0.5).value
        self._stale_timeout = self.declare_parameter('stale_timeout_sec', 1.0).value
        self._recover_hold = self.declare_parameter('recover_hold_sec', 2.0).value
        self._startup_grace = self.declare_parameter('startup_grace_sec', 30.0).value
        self._pause_on_fault = self.declare_parameter('pause_on_fault', True).value
        self._pause_requester = self.declare_parameter(
            'pause_requester_id', 'driver_watchdog').value
        pause_topic = self.declare_parameter(
            'pause_request_topic', '/waypoint_sequencer_node/pause_request').value
        self._restart_enabled = self.declare_parameter('restart_enabled', True).value
        self._restart_after = self.declare_parameter('restart_after_sec', 3.0).value
        self._restart_cooldown = self.declare_parameter('restart_cooldown_sec', 10.0).value
        kill_timeout = self.declare_parameter('kill_timeout_sec', 3.0).value
        # 周期処理がこれ以上遅れたら、監視側の詰まりとみなして判定を見送る
        self._guard = TickGuard(self.declare_parameter('max_tick_gap_sec', 2.0).value)
        self._terminator = ProcessTerminator(kill_timeout)

        now = time.monotonic()
        self._sources = []
        monitored = self.declare_parameter(
            'monitored_sources', Parameter.Type.STRING_ARRAY).value or []
        for source_id in monitored:
            source = self._create_source(source_id, now)
            if source is not None:
                self._sources.append(source)

        self._pause_pub = self.create_publisher(PauseRequest, pause_topic, 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(check_period, self._check)
        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            f'driver_watchdog_node started: sources={[s.id for s in self._sources]} '
            f'pause_on_fault={self._pause_on_fault} restart_enabled={self._restart_enabled}')

    def _create_source(self, source_id: str, now: float):
        if not self.declare_parameter(f'{source_id}.enabled', True).value:
            return None
        topic = self.declare_parameter(f'{source_id}.topic', '').value
        if not topic:
            raise ValueError(f'{source_id}.topic が指定されていない')
        source_type = self.declare_parameter(f'{source_id}.type', '').value
        if source_type not in SOURCE_TYPES:
            raise ValueError(
                f'{source_id}.type は {list(SOURCE_TYPES)} のいずれか: {source_type}')
        restart_node = self.declare_parameter(f'{source_id}.restart_node', '').value
        stale_timeout = self.declare_parameter(
            f'{source_id}.stale_timeout_sec', self._stale_timeout).value
        armed_on_first = self.declare_parameter(
            f'{source_id}.armed_on_first_message', False).value

        monitor = SourceMonitor(
            stale_timeout, self._recover_hold, now, self._startup_grace, armed_on_first)
        policy = RestartPolicy(self._restart_after, self._restart_cooldown)
        source = Source(source_id, topic, restart_node, monitor, policy)

        if source_type == 'connected':
            def on_message(msg, monitor=monitor):
                monitor.on_message(time.monotonic(), ok=msg.data)
        else:
            def on_message(msg, monitor=monitor):
                monitor.on_message(time.monotonic())
        self.create_subscription(
            SOURCE_TYPES[source_type], topic, on_message, qos_profile_sensor_data)
        return source

    def _check(self) -> None:
        now = time.monotonic()
        lagged = self._guard.check(now)
        if lagged:
            self.get_logger().warn('watchdog tick was delayed; skipping restart this cycle')

        for source in self._sources:
            if lagged:
                source.monitor.forgive(now)
            faulted = source.monitor.update(now)
            self._log_transition(source, faulted, now)
            source.faulted = faulted

            if (self._restart_enabled and source.restart_node and not lagged
                    and source.policy.should_restart(now, source.monitor.stale_sec(now))):
                self._restart(source, now)

        self._terminator.escalate(now)

    def _log_transition(self, source: Source, faulted: bool, now: float) -> None:
        if faulted and not source.faulted:
            self.get_logger().error(
                f'{source.id} ({source.topic}) fault: {self._describe(source, now)}')
        elif not faulted and source.faulted:
            self.get_logger().info(f'{source.id} ({source.topic}) recovered')

    def _restart(self, source: Source, now: float) -> None:
        pids = self._terminator.terminate(source.restart_node, now)
        source.restarts += 1
        self.get_logger().error(
            f'{source.id} stale for {source.monitor.stale_sec(now):.1f} s: '
            f'terminated {source.restart_node} (pids={pids}); launch respawn restarts it')

    @staticmethod
    def _describe(source: Source, now: float) -> str:
        if source.monitor.disconnected:
            return 'reports disconnected'
        return f'no data for {source.monitor.stale_sec(now):.1f} s'

    def _publish(self) -> None:
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()

        array = DiagnosticArray()
        array.header.stamp = stamp
        for source in self._sources:
            status = DiagnosticStatus()
            status.name = f'driver_watchdog/{source.id}'
            status.hardware_id = source.topic
            if source.faulted:
                status.level = DiagnosticStatus.ERROR
                status.message = self._describe(source, now)
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'ok' if source.monitor.received else 'waiting for first message'
            status.values.append(KeyValue(key='topic', value=source.topic))
            status.values.append(
                KeyValue(key='stale_sec', value=f'{source.monitor.stale_sec(now):.1f}'))
            status.values.append(KeyValue(key='restarts', value=str(source.restarts)))
            array.status.append(status)
        self._diag_pub.publish(array)

        # 状態が変わったときだけでなく、毎秒送る。~/stop で一時停止の要求が消えたり、
        # シーケンサが再起動したりしても、異常のあいだは一時停止が戻るように
        if self._pause_on_fault:
            faulted = [s.id for s in self._sources if s.faulted]
            request = PauseRequest()
            request.requester_id = self._pause_requester
            request.active = bool(faulted)
            request.reason = ', '.join(faulted)
            self._pause_pub.publish(request)


def main() -> None:
    rclpy.init()
    node = DriverWatchdogNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
