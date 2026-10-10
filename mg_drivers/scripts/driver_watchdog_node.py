#!/usr/bin/env python3
"""ドライバの生存を監視し、止まったら走行を一時停止して、必要ならプロセスを終了させる。

監視するのは、LiDAR のスキャン・オドメトリ・モータドライバの接続状態などのトピック。
途絶 (またはモータドライバが切断を報告) したら、次のことをする。

- 診断 `driver_watchdog_node: <id>` を ERROR で出す (途絶と受信頻度の判定は mg_utils.monitoring)
- シーケンサの pause_request に、requester_id 付きで一時停止を出し続ける (戻ったら解除する)
- restart_node を指定したソースは、途絶が続いたらそのノードのプロセスに SIGTERM を送る。
  launch の respawn が起動し直す (走行中に止まる rplidar_ros のように、自分では復旧しない
  上流のドライバのため)
- 状態の変化を、イベントログ (watchdog.*) に出す

プロセスの再起動の判定は driver_watchdog_core.py にある。
"""
import time

import rclpy
from diagnostic_updater import Updater
from mg_msgs.msg import PauseRequest
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from driver_watchdog_core import ProcessTerminator, RestartPolicy
from mg_utils.event_log import EventLog
from mg_utils.monitoring.activity import TickGuard
from mg_utils.monitoring.diagnostic_task import TopicHealthTask
from mg_utils.monitoring.topic_source import TopicSource, declare_source_config

HARDWARE_ID = 'mg-robot'


class WatchedSource:
    """監視する 1 つのトピックと、その再起動の状態"""

    def __init__(self, source: TopicSource, restart_node: str, policy: RestartPolicy) -> None:
        self.source = source
        self.restart_node = restart_node
        self.policy = policy
        self.restarts = 0
        self.faulted = False

    @property
    def id(self) -> str:
        return self.source.config.id

    @property
    def topic(self) -> str:
        return self.source.config.topic

    @property
    def monitor(self):
        return self.source.monitor


class DriverWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__('driver_watchdog_node')
        check_period = self.declare_parameter('check_period_sec', 0.5).value
        stale_timeout = self.declare_parameter('stale_timeout_sec', 1.0).value
        recover_hold = self.declare_parameter('recover_hold_sec', 2.0).value
        startup_grace = self.declare_parameter('startup_grace_sec', 30.0).value
        hz_warn_ratio = self.declare_parameter('hz_warn_ratio', 0.5).value
        self._pause_on_fault = self.declare_parameter('pause_on_fault', True).value
        self._pause_requester = self.declare_parameter(
            'pause_requester_id', 'driver_watchdog').value
        pause_topic = self.declare_parameter(
            'pause_request_topic', '/waypoint_sequencer_node/pause_request').value
        self._restart_enabled = self.declare_parameter('restart_enabled', True).value
        restart_after = self.declare_parameter('restart_after_sec', 3.0).value
        restart_cooldown = self.declare_parameter('restart_cooldown_sec', 10.0).value
        kill_timeout = self.declare_parameter('kill_timeout_sec', 3.0).value
        # 周期処理がこれ以上遅れたら、監視側の詰まりとみなして判定を見送る
        self._guard = TickGuard(self.declare_parameter('max_tick_gap_sec', 2.0).value)
        self._terminator = ProcessTerminator(kill_timeout)
        self._events = EventLog(self.get_logger())

        self._updater = Updater(self)
        self._updater.setHardwareID(HARDWARE_ID)
        self._sources = []
        monitored = self.declare_parameter(
            'monitored_sources', Parameter.Type.STRING_ARRAY).value or []
        for source_id in monitored:
            config = declare_source_config(self, source_id, stale_timeout)
            if not config.enabled:
                continue
            restart_node = self.declare_parameter(f'{source_id}.restart_node', '').value
            watched = WatchedSource(
                TopicSource(self, config, recover_hold, startup_grace), restart_node,
                RestartPolicy(restart_after, restart_cooldown))
            self._sources.append(watched)
            self._updater.add(TopicHealthTask(
                config.id, watched.monitor, time.monotonic,
                expected_hz=config.expected_hz, hz_warn_ratio=hz_warn_ratio,
                fault_level=config.fault_level, hardware_id=config.id,
                extra_values=lambda w=watched: {'topic': w.topic, 'restarts': str(w.restarts)}))

        self._pause_pub = self.create_publisher(PauseRequest, pause_topic, 10)
        self._paused_by = []
        self.create_timer(check_period, self._check)
        # 状態が変わったときだけでなく、毎秒送る。~/stop で一時停止の要求が消えたり、
        # シーケンサが再起動したりしても、異常のあいだは一時停止が戻るように
        self.create_timer(1.0, self._publish_pause)
        self._events.info(
            'node.started', name='driver_watchdog_node',
            sources=','.join(s.id for s in self._sources),
            pause_on_fault=self._pause_on_fault, restart_enabled=self._restart_enabled)

    def _check(self) -> None:
        now = time.monotonic()
        lagged = self._guard.check(now)
        if lagged:
            self.get_logger().warn('watchdog tick was delayed; skipping restart this cycle')

        for watched in self._sources:
            if lagged:
                watched.monitor.forgive(now)
            faulted = watched.monitor.update(now)
            self._log_transition(watched, faulted, now)
            watched.faulted = faulted

            if (self._restart_enabled and watched.restart_node and not lagged
                    and watched.policy.should_restart(now, watched.monitor.stale_sec(now))):
                self._restart(watched, now)

        self._terminator.escalate(now)

    def _log_transition(self, watched: WatchedSource, faulted: bool, now: float) -> None:
        if faulted and not watched.faulted:
            self._events.error(
                'watchdog.stale', source=watched.id, topic=watched.topic,
                reason=self._describe(watched, now),
                stale_sec=round(watched.monitor.stale_sec(now), 1))
        elif not faulted and watched.faulted:
            self._events.info('watchdog.recovered', source=watched.id, topic=watched.topic)

    def _restart(self, watched: WatchedSource, now: float) -> None:
        pids = self._terminator.terminate(watched.restart_node, now)
        watched.restarts += 1
        stale_sec = round(watched.monitor.stale_sec(now), 1)
        if pids:
            self._events.error(
                'watchdog.restart_requested', source=watched.id, node=watched.restart_node,
                pids=','.join(str(pid) for pid in pids), stale_sec=stale_sec)
        else:
            self._events.error(
                'watchdog.restart_failed', source=watched.id, node=watched.restart_node,
                reason='process not found', stale_sec=stale_sec)

    @staticmethod
    def _describe(watched: WatchedSource, now: float) -> str:
        if watched.monitor.disconnected:
            return 'reports disconnected'
        return 'no data'

    def _publish_pause(self) -> None:
        if not self._pause_on_fault:
            return
        faulted = [w.id for w in self._sources if w.faulted]
        request = PauseRequest()
        request.requester_id = self._pause_requester
        request.active = bool(faulted)
        request.reason = ', '.join(faulted)
        self._pause_pub.publish(request)
        if faulted != self._paused_by:
            if faulted:
                self._events.warn(
                    'watchdog.pause_changed', active=True, sources=','.join(faulted))
            else:
                self._events.info('watchdog.pause_changed', active=False, sources='')
            self._paused_by = faulted


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
