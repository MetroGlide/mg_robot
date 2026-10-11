#!/usr/bin/env python3
"""GNSS の測位の品質 (RTK が Fixed か) を、診断として出す。

ublox_gps_node の標準の診断は、RTK の Fixed と Float を区別しないので、`navpvt` から判定する
(判定は gnss_quality.py)。`navpvt` が途絶したら ERROR。測位の種類が変わったら
イベント `gnss.fix_changed` を出す。
"""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, DiagnosticTask, Updater
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ublox_msgs.msg import NavPVT

from gnss_quality import FLAGS_GNSS_FIX_OK, carrier_phase, classify
from mg_utils.event_log import EventLog
from mg_utils.monitoring.activity import ActivityMonitor

HARDWARE_ID = 'gnss'
MM = 1e-3
P_DOP_UNIT = 0.01

_LEVELS = {
    'ok': DiagnosticStatus.OK,
    'warn': DiagnosticStatus.WARN,
    'error': DiagnosticStatus.ERROR,
}


class QualityTask(DiagnosticTask):
    def __init__(self, node: 'GnssDiagnosticsNode') -> None:
        super().__init__('quality')
        self._node = node

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        node = self._node
        now = time.monotonic()
        if node.monitor.update(now):
            stat.summary(
                DiagnosticStatus.ERROR, f'no data for {node.monitor.stale_sec(now):.1f} s')
            return stat
        msg = node.last
        if msg is None:
            stat.summary(DiagnosticStatus.OK, 'waiting for first message')
            return stat
        label, level = classify(msg.fix_type, msg.flags)
        stat.summary(_LEVELS[level], label)
        stat.add('fix_type', str(msg.fix_type))
        stat.add('carrier_phase', str(carrier_phase(msg.flags)))
        stat.add('fix_ok', str(bool(msg.flags & FLAGS_GNSS_FIX_OK)))
        stat.add('h_acc_m', f'{msg.h_acc * MM:.3f}')
        stat.add('v_acc_m', f'{msg.v_acc * MM:.3f}')
        stat.add('p_dop', f'{msg.p_dop * P_DOP_UNIT:.2f}')
        stat.add('num_sv', str(msg.num_sv))
        return stat


class GnssDiagnosticsNode(Node):
    def __init__(self) -> None:
        super().__init__('gnss_diagnostics_node')
        topic = self.declare_parameter('topic', '/navpvt').value
        stale_timeout = self.declare_parameter('stale_timeout_sec', 3.0).value
        startup_grace = self.declare_parameter('startup_grace_sec', 30.0).value
        self.monitor = ActivityMonitor(
            stale_timeout, 2.0, time.monotonic(), startup_grace, armed_on_first_message=False)
        self.last = None
        self._label = None
        self._events = EventLog(self.get_logger())

        self._updater = Updater(self)
        self._updater.setHardwareID(HARDWARE_ID)
        self._updater.add(QualityTask(self))
        self.create_subscription(NavPVT, topic, self._on_navpvt, qos_profile_sensor_data)
        self._events.info('node.started', name='gnss_diagnostics_node', topic=topic)

    def _on_navpvt(self, msg: NavPVT) -> None:
        self.monitor.on_message(time.monotonic())
        self.last = msg
        label, level = classify(msg.fix_type, msg.flags)
        if label != self._label:
            fields = dict(from_=self._label or 'none', to=label, h_acc_m=round(msg.h_acc * MM, 3))
            if level == 'ok':
                self._events.info('gnss.fix_changed', **fields)
            elif level == 'warn':
                self._events.warn('gnss.fix_changed', **fields)
            else:
                self._events.error('gnss.fix_changed', **fields)
            self._label = label


def main() -> None:
    rclpy.init()
    node = GnssDiagnosticsNode()
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
