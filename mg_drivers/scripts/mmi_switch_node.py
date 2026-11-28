#!/usr/bin/env python3
"""MMI (操作盤のマイコン) の物理スイッチと LED で、シーケンサの走行を操作・表示する。

- 開始に割り当てたスイッチの押下で `~/trigger`、停止に割り当てたスイッチの押下で `~/stop` を呼ぶ。
  受け付けるかはシーケンサが決める
- LED1 を、シーケンサの「いまの活動」(`~/activity`) の状態ランプの意味に合わせる
  (led_source: trigger_acceptable なら、受け付けられる間は橙の点滅、それ以外は緑の点灯)。
  受け付けられるようになったらメロディを鳴らす。判断は mmi_protocol.IndicatorController
- `~/status` が届いている間だけハートビートを送る。届かなければ MMI が LED2 を赤にする
- 切断されたら一定の間隔で開き直す
- 診断 `mmi_switch_node: connection` と、イベントログ (driver.connected / disconnected、mmi.*) を出す

MMI は操作者への表示と入力のための装置で、安全停止には使わない (切断されても走行は止めない)。
"""
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from mg_msgs.msg import NavigationActivity, SequencerStatus
from mg_msgs.srv import TriggerSequence
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from mg_utils.event_log import EventLog
from mmi_protocol import (
    COLOR_BLUE, COLOR_GREEN, COLOR_OFF, COLOR_ORANGE, COLOR_RED, COLOR_WHITE, COLOR_YELLOW,
    HEARTBEAT, IndicatorController)
from mmi_serial import MmiLink

HARDWARE_ID = 'mg-robot'

# LED1 の表示の源
LED_SOURCE_ACTIVITY = 'activity'
LED_SOURCE_TRIGGER = 'trigger_acceptable'

# 状態ランプの意味 (NavigationActivity.lamp_response) ごとの、LED1 の定義色の既定
_DEFAULT_COLORS = {
    'normal': COLOR_GREEN, 'watch': COLOR_YELLOW, 'act': COLOR_RED,
    'operator': COLOR_BLUE, 'start': COLOR_ORANGE, 'none': COLOR_OFF,
}


class MmiSwitchNode(Node):
    def __init__(self) -> None:
        super().__init__('mmi_switch_node')
        device = self.declare_parameter('device_name', '/dev/ttyRobot-mmi').value
        baudrate = self.declare_parameter('baudrate', 115200).value
        ignore_after_open = self.declare_parameter('ignore_after_open_sec', 1.0).value
        reconnect_period = self.declare_parameter('reconnect_period_sec', 2.0).value
        heartbeat_period = self.declare_parameter('heartbeat_period_sec', 1.0).value
        read_period = self.declare_parameter('read_period_sec', 0.02).value
        status_timeout = self.declare_parameter('status_timeout_sec', 2.0).value
        reblink_after = self.declare_parameter('reblink_after_sec', 1.5).value
        status_topic = self.declare_parameter(
            'status_topic', '/waypoint_sequencer_node/status').value
        trigger_service = self.declare_parameter(
            'trigger_service', '/waypoint_sequencer_node/trigger').value
        self._requester_id = self.declare_parameter('requester_id', 'mmi_switch').value
        self._start_switch = self.declare_parameter('start_switch', 1).value
        # 走行を止めるスイッチの番号 (0 なら使わない) と、呼ぶサービス
        self._stop_switch = self.declare_parameter('stop_switch', 3).value
        stop_service = self.declare_parameter(
            'stop_service', '/waypoint_sequencer_node/stop').value
        led_source = self.declare_parameter('led_source', LED_SOURCE_ACTIVITY).value
        if led_source not in (LED_SOURCE_ACTIVITY, LED_SOURCE_TRIGGER):
            raise ValueError(f'led_source は {LED_SOURCE_ACTIVITY} か {LED_SOURCE_TRIGGER}: {led_source}')
        activity_topic = self.declare_parameter(
            'activity_topic', '/waypoint_sequencer_node/activity').value
        activity_timeout = self.declare_parameter('activity_timeout_sec', 3.0).value
        colors = {
            response: self.declare_parameter(f'led_color_{response}', color).value
            for response, color in _DEFAULT_COLORS.items()}
        unknown_color = self.declare_parameter('led_color_unknown', COLOR_WHITE).value

        self._events = EventLog(self.get_logger())
        self._link = MmiLink(device, baudrate, ignore_after_open)
        self._indicator = IndicatorController(
            status_timeout, reblink_after,
            follow_activity=led_source == LED_SOURCE_ACTIVITY,
            activity_timeout_sec=activity_timeout, colors=colors, unknown_color=unknown_color)
        self._presses = 0
        self._last_error = ''

        self._trigger_client = self.create_client(TriggerSequence, trigger_service)
        self._stop_client = self.create_client(Trigger, stop_service)
        self.create_subscription(SequencerStatus, status_topic, self._on_status, 10)
        if led_source == LED_SOURCE_ACTIVITY:
            # シーケンサの ~/activity と同じ QoS (transient_local) で、起動直後も今の活動が届く
            self.create_subscription(
                NavigationActivity, activity_topic, self._on_activity,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(read_period, self._on_read_timer)
        self.create_timer(heartbeat_period, self._on_heartbeat_timer)
        self.create_timer(reconnect_period, self._on_reconnect_timer)

        self._updater = Updater(self)
        self._updater.setHardwareID(HARDWARE_ID)
        self._updater.add('connection', self._diagnose_connection)

        self._events.info('node.started', name='mmi_switch_node', device=device)
        self._try_open()

    # ------------------------------------------------------------------
    # シリアル
    # ------------------------------------------------------------------

    def _try_open(self) -> None:
        try:
            self._link.open()
        except (serial.SerialException, OSError) as e:
            self._last_error = str(e)
            return
        self._last_error = ''
        self._events.info('driver.connected', device='mmi', port=self._link.device)
        self._send(self._indicator.on_connected())

    def _disconnect(self, error: Exception) -> None:
        self._link.close()
        self._last_error = str(error)
        self._events.warn(
            'driver.disconnected', device='mmi', port=self._link.device, reason=str(error))

    def _send(self, packets) -> None:
        if not self._link.connected:
            return
        try:
            for packet in packets:
                self._link.write(packet)
        except (serial.SerialException, OSError) as e:
            self._disconnect(e)

    def _on_reconnect_timer(self) -> None:
        if not self._link.connected:
            self._try_open()

    def _on_read_timer(self) -> None:
        if not self._link.connected:
            return
        try:
            switches, ignored = self._link.read_switches()
        except (serial.SerialException, OSError) as e:
            self._disconnect(e)
            return
        for switch in ignored:
            self._events.info('mmi.switch_ignored', switch=switch, reason='just_opened')
        for switch in switches:
            self._on_switch(switch)
        self._send(self._indicator.tick(time.monotonic()))

    def _on_heartbeat_timer(self) -> None:
        if self._indicator.status_fresh(time.monotonic()):
            self._send([HEARTBEAT])

    # ------------------------------------------------------------------
    # シーケンサ
    # ------------------------------------------------------------------

    def _on_status(self, msg: SequencerStatus) -> None:
        self._send(self._indicator.on_status(msg.trigger_acceptable, time.monotonic()))

    def _on_activity(self, msg: NavigationActivity) -> None:
        self._send(self._indicator.on_activity(
            msg.lamp_response, msg.lamp_motion, time.monotonic()))

    def _on_switch(self, switch: int) -> None:
        self._presses += 1
        self._events.info('mmi.switch_pressed', switch=switch)
        # どのスイッチでも、MMI は点滅を点灯に変える
        self._indicator.on_press(time.monotonic())
        if switch == self._stop_switch:
            self._request_stop()
            return
        if switch != self._start_switch:
            return
        if not self._trigger_client.service_is_ready():
            self._events.warn('mmi.trigger_failed', reason='service_unavailable')
            return
        request = TriggerSequence.Request()
        request.requester_id = self._requester_id
        future = self._trigger_client.call_async(request)
        future.add_done_callback(self._on_trigger_done)

    def _on_trigger_done(self, future) -> None:
        response = future.result()
        if response is None:
            self._events.warn('mmi.trigger_failed', reason='no_response')
            return
        self._events.info(
            'mmi.trigger_result', accepted=response.success, message=response.message)

    def _request_stop(self) -> None:
        if not self._stop_client.service_is_ready():
            self._events.warn('mmi.stop_failed', reason='service_unavailable')
            return
        future = self._stop_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_stop_done)

    def _on_stop_done(self, future) -> None:
        response = future.result()
        if response is None:
            self._events.warn('mmi.stop_failed', reason='no_response')
            return
        self._events.info('mmi.stop_result', success=response.success, message=response.message)

    # ------------------------------------------------------------------
    # 診断
    # ------------------------------------------------------------------

    def _diagnose_connection(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        status_fresh = self._indicator.status_fresh(time.monotonic())
        # 走行には要らない装置なので、切断も WARN にとどめる
        if not self._link.connected:
            stat.summary(DiagnosticStatus.WARN, 'disconnected')
        elif not status_fresh:
            stat.summary(DiagnosticStatus.WARN, 'no sequencer status')
        else:
            stat.summary(DiagnosticStatus.OK, 'connected')
        stat.add('device', self._link.device)
        stat.add('sequencer_status_fresh', str(status_fresh))
        shown = self._indicator.shown
        stat.add('display', shown.describe() if shown is not None else 'none')
        stat.add('presses', str(self._presses))
        if self._last_error:
            stat.add('last_error', self._last_error)
        return stat


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MmiSwitchNode()
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
