"""ゲート (SetBool で開閉するサービス) を、要求元ごとに調停するノード。

複数の要求元 (例: 走行の手順と監督ノード) が同じゲートを直接切り替えると互いの意図を上書きするため、
要求元ごとのサービスを設け、このノードだけがゲートを操作する。

  ~/<要求元>/change_publish_state  (std_srvs/SetBool) 要求元ごとの開閉の要求

すべての要求元が data=true のときだけゲートを開く。
状態 (要求元の意図、ゲートへの反映済みか、閉じている要求元) は、変化したときに
~/state (localization_supervisor_msgs/GateState, transient_local) で配信する。

パラメータ
  gate_service   操作するゲートのサービス (std_srvs/SetBool)
  requesters     要求元の名前の一覧
  warn_holders   この要求元がゲートを閉じたときは、イベントを warn で出す (異常による切り離しなど)
  hardware_id    診断のハードウェア ID
  period_sec     ゲートに合わせ直す周期
"""

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, DiagnosticTask, Updater
from localization_supervisor_msgs.msg import GateState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from ..core.gate_arbiter import GateArbiter, gate_health
from .event_sinks.logfmt import LogfmtEventSink

_LEVELS = {'ok': DiagnosticStatus.OK, 'warn': DiagnosticStatus.WARN}


class GateTask(DiagnosticTask):
    """診断 `gate`: 要求が、ゲートに反映できているか。"""

    def __init__(self, arbiter: GateArbiter) -> None:
        super().__init__('gate')
        self._arbiter = arbiter

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        level, message, values = gate_health(
            self._arbiter.desired, self._arbiter.applied, self._arbiter.holders)
        stat.summary(_LEVELS[level], message)
        for key, value in values.items():
            stat.add(key, value)
        return stat


class GateArbiterNode(Node):
    def __init__(self):
        super().__init__('gate_arbiter')
        gate_service = self.declare_parameter('gate_service', 'change_publish_state').value
        requesters = list(self.declare_parameter('requesters', ['supervisor']).value)
        self._warn_holders = set(self.declare_parameter('warn_holders', ['supervisor']).value)
        hardware_id = self.declare_parameter('hardware_id', 'gate').value
        period_sec = self.declare_parameter('period_sec', 1.0).value

        self._arbiter = GateArbiter(requesters)
        self._events = LogfmtEventSink(self.get_logger())
        self._updater = Updater(self)
        self._updater.setHardwareID(hardware_id)
        self._updater.add(GateTask(self._arbiter))
        self._gate_client = self.create_client(SetBool, gate_service)
        self._pending = False
        self._gate_was_ready = False
        self._state_pub = self.create_publisher(
            GateState, '~/state',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._last_state = None
        self._publish_state()
        for name in requesters:
            self.create_service(
                SetBool, f'~/{name}/change_publish_state',
                lambda request, response, who=name: self._on_request(who, request, response))
        # 反映に失敗したとき・ゲートが後から起動したときのために、定期的に合わせ直す
        self.create_timer(period_sec, self._reconcile)
        self._events.info('node.started', name=self.get_name(), requesters=','.join(requesters))

    def _on_request(self, who, request, response):
        before = self._arbiter.desired
        before_holders = self._arbiter.holders
        self._arbiter.request(who, request.data)
        after = self._arbiter.desired
        holders = self._arbiter.holders
        if before != after or before_holders != holders:
            fields = dict(
                requester=who, attached=after, holders=','.join(holders))
            if self._warn_holders.intersection(holders):
                self._events.warn('gate.changed', **fields)
            else:
                self._events.info('gate.changed', **fields)
        response.success = True
        response.message = 'attached' if after else f'detached (held by: {", ".join(holders)})'
        self._publish_state()
        if before != after:
            self._reconcile()
        return response

    def _publish_state(self):
        state = (self._arbiter.desired, self._arbiter.applied, tuple(self._arbiter.holders))
        if state == self._last_state:
            return
        self._last_state = state
        msg = GateState()
        msg.desired, msg.applied, holders = state
        msg.holders = list(holders)
        self._state_pub.publish(msg)

    def _reconcile(self):
        ready = self._gate_client.service_is_ready()
        if self._gate_was_ready and not ready:
            # ゲートのノードが落ちた (再起動されると開いた状態に戻る) ので、次に現れたら送り直す
            self._arbiter.mark_unknown()
            self._publish_state()
        self._gate_was_ready = ready
        if self._pending or not ready or not self._arbiter.needs_apply():
            return
        request = SetBool.Request()
        request.data = self._arbiter.desired
        self._pending = True
        future = self._gate_client.call_async(request)
        future.add_done_callback(lambda f, want=request.data: self._on_done(f, want))

    def _on_done(self, future, attach):
        self._pending = False
        result = future.result()
        if result is not None and result.success:
            self._arbiter.mark_applied(attach)
            self._publish_state()
        else:
            self.get_logger().warning('failed to change the gate; will retry')


def main(args=None):
    rclpy.init(args=args)
    node = GateArbiterNode()
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
