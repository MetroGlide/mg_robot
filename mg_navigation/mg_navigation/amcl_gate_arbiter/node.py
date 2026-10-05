"""AMCL の出力を EKF に入れるかどうかを、要求元ごとに調停するノード。

ウェイポイント (`amcl_on` / `amcl_off`) と監督ノードが同じゲート (amcl_publish_controller_node) を
直接切り替えると互いの意図を上書きするため、要求元ごとのサービスを設け、このノードだけがゲートを操作する。

  ~/waypoint/change_publish_state    (std_srvs/SetBool) ウェイポイントの amcl_on / amcl_off
  ~/supervisor/change_publish_state  (std_srvs/SetBool) 自己位置の監督ノードによる切り離し / 復帰

すべての要求元が data=true のときだけゲートを開く。
状態 (要求元の意図、ゲートへの反映済みか、止めている要求元) は、変化したときに
~/state (mg_msgs/GateArbiterState, transient_local) で配信する。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from mg_msgs.msg import GateArbiterState

from .arbiter import GateArbiter

REQUESTERS = ('waypoint', 'supervisor')


class AmclGateArbiterNode(Node):
    def __init__(self):
        super().__init__('amcl_gate_arbiter')
        self.declare_parameter('gate_service', '/amcl_publish_controller_node/change_publish_state')
        self.declare_parameter('period_sec', 1.0)

        self._arbiter = GateArbiter(REQUESTERS)
        self._gate_client = self.create_client(
            SetBool, self.get_parameter('gate_service').value)
        self._pending = False
        self._gate_was_ready = False
        self._state_pub = self.create_publisher(
            GateArbiterState, '~/state',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._last_state = None
        self._publish_state()
        for name in REQUESTERS:
            self.create_service(
                SetBool, f'~/{name}/change_publish_state',
                lambda request, response, who=name: self._on_request(who, request, response))
        # 反映に失敗したとき・ゲートが後から起動したときのために、定期的に合わせ直す
        self.create_timer(self.get_parameter('period_sec').value, self._reconcile)
        self.get_logger().info(f'AMCL gate arbiter started (requesters: {", ".join(REQUESTERS)})')

    def _on_request(self, who, request, response):
        before = self._arbiter.desired
        self._arbiter.request(who, request.data)
        after = self._arbiter.desired
        holders = self._arbiter.holders
        self.get_logger().info(
            f'{who}: {"attach" if request.data else "detach"} -> AMCL output '
            f'{"attached" if after else "detached"}'
            + (f' (held by: {", ".join(holders)})' if holders else ''))
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
        msg = GateArbiterState()
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
            self.get_logger().warning('failed to change the AMCL gate; will retry')


def main(args=None):
    rclpy.init(args=args)
    node = AmclGateArbiterNode()
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
