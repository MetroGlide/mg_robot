"""センサ構成が変わったら、ほかのノードのパラメータを組ごとの値に切り替える (センサの途絶の見張りからの引き継ぎ)。

例: LiDAR なしの構成では、collision_monitor の scan の途絶による停止を切り、センサの途絶の見張りの LiDAR の
一時停止を外す (深度の障害物で、構成の速度の上限で走る)。全部のセンサ構成に戻ったら、all の値に戻す。
パラメータは、各ノードの set_parameters サービスで送る。届かない・失敗したときは、retry_sec ごとに送り直す。
起動の直後 (全部のセンサ構成) は送らない (各ノードは設定ファイルの値、つまり all の値で起動している)。
"""
import math
from typing import Any, Dict, List, Mapping

from localization_supervisor.core.interfaces import ParamSpec
from localization_supervisor.core.supervisor import StepResult
from localization_supervisor.ros.actuators import Actuator
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

ALL = 'all'


class SensorSetParameters(Actuator):
    PARAMS = {
        'sets': ParamSpec(dict, None, '構成の名前 (全部のセンサ構成は all) → ノード名 → {パラメータ名: 値}'),
        'retry_sec': ParamSpec(float, 2.0, '届かない・失敗したときに、送り直すまでの時間 [s]'),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params, node)
        self._sets: Dict[str, Dict[str, Dict[str, Any]]] = {
            set_name: {node_name: dict(values) for node_name, values in nodes.items()}
            for set_name, nodes in self.params['sets'].items()}
        self._clients: Dict[str, Any] = {}
        self._wanted = ALL
        self._done = True
        self._pending: List[Any] = []
        self._sent_at = -math.inf

    def apply(self, result: StepResult) -> None:
        wanted = result.sensor_set or ALL
        if wanted != self._wanted:
            self._wanted, self._done, self._pending, self._sent_at = wanted, False, [], -math.inf
        if self._done:
            return
        if self._pending:
            if not all(f.done() for f in self._pending):
                return
            if all(_succeeded(f) for f in self._pending):
                self._done = True
                self.node.get_logger().info(f'sensor set {wanted}: parameters applied')
                return
            self._pending = []
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if now - self._sent_at < self.params['retry_sec']:
            return
        self._sent_at = now
        nodes = self._sets.get(wanted, {})
        futures = [f for f in (self._send(n, v) for n, v in nodes.items()) if f is not None]
        if len(futures) == len(nodes):
            self._pending = futures
            self._done = not futures

    def _send(self, node_name: str, values: Mapping[str, Any]) -> Any:
        """届けば応答を待つ future、届かなければ None (送り直す)。"""
        client = self._clients.get(node_name)
        if client is None:
            client = self.node.create_client(SetParameters, f'{node_name}/set_parameters')
            self._clients[node_name] = client
        if not client.service_is_ready():
            self.node.get_logger().warn(f'sensor set: {node_name}/set_parameters is not ready')
            return None
        request = SetParameters.Request()
        request.parameters = [Parameter(key, value=value).to_parameter_msg() for key, value in values.items()]
        return client.call_async(request)


def _succeeded(future: Any) -> bool:
    response = future.result()
    return response is not None and all(r.successful for r in response.results)
