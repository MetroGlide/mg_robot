"""標準の対処。"""
from typing import Any, List, Mapping, Optional

from std_srvs.srv import Empty, SetBool, Trigger

from ...core.interfaces import ParamSpec
from ...core.supervisor import StepResult
from .base import Actuator


class Gate(Actuator):
    """ゲート (SetBool で開閉するサービス。複数の要求元を調停するゲートの調停ノードの、この監督用の窓口) を、
    走行を止めてほしい状態 (hold_requested) で、blame が blames にあるあいだ閉じる (data=false)。戻ったら開ける。

    状態から毎周期求めて、変わったとき・まだ届いていないときだけ呼ぶ (サービスがなくても、次の周期で再び試す)。
    """
    PARAMS = {
        'service': ParamSpec(str, None, 'SetBool のサービス (true: 開ける、false: 閉じる)'),
        'blames': ParamSpec(list, ['amcl', 'unknown'], '閉じる blame'),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params, node)
        self._client = node.create_client(SetBool, self.params['service'])
        self._applied: Optional[bool] = None
        self._pending = False

    def apply(self, result: StepResult) -> None:
        attach = not (result.hold_requested and result.blame in self.params['blames'])
        if attach == self._applied or self._pending or not self._client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = attach
        self._pending = True
        self._client.call_async(request).add_done_callback(lambda future, want=attach: self._on_done(future, want))

    def _on_done(self, future, attach: bool) -> None:
        self._pending = False
        response = future.result()
        if response is not None and response.success:
            self._applied = attach


class ServiceCall(Actuator):
    """指示 (on) が出たとき、サービスを呼ぶ (例: 再初期化の後に、コストマップを消す)。
    on は reinit・isolate・restore。delay_sec 後に呼ぶ (推定器が初期化し直されるのを待つ)。"""
    SERVICE_TYPES = {'empty': Empty, 'trigger': Trigger}
    PARAMS = {
        'service': ParamSpec(str, None),
        'type': ParamSpec(str, 'empty', 'empty | trigger'),
        'on': ParamSpec(list, ['reinit']),
        'delay_sec': ParamSpec(float, 0.0),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params, node)
        if self.params['type'] not in self.SERVICE_TYPES:
            raise ValueError(f"type: expected one of {sorted(self.SERVICE_TYPES)}, got {self.params['type']!r}")
        self._service_type = self.SERVICE_TYPES[self.params['type']]
        self._client = node.create_client(self._service_type, self.params['service'])
        self._timers: List[Any] = []

    def apply(self, result: StepResult) -> None:
        if not any(d.kind in self.params['on'] for d in result.directives):
            return
        delay = self.params['delay_sec']
        if delay <= 0.0:
            self._call()
            return
        timer = self.node.create_timer(delay, lambda: self._on_timer())
        self._timers.append(timer)

    def _on_timer(self) -> None:
        for timer in self._timers:
            timer.cancel()
        self._timers.clear()
        self._call()

    def _call(self) -> None:
        if self._client.service_is_ready():
            self._client.call_async(self._service_type.Request())
        else:
            self.node.get_logger().warning(f"service {self.params['service']} is not available")
