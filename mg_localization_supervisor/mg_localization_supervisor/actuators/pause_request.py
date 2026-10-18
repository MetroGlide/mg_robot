"""走行を止めてほしい状態のあいだ、シーケンサの一時停止を要求し続ける。

シーケンサの pause_request に、要求元 (requester_id) 付きで出し続け (driver_watchdog_node と同じ方式)、
戻ったら解除する。要求は周期ごとに出すので、シーケンサが後から起動しても届く。
シーケンサを通らないゴール (手動のゴールなど) は止められない。
"""
from typing import Any, Mapping

from localization_supervisor.core.interfaces import ParamSpec
from localization_supervisor.core.supervisor import StepResult
from localization_supervisor.ros.actuators import Actuator
from mg_msgs.msg import PauseRequest


class PauseRequestActuator(Actuator):
    PARAMS = {
        'topic': ParamSpec(str, '/waypoint_sequencer_node/pause_request'),
        'requester_id': ParamSpec(str, 'localization'),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params, node)
        self._pub = node.create_publisher(PauseRequest, self.params['topic'], 10)

    def apply(self, result: StepResult) -> None:
        request = PauseRequest()
        request.requester_id = self.params['requester_id']
        request.active = bool(result.hold_requested)
        request.reason = f'localization {result.state.name} blame={result.blame}' if result.hold_requested else ''
        self._pub.publish(request)
