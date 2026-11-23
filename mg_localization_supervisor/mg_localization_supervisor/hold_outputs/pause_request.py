"""門のノードの一時停止の要求を、シーケンサの pause_request に伝える。

止めてほしい間は周期ごとに出し続け (シーケンサが後から起動しても届く)、解いたら一度だけ解除を出す。
シーケンサを通らないゴール (手動のゴールなど) は止められない。
"""
from typing import Any, Mapping

from localization_supervisor.core.interfaces import ParamSpec
from localization_supervisor.ros.hold_outputs import HoldOutput
from mg_msgs.msg import PauseRequest


class PauseRequestHold(HoldOutput):
    PARAMS = {
        'topic': ParamSpec(str, '/waypoint_sequencer_node/pause_request'),
        'requester_id': ParamSpec(str, 'motion_gate'),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params, node)
        self._pub = node.create_publisher(PauseRequest, self.params['topic'], 10)
        self._active = False

    def apply(self, active: bool, reason: str) -> None:
        if not active and not self._active:
            return
        self._active = active
        request = PauseRequest()
        request.requester_id = self.params['requester_id']
        request.active = active
        request.reason = f'localization gate: {reason}' if active else ''
        self._pub.publish(request)
