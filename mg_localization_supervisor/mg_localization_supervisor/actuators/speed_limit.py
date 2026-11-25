"""走り方の速度の上限を、Nav2 の controller_server の速度の上限 (nav2_msgs/SpeedLimit) に出す。

周期ごとに出し続ける (controller_server は届いた上限をそのまま使い、後から起動しても次の周期で届く)。
上限がないときは、上限なし (0) を出す。controller_server の後ろの behavior_server (首振り・後退) は、
上限の対象にならない。
"""
from typing import Any, Mapping

from localization_supervisor.core.interfaces import ParamSpec
from localization_supervisor.core.supervisor import StepResult
from localization_supervisor.ros.actuators import Actuator
from nav2_msgs.msg import SpeedLimit

# nav2_costmap_2d の NO_SPEED_LIMIT と同じ
NO_SPEED_LIMIT = 0.0


class SpeedLimitActuator(Actuator):
    PARAMS = {
        'topic': ParamSpec(str, '/speed_limit'),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params, node)
        self._pub = node.create_publisher(SpeedLimit, self.params['topic'], 10)

    def apply(self, result: StepResult) -> None:
        conduct = result.conduct
        limit = NO_SPEED_LIMIT if conduct is None else float(conduct.speed_limit_mps)
        msg = SpeedLimit()
        msg.percentage = False
        msg.speed_limit = limit if limit > 0.0 else NO_SPEED_LIMIT
        self._pub.publish(msg)
