"""再初期化の後に、グローバルコストマップを消す。

ずれていた間に map 座標で記録した障害物が、実在しない位置に残るため。
"""
from localization_supervisor.core.interfaces import ParamSpec
from localization_supervisor.ros.actuators.standard import ServiceCall
from nav2_msgs.srv import ClearEntireCostmap


class ClearCostmap(ServiceCall):
    SERVICE_TYPES = {'clear': ClearEntireCostmap}
    PARAMS = {
        **ServiceCall.PARAMS,
        'service': ParamSpec(str, '/global_costmap/clear_entirely_global_costmap'),
        'type': ParamSpec(str, 'clear'),
        'delay_sec': ParamSpec(float, 1.0),
    }
