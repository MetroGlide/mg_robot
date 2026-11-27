"""実行時のノードのパラメータを取得して、結果に残す (どの構成で走ったかを結果から読めるようにする)。"""
from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, List

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.nav2 import call_service

if TYPE_CHECKING:
    import rclpy.node

# 取得できなかったノードに入れる値
UNAVAILABLE = "unavailable"

_ARRAY_FIELDS = {
    ParameterType.PARAMETER_BYTE_ARRAY: "byte_array_value",
    ParameterType.PARAMETER_BOOL_ARRAY: "bool_array_value",
    ParameterType.PARAMETER_INTEGER_ARRAY: "integer_array_value",
    ParameterType.PARAMETER_DOUBLE_ARRAY: "double_array_value",
    ParameterType.PARAMETER_STRING_ARRAY: "string_array_value",
}
_SCALAR_FIELDS = {
    ParameterType.PARAMETER_BOOL: "bool_value",
    ParameterType.PARAMETER_INTEGER: "integer_value",
    ParameterType.PARAMETER_DOUBLE: "double_value",
    ParameterType.PARAMETER_STRING: "string_value",
}


def parameter_value(value) -> Any:
    """rcl_interfaces の ParameterValue を JSON にできる値にする。未設定は None。"""
    if value.type in _SCALAR_FIELDS:
        return getattr(value, _SCALAR_FIELDS[value.type])
    if value.type in _ARRAY_FIELDS:
        return list(getattr(value, _ARRAY_FIELDS[value.type]))
    return None


def read_parameters(
    node: "rclpy.node.Node", targets: Dict[str, List[str]], timeout_sec: float
) -> Dict[str, Any]:
    """ノード名 -> パラメータ名の並び、からパラメータの値を取得する。

    取得できなかったノードは {"status": UNAVAILABLE, "reason": ...} にする (合否には使わない記録なので、失敗で止めない)。
    """
    values: Dict[str, Any] = {}
    for node_name, names in targets.items():
        service = f"/{node_name.lstrip('/')}/get_parameters"
        client = node.create_client(GetParameters, service)
        try:
            response = call_service(node, client, GetParameters.Request(names=list(names)), timeout_sec)
            values[node_name] = {
                name: parameter_value(value) for name, value in zip(names, response.values)}
        except ScenarioError as e:
            values[node_name] = {"status": UNAVAILABLE, "reason": str(e)}
        finally:
            node.destroy_client(client)
    return values
