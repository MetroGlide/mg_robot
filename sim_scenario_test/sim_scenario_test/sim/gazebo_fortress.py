from __future__ import annotations

import subprocess
import urllib.parse
from typing import TYPE_CHECKING

from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.geometry import Pose, quaternion_from_yaw
from sim_scenario_test.registry import register_backend
from sim_scenario_test.sim.base import SimulationBackend
from sim_scenario_test.sim.model import ModelSpec

if TYPE_CHECKING:
    import rclpy.node

_FUEL_BASE_URL = "https://fuel.gazebosim.org/1.0"


@register_backend("gazebo_fortress")
class GazeboFortressBackend(SimulationBackend):
    """Gazebo Fortress 向けバックエンド。

    `ign service` CLI をサブプロセス経由で呼び出す (Humble + Fortress では
    Python の gz-transport バインディングが利用できないため)。
    ワールドに UserCommands システムプラグインが必要。
    create/remove サービスの成功応答は「要求を受理した」ことを意味し、
    実体の生成・削除完了までは保証しない。
    """

    def __init__(
        self,
        node: "rclpy.node.Node",
        world_name: str,
        timeout_ms: int = 5000,
    ):
        super().__init__(node, world_name)
        self._timeout_ms = timeout_ms

    def is_ready(self) -> bool:
        try:
            result = subprocess.run(
                ["ign", "service", "-l"],
                capture_output=True,
                text=True,
                timeout=self._timeout_ms / 1000.0 + 2.0,
            )
        except subprocess.TimeoutExpired:
            return False
        except FileNotFoundError as e:
            raise ScenarioError("'ign' command not found (Gazebo Fortress required)") from e
        return f"/world/{self._world}/create" in result.stdout.split()

    def entity_exists(self, name: str) -> bool:
        try:
            result = subprocess.run(
                ["ign", "model", "--list"],
                capture_output=True, text=True,
                timeout=self._timeout_ms / 1000.0 + 2.0)
        except subprocess.TimeoutExpired as e:
            raise ScenarioError("ign model --list timed out") from e
        except FileNotFoundError as e:
            raise ScenarioError("'ign' command not found (Gazebo Fortress required)") from e
        names = [line.strip()[2:] for line in result.stdout.splitlines()
                 if line.strip().startswith("- ")]
        return name in names

    def set_entity_pose(self, name: str, pose: Pose) -> None:
        req = f'name: "{name}" {_pose_fields(pose)}'
        self._call_service(
            f"/world/{self._world}/set_pose",
            "ignition.msgs.Pose", "ignition.msgs.Boolean", req)

    def spawn_entity(self, name: str, model: ModelSpec, pose: Pose) -> None:
        if model.type == "fuel":
            sdf = _include_sdf(name, _fuel_url(model.uri), model.static)
        elif model.type == "local":
            sdf = _read_sdf(model.path)
        else:
            sdf = _primitive_sdf(name, model)
        escaped = sdf.replace("\\", "\\\\").replace('"', '\\"').replace("\n", " ")
        req = (
            f'sdf: "{escaped}" name: "{name}" allow_renaming: false '
            f"pose {{ {_pose_fields(pose)} }}"
        )
        self._call_service(
            f"/world/{self._world}/create",
            "ignition.msgs.EntityFactory", "ignition.msgs.Boolean", req)

    def remove_entity(self, name: str) -> None:
        self._call_service(
            f"/world/{self._world}/remove",
            "ignition.msgs.Entity", "ignition.msgs.Boolean",
            f'name: "{name}" type: MODEL')

    def _call_service(self, service: str, req_type: str, rep_type: str, req: str) -> None:
        cmd = [
            "ign", "service",
            "-s", service,
            "--reqtype", req_type,
            "--reptype", rep_type,
            "--timeout", str(self._timeout_ms),
            "--req", req,
        ]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True,
                timeout=self._timeout_ms / 1000.0 + 2.0)
        except subprocess.TimeoutExpired as e:
            raise ScenarioError(f"gz service {service} timed out") from e
        except FileNotFoundError as e:
            raise ScenarioError("'ign' command not found (Gazebo Fortress required)") from e
        if result.returncode != 0:
            raise ScenarioError(
                f"gz service {service} failed: {result.stderr.strip()}")
        if result.stdout.strip() == "data: false":
            raise ScenarioError(f"gz service {service} returned false")


def _pose_fields(pose: Pose) -> str:
    qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
    return (
        f"position {{ x: {pose.x} y: {pose.y} z: {pose.z} }} "
        f"orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }}"
    )


def _fuel_url(uri: str) -> str:
    if uri.startswith("http"):
        return uri
    owner, sep, name = uri.partition("/models/")
    if not sep:
        return uri
    return f"{_FUEL_BASE_URL}/{owner}/models/{urllib.parse.quote(name)}"


def _include_sdf(name: str, uri: str, static: bool) -> str:
    """モデルを include で読み込み、static 指定を上書きする。"""
    return (
        f'<sdf version="1.6"><include><uri>{uri}</uri><name>{name}</name>'
        f'<static>{"true" if static else "false"}</static></include></sdf>'
    )


def _read_sdf(path: str) -> str:
    try:
        with open(path, "r") as f:
            return f.read()
    except OSError as e:
        raise ScenarioError(f"failed to read SDF '{path}': {e}") from e


def _primitive_sdf(name: str, model: ModelSpec) -> str:
    size = model.size
    if model.shape == "box":
        geom = (f"<box><size>{size.get('x', 1.0)} {size.get('y', 1.0)} "
                f"{size.get('z', 1.0)}</size></box>")
    elif model.shape == "cylinder":
        geom = (f"<cylinder><radius>{size.get('radius', 0.5)}</radius>"
                f"<length>{size.get('length', 1.0)}</length></cylinder>")
    else:
        geom = f"<sphere><radius>{size.get('radius', 0.5)}</radius></sphere>"
    static = "true" if model.static else "false"
    return (
        f'<sdf version="1.6"><model name="{name}"><static>{static}</static>'
        f'<link name="link">'
        f'<collision name="collision"><geometry>{geom}</geometry></collision>'
        f'<visual name="visual"><geometry>{geom}</geometry></visual>'
        f"</link></model></sdf>"
    )
