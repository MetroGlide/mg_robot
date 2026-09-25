"""環境変数からの設定と、操作対象サービスの定義。"""
import os
import re
from dataclasses import dataclass
from pathlib import Path

DEFAULT_ALLOWED_ORIGINS = [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
]


@dataclass(frozen=True)
class ServiceSpec:
    """docker compose のサービス 1 つ分の定義。

    layer は UI の System ページでの表示グループ。None の場合は UI に表示しない。
    hardware が True のサービスは実機のハードウェアを使うため、
    動作中はシナリオテスト用スタックを起動しない。
    """

    key: str
    label: str
    layer: str | None = None
    hardware: bool = False


SERVICE_LAYERS = (
    ("core", "Core"),
    ("function", "Function"),
    ("tool", "Tool"),
)

SERVICES = (
    ServiceSpec("foxglove-bridge", "Foxglove Bridge", "core"),
    ServiceSpec("diagnostics", "Diagnostics", "core"),
    ServiceSpec("navigation", "Navigation", "function", hardware=True),
    ServiceSpec("slam", "SLAM", "function", hardware=True),
    ServiceSpec("slam-gnss-2d", "SLAM GNSS 2D", hardware=True),
    ServiceSpec("waypoint-editor", "Waypoint Editor", "tool"),
    ServiceSpec("gazebo-simulation", "Gazebo Simulation", "tool"),
    ServiceSpec("rviz2", "RViz2", "tool"),
    ServiceSpec("rviz2-navigation", "RViz2 Navigation", "tool"),
    ServiceSpec("rviz2-slam", "RViz2 SLAM", "tool"),
    ServiceSpec("scenario-test", "Scenario Test"),
    ServiceSpec("map-preview", "Map Preview"),
    ServiceSpec("reoptimize-slam", "Reoptimize SLAM"),
    ServiceSpec("scenario-remote-stack", "Scenario Remote Stack"),
)

SERVICE_KEYS = frozenset(spec.key for spec in SERVICES)
HARDWARE_SERVICE_KEYS = tuple(spec.key for spec in SERVICES if spec.hardware)

SCENARIO_STACK_SERVICE = "scenario-remote-stack"
MAP_PREVIEW_SERVICE = "map-preview"
REOPTIMIZE_SERVICE = "reoptimize-slam"
ROSBAG_REPLAY_SERVICE = "rosbag-replay"
GAZEBO_SERVICE = "gazebo-simulation"


def _split_csv(value: str | None) -> list[str]:
    if not value:
        return []
    return [item.strip() for item in value.split(",") if item.strip()]


@dataclass(frozen=True)
class Settings:
    project_dir: str
    host_project_dir: str
    host_home: str
    simulation_world: str
    simulation_robot_name: str
    settings_dir: Path
    allowed_origins: list[str]
    scenario_stack_allowed_packages: list[str]

    @classmethod
    def from_env(cls) -> "Settings":
        project_dir = os.environ.get("PROJECT_DIR", "/app")
        return cls(
            project_dir=project_dir,
            host_project_dir=os.environ.get("HOST_PROJECT_DIR", project_dir),
            host_home=os.environ.get("HOST_HOME", os.environ.get("HOME", "/root")),
            simulation_world=os.environ.get("SIMULATION_WORLD", "warehouse"),
            simulation_robot_name=os.environ.get("SIMULATION_ROBOT_NAME", "mg"),
            settings_dir=Path(
                os.environ.get("UI_DATA_DIR", "/root/ros2_data/mg_ui_local")),
            allowed_origins=(
                _split_csv(os.environ.get("SYSTEM_MANAGER_ALLOW_ORIGINS"))
                or DEFAULT_ALLOWED_ORIGINS),
            scenario_stack_allowed_packages=(
                _split_csv(os.environ.get("SCENARIO_STACK_ALLOWED_PACKAGES"))
                or ["mg_bringup"]),
        )


PATH_RE = re.compile(r"^[a-zA-Z0-9/_\-\.]+$")
MAP_NAME_RE = re.compile(r"^[a-zA-Z0-9_\-]+$")
ARG_KEY_RE = re.compile(r"^[a-zA-Z_][a-zA-Z0-9_]*$")
