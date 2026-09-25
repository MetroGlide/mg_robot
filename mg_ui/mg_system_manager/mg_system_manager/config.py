"""環境変数からの設定と、操作対象サービスの定義。"""
import os
import re
from dataclasses import dataclass
from pathlib import Path

DEFAULT_ALLOWED_ORIGINS = [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
]

# タブレットなど別端末から IP で Web UI を開く場合を許可する。
# Web UI の配信ポート(8080)と開発サーバ(5173)に限り、
# localhost・プライベート IP・Tailscale(100.64.0.0/10)・mDNS(.local) を対象にする。
_PRIVATE_HOST = (
    r"(localhost|127\.0\.0\.1"
    r"|10\.\d{1,3}\.\d{1,3}\.\d{1,3}"
    r"|192\.168\.\d{1,3}\.\d{1,3}"
    r"|172\.(1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3}"
    r"|100\.(6[4-9]|[7-9]\d|1[01]\d|12[0-7])\.\d{1,3}\.\d{1,3}"
    r"|[a-zA-Z0-9-]+\.local)"
)
DEFAULT_ORIGIN_REGEX = rf"^http://{_PRIVATE_HOST}:(8080|5173)$"


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
    ServiceSpec("scenario-env", "Scenario Env"),
)

SERVICE_KEYS = frozenset(spec.key for spec in SERVICES)
HARDWARE_SERVICE_KEYS = tuple(spec.key for spec in SERVICES if spec.hardware)

SCENARIO_STACK_SERVICE = "scenario-remote-stack"
SCENARIO_TEST_SERVICE = "scenario-test"
SCENARIO_ENV_SERVICE = "scenario-env"
MAP_PREVIEW_SERVICE = "map-preview"
REOPTIMIZE_SERVICE = "reoptimize-slam"
ROSBAG_REPLAY_SERVICE = "rosbag-replay"
GAZEBO_SERVICE = "gazebo-simulation"


def _split_csv(value: str | None) -> list[str]:
    if not value:
        return []
    return [item.strip() for item in value.split(",") if item.strip()]


def _default_compose_project(host_project_dir: str) -> str:
    """COMPOSE_PROJECT_NAME 未設定時に docker compose が使う名前を再現する。"""
    return re.sub(r"[^a-z0-9_-]", "", Path(host_project_dir).name.lower())


def _compose_files(use_gpu: str | None) -> tuple[str, ...]:
    """Makefile と同じく、USE_GPU に応じて GPU 用の compose ファイルを重ねる。"""
    if use_gpu in ("nvidia", "amd"):
        return ("compose.yaml", f"compose.gpu.{use_gpu}.yaml")
    return ()


@dataclass(frozen=True)
class Settings:
    project_dir: str
    host_project_dir: str
    host_home: str
    compose_project: str
    simulation_world: str
    simulation_robot_name: str
    settings_dir: Path
    allowed_origins: list[str]
    scenario_stack_allowed_packages: list[str]
    # シナリオテスト。パスは system_manager と scenario-test コンテナで共通のもの
    scenario_dirs: list[str]
    scenario_results_dir: Path
    scenario_profile: str
    scenario_ros_domain_id: str
    # docker compose に -f で渡すファイル。空なら compose の既定(compose.yaml)
    compose_files: tuple[str, ...] = ()

    def is_origin_allowed(self, origin: str) -> bool:
        return (origin in self.allowed_origins
                or re.match(DEFAULT_ORIGIN_REGEX, origin) is not None)

    @classmethod
    def from_env(cls) -> "Settings":
        project_dir = os.environ.get("PROJECT_DIR", "/app")
        host_project_dir = os.environ.get("HOST_PROJECT_DIR", project_dir)
        return cls(
            project_dir=project_dir,
            host_project_dir=host_project_dir,
            host_home=os.environ.get("HOST_HOME", os.environ.get("HOME", "/root")),
            compose_project=(
                os.environ.get("COMPOSE_PROJECT_NAME")
                or _default_compose_project(host_project_dir)),
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
            scenario_dirs=(
                _split_csv(os.environ.get("SCENARIO_DIRS"))
                or [f"{project_dir}/mg_scenario_test/scenarios/regression",
                    f"{project_dir}/mg_scenario_test/scenarios/examples"]),
            scenario_results_dir=Path(os.environ.get(
                "SCENARIO_RESULTS_DIR", "/root/ros2_data/scenario_results")),
            scenario_profile=os.environ.get("SCENARIO_PROFILE", "mg01"),
            scenario_ros_domain_id=(
                os.environ.get("SCENARIO_ROS_DOMAIN_ID") or "42"),
            compose_files=_compose_files(os.environ.get("USE_GPU")),
        )


PATH_RE = re.compile(r"^[a-zA-Z0-9/_\-\.]+$")
MAP_NAME_RE = re.compile(r"^[a-zA-Z0-9_\-]+$")
ARG_KEY_RE = re.compile(r"^[a-zA-Z_][a-zA-Z0-9_]*$")
# シナリオ名・run の id・結果ディレクトリ名
SCENARIO_NAME_RE = re.compile(r"^[a-zA-Z0-9_\-]+$")
# 実機PCのアドレス (IPv4 またはホスト名)
HOST_RE = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9\.\-]*$")
