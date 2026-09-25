import sys
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mg_system_manager import docker_ops  # noqa: E402
from mg_system_manager.app import create_app  # noqa: E402
from mg_system_manager.config import Settings  # noqa: E402
from mg_system_manager.docker_ops import ComposeRunner  # noqa: E402
from sm_fakes import ComposeCalls, FakeDockerClient  # noqa: E402


@pytest.fixture
def compose_calls(monkeypatch) -> ComposeCalls:
    calls = ComposeCalls()
    monkeypatch.setattr(docker_ops.subprocess, "run", calls)
    return calls


@pytest.fixture
def settings(tmp_path) -> Settings:
    return Settings(
        project_dir=str(tmp_path / "project"),
        host_project_dir="/host/project",
        host_home="/host/home",
        compose_project="mg",
        simulation_world="warehouse",
        simulation_robot_name="mg",
        settings_dir=tmp_path / "ui",
        allowed_origins=["http://localhost:8080"],
        scenario_stack_allowed_packages=["mg_bringup"],
        scenario_dirs=[str(tmp_path / "scenarios" / "regression"),
                       str(tmp_path / "scenarios" / "examples")],
        scenario_results_dir=tmp_path / "results",
        scenario_profile="mg01",
        scenario_ros_domain_id="42",
    )


@pytest.fixture
def containers() -> list:
    return []


@pytest.fixture
def runner(settings, containers) -> ComposeRunner:
    return ComposeRunner(settings, client=FakeDockerClient(containers))


@pytest.fixture
def client(settings, runner) -> TestClient:
    return TestClient(create_app(settings, runner=runner))
