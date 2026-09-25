"""シナリオテストの実行・停止と、進み具合・結果の参照。

実行は `docker compose run -d scenario-test` で起動し、sim_scenario_test の CLI に
SCENARIO_ARGS を渡す(Makefile の scenario-test / scenario-test-attach と同じ引数)。
attach モード用の環境(シミュレータとスタック)は scenario-env サービスで起動する。
scenario-test / scenario-env は SERVICES の汎用ルート(/<service>/start など)と
重ならないよう、/scenario/ の下に置く。
"""
import logging
import time

from fastapi import APIRouter, Depends, HTTPException, Query
from pydantic import BaseModel

from mg_system_manager import scenario_results
from mg_system_manager.config import (
    GAZEBO_SERVICE,
    HARDWARE_SERVICE_KEYS,
    HOST_RE,
    SCENARIO_ENV_SERVICE,
    SCENARIO_NAME_RE,
    SCENARIO_TEST_SERVICE,
    Settings,
)
from mg_system_manager.dependencies import get_runner, get_settings
from mg_system_manager.docker_ops import BusyError, ComposeRunner
from mg_system_manager.responses import result

logger = logging.getLogger(__name__)

router = APIRouter()

_MAX_SCENARIOS = 100
_MAX_REPEAT = 20
_START_TIMEOUT_S = 60
_ENV_START_TIMEOUT_S = 120
# scenario-test を止めるときの猶予。過ぎたら強制終了する
_STOP_TIMEOUT_S = 10
_REMOTE_CYCLONEDDS_URI = "file:///app/docker/cyclonedds/remote.xml"
_REMOTE_STACK_PORT = 8001


class ScenarioTestStartRequest(BaseModel):
    scenarios: list[str]
    repeat: int = 1
    gui: bool = False
    # 起動済みの環境(scenario-env など)に接続して実行する
    attach: bool = False
    # 実機PCのアドレス。指定するとナビゲーションスタックを実機PCで起動する
    robot: str = ""


class ScenarioEnvStartRequest(BaseModel):
    gui: bool = False


def _validate_start(body: ScenarioTestStartRequest) -> str | None:
    if not body.scenarios:
        return "no scenarios selected"
    if len(body.scenarios) > _MAX_SCENARIOS:
        return f"too many scenarios (max {_MAX_SCENARIOS})"
    for name in body.scenarios:
        if not SCENARIO_NAME_RE.match(name):
            return f"invalid scenario name: {name}"
    if not 1 <= body.repeat <= _MAX_REPEAT:
        return f"repeat must be between 1 and {_MAX_REPEAT}"
    if body.robot and not HOST_RE.match(body.robot):
        return "invalid robot address"
    if body.robot and body.attach:
        return "robot and attach cannot be used together"
    return None


def _check_running_services(
    status: dict[str, str], body: ScenarioTestStartRequest
) -> str | None:
    """同時に動いていると干渉するサービスと、attach の前提を確認する。"""
    if status.get(SCENARIO_TEST_SERVICE) == "running":
        return "scenario test is already running"
    if body.attach:
        if not any(status.get(s) == "running"
                   for s in (SCENARIO_ENV_SERVICE, GAZEBO_SERVICE)):
            return (f"attach needs a running simulator and stack "
                    f"({SCENARIO_ENV_SERVICE} or {GAZEBO_SERVICE})")
        return None
    # シナリオごとにシミュレータを起動するため、同じワールドのシミュレータや
    # 同じドメインのナビゲーションが動いていると干渉する
    conflicting = [
        s for s in (SCENARIO_ENV_SERVICE, GAZEBO_SERVICE, *HARDWARE_SERVICE_KEYS)
        if status.get(s) == "running"]
    if conflicting:
        return f"stop these services first: {conflicting}"
    return None


def build_scenario_args(
    body: ScenarioTestStartRequest, settings: Settings, results_dir: str
) -> list[str]:
    """scenario_cli.py に渡す引数。"""
    args = ["run-all", *body.scenarios]
    for directory in settings.scenario_dirs:
        args += ["--scenario-dir", directory]
    args += ["--results-dir", results_dir]
    if body.repeat > 1:
        args += ["--repeat", str(body.repeat)]
    if body.attach:
        args.append("--attach")
    elif body.gui:
        args.append("--gui")
    if body.robot:
        args += ["--remote-stack",
                 f"http://{body.robot}:{_REMOTE_STACK_PORT}"]
    return args


def remote_env(body: ScenarioTestStartRequest, settings: Settings) -> list[str]:
    """docker compose run の -e。実機PCと DDS でつなぐ設定で、Makefile の ROBOT= と同じ。"""
    if not body.robot:
        return []
    env = {
        "ROS_DOMAIN_ID": settings.scenario_ros_domain_id,
        "CYCLONEDDS_URI": _REMOTE_CYCLONEDDS_URI,
        "MG_REMOTE_PEER": body.robot,
    }
    return [option for k, v in env.items() for option in ("-e", f"{k}={v}")]


def _start(
    runner: ComposeRunner, settings: Settings, body: ScenarioTestStartRequest
) -> dict:
    run_id = time.strftime("%Y%m%d_%H%M%S")
    results_dir = str(settings.scenario_results_dir / run_id)
    try:
        with runner.operation(SCENARIO_TEST_SERVICE):
            error = _check_running_services(runner.get_status(fresh=True), body)
            if error:
                return result(False, error)
            # 前回の実行で残ったコンテナを片付ける(ログを見られるよう --rm は付けない)
            runner.remove_stopped(SCENARIO_TEST_SERVICE)
            scenario_args = " ".join(
                build_scenario_args(body, settings, results_dir))
            logger.info("start_scenario_test run_id=%s args=%s",
                        run_id, scenario_args)
            ok, msg = runner.compose(
                ["run", "-d", *remote_env(body, settings),
                 "-e", f"SCENARIO_ARGS={scenario_args}",
                 SCENARIO_TEST_SERVICE],
                timeout=_START_TIMEOUT_S)
    except BusyError as e:
        return result(False, str(e))
    return {**result(ok, msg), "run_id": run_id if ok else ""}


@router.get("/scenario/list")
def list_scenarios(settings: Settings = Depends(get_settings)):
    return {"scenarios": scenario_results.list_scenarios(settings.scenario_dirs)}


@router.post("/scenario/run/start")
def scenario_test_start(
    body: ScenarioTestStartRequest,
    runner: ComposeRunner = Depends(get_runner),
    settings: Settings = Depends(get_settings),
):
    error = _validate_start(body)
    if error:
        return result(False, error)
    return _start(runner, settings, body)


@router.post("/scenario/run/stop")
def scenario_test_stop(runner: ComposeRunner = Depends(get_runner)):
    return result(*runner.stop(SCENARIO_TEST_SERVICE, timeout=_STOP_TIMEOUT_S))


@router.get("/scenario/status")
def scenario_test_status(
    runner: ComposeRunner = Depends(get_runner),
    settings: Settings = Depends(get_settings),
):
    status = runner.get_status()
    running = status.get(SCENARIO_TEST_SERVICE) == "running"
    return {
        "services": {
            s: status.get(s)
            for s in (SCENARIO_TEST_SERVICE, SCENARIO_ENV_SERVICE, GAZEBO_SERVICE)
        },
        "run": scenario_results.latest_run(
            settings.scenario_results_dir, running),
    }


@router.get("/scenario/runs")
def scenario_test_runs(
    limit: int = Query(30, ge=1, le=200),
    runner: ComposeRunner = Depends(get_runner),
    settings: Settings = Depends(get_settings),
):
    running = runner.get_status().get(SCENARIO_TEST_SERVICE) == "running"
    return {"runs": scenario_results.list_runs(
        settings.scenario_results_dir, limit, running)}


@router.get("/scenario/runs/{run_id}")
def scenario_test_run(
    run_id: str,
    runner: ComposeRunner = Depends(get_runner),
    settings: Settings = Depends(get_settings),
):
    root = settings.scenario_results_dir
    progress = scenario_results.read_progress(root, run_id)
    if progress is None:
        raise HTTPException(status_code=404, detail="run not found")
    ids = scenario_results.run_ids(root)
    active = (runner.get_status().get(SCENARIO_TEST_SERVICE) == "running"
              and bool(ids) and ids[0] == run_id)
    return {**progress, "run_id": run_id,
            "state": scenario_results.run_state(progress, active)}


@router.get("/scenario/runs/{run_id}/{entry}/result")
def scenario_test_result(
    run_id: str, entry: str, settings: Settings = Depends(get_settings)
):
    path = scenario_results.entry_file(
        settings.scenario_results_dir, run_id, entry,
        scenario_results.RESULT_FILE)
    data = scenario_results.read_result(path) if path is not None else None
    if data is None:
        raise HTTPException(status_code=404, detail="result not found")
    return data


@router.get("/scenario/runs/{run_id}/{entry}/log")
def scenario_test_log(
    run_id: str,
    entry: str,
    name: str = Query("launch", pattern="^(launch|stack)$"),
    tail: int = Query(500, ge=1, le=5000),
    settings: Settings = Depends(get_settings),
):
    path = scenario_results.entry_file(
        settings.scenario_results_dir, run_id, entry,
        scenario_results.LOG_FILES[name])
    text = scenario_results.tail_lines(path, tail) if path is not None else None
    if text is None:
        raise HTTPException(status_code=404, detail="log not found")
    return {"log": text}


@router.post("/scenario/env/start")
def scenario_env_start(
    body: ScenarioEnvStartRequest,
    runner: ComposeRunner = Depends(get_runner),
    settings: Settings = Depends(get_settings),
):
    status = runner.get_status(fresh=True)
    if status.get(SCENARIO_TEST_SERVICE) == "running":
        return result(False, "scenario test is running")
    conflicting = [
        s for s in (GAZEBO_SERVICE, *HARDWARE_SERVICE_KEYS)
        if status.get(s) == "running"]
    if conflicting:
        return result(False, f"stop these services first: {conflicting}")
    env_args = (f"profile:={settings.scenario_profile} "
                f"world:={settings.simulation_world} "
                f"headless:={'false' if body.gui else 'true'}")
    return result(*runner.up(
        SCENARIO_ENV_SERVICE, {"SCENARIO_ENV_ARGS": env_args},
        timeout=_ENV_START_TIMEOUT_S, extra_args=("--force-recreate",)))


@router.post("/scenario/env/stop")
def scenario_env_stop(runner: ComposeRunner = Depends(get_runner)):
    return result(*runner.stop(SCENARIO_ENV_SERVICE))
