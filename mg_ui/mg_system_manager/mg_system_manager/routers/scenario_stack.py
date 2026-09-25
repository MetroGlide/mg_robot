import logging

from fastapi import APIRouter, Depends
from pydantic import BaseModel

from mg_system_manager.config import (
    ARG_KEY_RE,
    HARDWARE_SERVICE_KEYS,
    PATH_RE,
    SCENARIO_STACK_SERVICE,
    Settings,
)
from mg_system_manager.dependencies import get_runner, get_settings
from mg_system_manager.docker_ops import ComposeRunner
from mg_system_manager.responses import result

logger = logging.getLogger(__name__)

router = APIRouter()

_LOG_TAIL_LINES = 20000
_STOP_TIMEOUT_S = 30
_START_TIMEOUT_S = 120


class ScenarioStackStartRequest(BaseModel):
    package: str
    file: str
    args: dict[str, str] = {}


def _start(
    runner: ComposeRunner, package: str, file: str, args: dict[str, str]
) -> tuple[bool, str]:
    status = runner.get_status()
    running = [s for s in HARDWARE_SERVICE_KEYS if status.get(s) == "running"]
    if running:
        return False, f"hardware services are running: {running}"
    logger.info("start_scenario_stack package=%s file=%s args=%s",
                package, file, args)
    return runner.up(
        SCENARIO_STACK_SERVICE,
        {
            "STACK_PACKAGE": package,
            "STACK_FILE": file,
            "STACK_ARGS": " ".join(f"{k}:={v}" for k, v in args.items()),
        },
        timeout=_START_TIMEOUT_S,
        extra_args=("--force-recreate",),
    )


def _stop(runner: ComposeRunner) -> tuple[bool, str]:
    if runner.get_container(SCENARIO_STACK_SERVICE) is None:
        return True, "not running"
    ok, msg = runner.stop(SCENARIO_STACK_SERVICE, timeout=_STOP_TIMEOUT_S)
    if not ok:
        return False, msg
    return runner.remove(SCENARIO_STACK_SERVICE)


@router.post("/scenario-stack/start")
def scenario_stack_start(
    body: ScenarioStackStartRequest,
    runner: ComposeRunner = Depends(get_runner),
    settings: Settings = Depends(get_settings),
):
    if body.package not in settings.scenario_stack_allowed_packages:
        return result(False, f"package not allowed: {body.package}")
    if not PATH_RE.match(body.file) or ".." in body.file:
        return result(False, "invalid file")
    for key, value in body.args.items():
        if not ARG_KEY_RE.match(key):
            return result(False, f"invalid arg name: {key}")
        if not PATH_RE.match(value):
            return result(False, f"invalid arg value for {key}")
    return result(*_start(runner, body.package, body.file, body.args))


@router.post("/scenario-stack/stop")
def scenario_stack_stop(runner: ComposeRunner = Depends(get_runner)):
    return result(*_stop(runner))


@router.get("/scenario-stack/logs")
def scenario_stack_logs(runner: ComposeRunner = Depends(get_runner)):
    return {"logs": runner.logs(SCENARIO_STACK_SERVICE, _LOG_TAIL_LINES)}
