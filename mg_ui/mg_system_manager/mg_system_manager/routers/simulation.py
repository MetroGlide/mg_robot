import logging
import math

from fastapi import APIRouter, Depends
from pydantic import BaseModel

from mg_system_manager.config import GAZEBO_SERVICE, Settings
from mg_system_manager.dependencies import get_runner, get_settings
from mg_system_manager.docker_ops import ComposeRunner
from mg_system_manager.responses import result

logger = logging.getLogger(__name__)

router = APIRouter()


class ResetPoseRequest(BaseModel):
    x: float
    y: float
    z: float
    yaw: float


@router.post("/simulation/reset-pose")
def reset_sim_robot_pose(
    body: ResetPoseRequest,
    runner: ComposeRunner = Depends(get_runner),
    settings: Settings = Depends(get_settings),
):
    logger.info("reset_sim_robot_pose %s", body)
    qz = math.sin(body.yaw / 2.0)
    qw = math.cos(body.yaw / 2.0)
    request = (
        f'name: "{settings.simulation_robot_name}" '
        f"position {{ x: {body.x} y: {body.y} z: {body.z} }} "
        f"orientation {{ x: 0.0 y: 0.0 z: {qz} w: {qw} }}"
    )
    command = (
        f"ign service "
        f"-s /world/{settings.simulation_world}/set_pose "
        f"--reqtype ignition.msgs.Pose "
        f"--reptype ignition.msgs.Boolean "
        f"--timeout 5000 "
        f"--req '{request}'"
    )
    return result(*runner.exec_in_container(
        GAZEBO_SERVICE, ["bash", "-lc", command]))
