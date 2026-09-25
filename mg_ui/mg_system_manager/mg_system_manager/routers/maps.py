import datetime
import os
from pathlib import Path

from fastapi import APIRouter, Depends
from pydantic import BaseModel

from mg_system_manager.config import (
    MAP_NAME_RE,
    MAP_PREVIEW_SERVICE,
    PATH_RE,
    REOPTIMIZE_SERVICE,
)
from mg_system_manager.dependencies import get_runner
from mg_system_manager.docker_ops import ComposeRunner
from mg_system_manager.responses import result
from mg_system_manager.ros_cmd import run_local_ros2_cmd

router = APIRouter()

_MAP_SAVER_OPTIONS = "--occ 0.65 --free 0.15 --mode trinary"


def _save_grid_map(map_dir: str, map_name: str) -> tuple[bool, str]:
    # map_saver_cli は /map トピックから直接地図画像を書き出すため、
    # サービス呼び出しのタイムアウトに依存しない
    return run_local_ros2_cmd(
        f"ros2 run nav2_map_server map_saver_cli -t map "
        f"-f '{map_dir}/{map_name}' {_MAP_SAVER_OPTIONS}")


def save_common_map(map_dir: str, map_name: str) -> tuple[bool, str]:
    if (Path(map_dir) / f"{map_name}.pgm").exists() or \
            (Path(map_dir) / f"{map_name}.yaml").exists():
        return False, f"map already exists: {map_dir}/{map_name}"
    try:
        os.makedirs(map_dir, exist_ok=True)
    except Exception as e:
        return False, f"Failed to create directory: {e}"
    return _save_grid_map(map_dir, map_name)


def save_slam_gnss_2d_map(slam_map_dir: str) -> tuple[bool, str]:
    try:
        os.makedirs(slam_map_dir, exist_ok=True)
    except Exception as e:
        return False, f"Failed to create directory: {e}"

    ok, out = _save_grid_map(slam_map_dir, "map")
    if not ok:
        return False, f"Failed to save map via map_saver_cli: {out}"

    for node in ["/slam_gnss_2d_node", "/slam_gnss_2d_offline_node",
                 "/reoptimize_node"]:
        run_local_ros2_cmd(
            f"ros2 param set {node} save_dir '{slam_map_dir}'")

    ok, out = run_local_ros2_cmd(
        f"ros2 service call /slam_gnss_2d/save_slam_map "
        f"slam_gnss_2d_msgs/srv/SaveSlamMap \"{{map_dir: '{slam_map_dir}'}}\"")
    if not ok:
        return False, f"Failed to call /slam_gnss_2d/save_slam_map: {out}"
    return True, "SLAM map saved successfully"


def list_slam_gnss_2d_maps(base_dir: str) -> list[str]:
    path = Path(base_dir)
    if not path.is_dir():
        return []
    return [d.name for d in path.iterdir() if d.is_dir()]


class SaveCommonMapRequest(BaseModel):
    map_dir: str = "/root/ros2_data"
    map_name: str = "map"


@router.post("/map/common/save")
def post_save_common_map(body: SaveCommonMapRequest):
    if not PATH_RE.match(body.map_dir):
        return result(False, "invalid map_dir")
    if not MAP_NAME_RE.match(body.map_name):
        return result(False, "invalid map_name")
    return result(*save_common_map(body.map_dir, body.map_name))


class SaveSlamGnss2DMapRequest(BaseModel):
    output_dir: str = "/root/ros2_data/slam_maps"


@router.post("/slam_gnss_2d/map/save")
def post_save_slam_gnss_2d_map(body: SaveSlamGnss2DMapRequest):
    if not PATH_RE.match(body.output_dir):
        return result(False, "invalid output_dir")
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return result(*save_slam_gnss_2d_map(
        os.path.join(body.output_dir, timestamp)))


@router.get("/slam_gnss_2d/maps")
def get_slam_gnss_2d_maps(base_dir: str = "/root/ros2_data/slam_maps"):
    if not PATH_RE.match(base_dir):
        return {"success": False, "message": "invalid base_dir", "maps": []}
    return {"success": True, "maps": list_slam_gnss_2d_maps(base_dir)}


class SlamGnss2DPreviewStartRequest(BaseModel):
    slam_map_path: str


@router.post("/slam_gnss_2d/preview/start")
def start_slam_gnss_2d_preview(
    body: SlamGnss2DPreviewStartRequest,
    runner: ComposeRunner = Depends(get_runner),
):
    if not PATH_RE.match(body.slam_map_path):
        return result(False, "invalid slam_map_path")
    return result(*runner.up(
        MAP_PREVIEW_SERVICE, {"SLAM_MAP_DIR": body.slam_map_path}))


@router.post("/slam_gnss_2d/preview/stop")
def stop_slam_gnss_2d_preview(runner: ComposeRunner = Depends(get_runner)):
    return result(*runner.stop(MAP_PREVIEW_SERVICE))


class SlamGnss2DReoptimizeStartRequest(BaseModel):
    input_dir: str
    bag_path: str = ""
    save_dir: str = ""


@router.post("/slam_gnss_2d/reoptimize/start")
def start_slam_gnss_2d_reoptimize(
    body: SlamGnss2DReoptimizeStartRequest,
    runner: ComposeRunner = Depends(get_runner),
):
    if not PATH_RE.match(body.input_dir):
        return result(False, "invalid input_dir")
    if body.bag_path and not PATH_RE.match(body.bag_path):
        return result(False, "invalid bag_path")
    if body.save_dir and not PATH_RE.match(body.save_dir):
        return result(False, "invalid save_dir")
    return result(*runner.up(REOPTIMIZE_SERVICE, {
        "INPUT_DIR": body.input_dir,
        "BAG_PATH": body.bag_path,
        "SAVE_DIR": body.save_dir,
    }))


@router.post("/slam_gnss_2d/reoptimize/stop")
def stop_slam_gnss_2d_reoptimize(runner: ComposeRunner = Depends(get_runner)):
    return result(*runner.stop(REOPTIMIZE_SERVICE))
