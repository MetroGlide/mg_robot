import logging
import re
from pathlib import Path

from fastapi import APIRouter, Depends
from pydantic import BaseModel

from mg_system_manager.config import PATH_RE, ROSBAG_REPLAY_SERVICE, Settings
from mg_system_manager.dependencies import get_runner, get_settings
from mg_system_manager.docker_ops import ComposeRunner
from mg_system_manager.responses import result

logger = logging.getLogger(__name__)

router = APIRouter()


def parse_env_file(path: Path) -> dict[str, str]:
    """シンプルな .env パーサー。${VAR} 形式の変数参照を展開する。"""
    env: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except Exception as e:
        logger.warning("parse_env_file failed path=%s: %s", path, e)
        return env
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        value = re.sub(
            r"\$\{([^}]+)\}",
            lambda m: env.get(m.group(1), ""),
            value.strip(),
        )
        env[key.strip()] = value
    return env


@router.get("/rosbag-replay/env")
def rosbag_replay_env(settings: Settings = Depends(get_settings)):
    env = parse_env_file(Path(settings.project_dir) / ".env")
    topics = [t for t in env.get("ROSBAG_TOPICS", "").split() if t]
    return {"file": env.get("ROSBAG_FILE", ""), "topics": topics}


class RosbagStartRequest(BaseModel):
    file: str
    topics: list[str] = []


@router.post("/rosbag-replay/start")
def rosbag_replay_start(
    body: RosbagStartRequest, runner: ComposeRunner = Depends(get_runner)
):
    if not PATH_RE.match(body.file):
        return result(False, "invalid file path")
    for topic in body.topics:
        if not PATH_RE.match(topic):
            return result(False, f"invalid topic: {topic}")
    return result(*runner.up(ROSBAG_REPLAY_SERVICE, {
        "ROSBAG_FILE": body.file,
        "ROSBAG_TOPICS": " ".join(body.topics),
    }))


@router.post("/rosbag-replay/stop")
def rosbag_replay_stop(runner: ComposeRunner = Depends(get_runner)):
    return result(*runner.stop(ROSBAG_REPLAY_SERVICE))
