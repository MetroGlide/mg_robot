import logging
import os
import subprocess
from typing import Any

import docker

from mg_system_manager.config import Settings

logger = logging.getLogger(__name__)

_COMPOSE_SERVICE_LABEL = "com.docker.compose.service"


class ComposeRunner:
    """docker compose の実行とコンテナの検索を 1 か所にまとめる。"""

    def __init__(self, settings: Settings, client: Any = None) -> None:
        self._settings = settings
        self._client = client if client is not None else docker.from_env()
        logger.info(
            "ComposeRunner initialized: host_project_dir=%s host_home=%s "
            "simulation_world=%s robot=%s",
            settings.host_project_dir,
            settings.host_home,
            settings.simulation_world,
            settings.simulation_robot_name,
        )

    def compose(
        self,
        args: list[str],
        env_extra: dict[str, str] | None = None,
        timeout: int = 60,
    ) -> tuple[bool, str]:
        env = os.environ.copy()
        env["HOME"] = self._settings.host_home
        if env_extra:
            env.update(env_extra)
        command = ["docker", "compose", *args]
        logger.info("%s (cwd=%s)", " ".join(command),
                    self._settings.host_project_dir)
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=self._settings.host_project_dir,
                env=env,
            )
        except subprocess.TimeoutExpired:
            logger.error("%s timed out", " ".join(command))
            return False, "command timed out"
        except Exception as e:
            logger.error("%s exception: %s", " ".join(command), e)
            return False, str(e)
        stdout = result.stdout.strip()
        stderr = result.stderr.strip()
        if result.returncode == 0:
            logger.info("%s succeeded stdout=%s", " ".join(command), stdout)
            return True, stdout
        logger.error("%s failed rc=%d stderr=%s", " ".join(command),
                     result.returncode, stderr)
        return False, stderr

    def up(
        self,
        service: str,
        env_extra: dict[str, str] | None = None,
        timeout: int = 60,
        extra_args: tuple[str, ...] = (),
    ) -> tuple[bool, str]:
        return self.compose(
            ["up", "-d", *extra_args, service], env_extra, timeout)

    def restart(self, service: str) -> tuple[bool, str]:
        return self.compose(["restart", service])

    def get_container(self, service: str):
        containers = self._client.containers.list(
            all=True,
            filters={"label": [f"{_COMPOSE_SERVICE_LABEL}={service}"]},
        )
        container = containers[0] if containers else None
        if container is None:
            logger.warning("container not found for service=%s", service)
        return container

    def get_status(self) -> dict[str, str]:
        containers = self._client.containers.list(
            all=True,
            filters={"label": [_COMPOSE_SERVICE_LABEL]},
        )
        return {
            c.labels[_COMPOSE_SERVICE_LABEL]: c.status for c in containers
        }

    def stop(self, service: str, timeout: int | None = None) -> tuple[bool, str]:
        logger.info("stop service=%s", service)
        container = self.get_container(service)
        if container is None:
            return False, f"container not found: {service}"
        try:
            if timeout is None:
                container.stop()
            else:
                container.stop(timeout=timeout)
        except Exception as e:
            logger.error("stop failed service=%s: %s", service, e)
            return False, str(e)
        logger.info("stopped service=%s", service)
        return True, ""

    def remove(self, service: str) -> tuple[bool, str]:
        container = self.get_container(service)
        if container is None:
            return True, "not found"
        try:
            container.remove()
        except Exception as e:
            logger.error("remove failed service=%s: %s", service, e)
            return False, str(e)
        return True, ""

    def exec_in_container(
        self, service: str, cmd: list[str]
    ) -> tuple[bool, str]:
        container = self.get_container(service)
        if container is None:
            return False, f"container not found: {service}"
        try:
            result = container.exec_run(cmd)
        except Exception as e:
            logger.error("exec_in_container exception service=%s: %s",
                         service, e)
            return False, str(e)
        output = result.output.decode(errors="replace")
        ok = result.exit_code == 0
        if ok:
            logger.info("exec_in_container succeeded service=%s: %s",
                        service, output[:200])
        else:
            logger.error(
                "exec_in_container failed service=%s exit_code=%d: %s",
                service, result.exit_code, output[:200])
        return ok, output

    def logs(self, service: str, tail: int) -> str:
        container = self.get_container(service)
        if container is None:
            return ""
        return container.logs(tail=tail).decode(errors="replace")
