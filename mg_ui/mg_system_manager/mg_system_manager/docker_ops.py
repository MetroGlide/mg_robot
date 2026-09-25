import logging
import os
import subprocess
import threading
from contextlib import contextmanager
from typing import Any, Iterator

import docker

from mg_system_manager.config import (
    HARDWARE_SERVICE_KEYS,
    SCENARIO_STACK_SERVICE,
    Settings,
)

logger = logging.getLogger(__name__)

_COMPOSE_SERVICE_LABEL = "com.docker.compose.service"
_COMPOSE_PROJECT_LABEL = "com.docker.compose.project"

# 実機のハードウェアを使うサービスとシナリオテスト用スタックは、同時に起動処理を走らせない
_EXCLUSIVE_SERVICES = frozenset((*HARDWARE_SERVICE_KEYS, SCENARIO_STACK_SERVICE))
_EXCLUSIVE_GROUP = "<hardware-exclusive>"


class BusyError(Exception):
    """同じサービスに対する別の操作が実行中であることを表す。"""


class ComposeRunner:
    """docker compose の実行とコンテナの検索を 1 か所にまとめる。"""

    def __init__(self, settings: Settings, client: Any = None) -> None:
        self._settings = settings
        self._client = client if client is not None else docker.from_env()
        self._locks: dict[str, threading.RLock] = {}
        self._locks_guard = threading.Lock()
        logger.info(
            "ComposeRunner initialized: host_project_dir=%s host_home=%s "
            "simulation_world=%s robot=%s",
            settings.host_project_dir,
            settings.host_home,
            settings.simulation_world,
            settings.simulation_robot_name,
        )

    def _lock_for(self, key: str) -> threading.RLock:
        with self._locks_guard:
            return self._locks.setdefault(key, threading.RLock())

    @contextmanager
    def operation(self, service: str, exclusive: bool = False) -> Iterator[None]:
        """サービスに対する操作の排他区間。

        同じサービスの操作が実行中なら待たずに BusyError にする(二重クリックや複数端末からの
        同時操作で compose が並行実行されるのを防ぐ)。exclusive を指定すると、
        ハードウェアを使うサービスとシナリオ用スタックの起動処理も互いに排他する。
        同じスレッドからの入れ子は許可する。
        """
        keys = [service]
        if exclusive and service in _EXCLUSIVE_SERVICES:
            keys.append(_EXCLUSIVE_GROUP)
        acquired: list[threading.RLock] = []
        try:
            for key in keys:
                lock = self._lock_for(key)
                if not lock.acquire(blocking=False):
                    raise BusyError(
                        f"another operation is in progress: {service}")
                acquired.append(lock)
            yield
        finally:
            for lock in reversed(acquired):
                lock.release()

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
        try:
            with self.operation(service, exclusive=True):
                return self.compose(
                    ["up", "-d", *extra_args, service], env_extra, timeout)
        except BusyError as e:
            return False, str(e)

    def restart(self, service: str) -> tuple[bool, str]:
        try:
            with self.operation(service, exclusive=True):
                return self.compose(["restart", service])
        except BusyError as e:
            return False, str(e)

    def _list(self, service: str | None = None) -> list:
        """このプロジェクトのコンテナを、動作中のものが先頭になる順で返す。

        docker compose run で作られる one-off コンテナも含める。
        別プロジェクトのコンテナは対象外にする。
        """
        labels = [f"{_COMPOSE_PROJECT_LABEL}={self._settings.compose_project}"]
        labels.append(
            f"{_COMPOSE_SERVICE_LABEL}={service}" if service
            else _COMPOSE_SERVICE_LABEL)
        containers = self._client.containers.list(
            all=True, filters={"label": labels})
        return sorted(containers, key=lambda c: c.status != "running")

    def get_container(self, service: str):
        containers = self._list(service)
        if not containers:
            logger.warning("container not found for service=%s", service)
            return None
        return containers[0]

    def get_status(self) -> dict[str, str]:
        status: dict[str, str] = {}
        for container in self._list():
            service = container.labels[_COMPOSE_SERVICE_LABEL]
            if status.get(service) != "running":
                status[service] = container.status
        return status

    def stop(self, service: str, timeout: int | None = None) -> tuple[bool, str]:
        try:
            with self.operation(service):
                return self._stop(service, timeout)
        except BusyError as e:
            return False, str(e)

    def _stop(self, service: str, timeout: int | None) -> tuple[bool, str]:
        logger.info("stop service=%s", service)
        containers = self._list(service)
        if not containers:
            return False, f"container not found: {service}"
        targets = [c for c in containers if c.status == "running"]
        try:
            for container in targets or containers[:1]:
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
