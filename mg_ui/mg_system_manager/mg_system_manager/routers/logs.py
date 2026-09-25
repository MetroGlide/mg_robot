import asyncio
import json
import logging
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from mg_system_manager.config import SERVICE_KEYS
from mg_system_manager.docker_ops import ComposeRunner

logger = logging.getLogger(__name__)

router = APIRouter()

_RETRY_INTERVAL_S = 5.0


async def _stream_container_logs(
    runner: ComposeRunner, queue: asyncio.Queue, service: str
) -> None:
    while True:
        container = runner.get_container(service)
        if container is None:
            await queue.put(
                {"service": service, "error": "container not found"}
            )
            await asyncio.sleep(_RETRY_INTERVAL_S)
            continue

        container_ref = container.name or container.id
        if not container_ref:
            await queue.put(
                {"service": service, "error": "container reference missing"}
            )
            await asyncio.sleep(_RETRY_INTERVAL_S)
            continue

        proc = await asyncio.create_subprocess_exec(
            "docker",
            "logs",
            "-f",
            "--since",
            "0s",
            container_ref,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )

        try:
            assert proc.stdout is not None
            async for line in proc.stdout:
                text = line.decode(errors="replace").rstrip("\n")
                if text:
                    await queue.put({"service": service, "line": text})
            await proc.wait()
            await asyncio.sleep(_RETRY_INTERVAL_S)
        except asyncio.CancelledError:
            if proc.returncode is None:
                proc.terminate()
                await proc.wait()
            raise


def _filter_log_services(raw_services: Any) -> set[str]:
    if not isinstance(raw_services, list):
        return set()
    return {
        service
        for service in raw_services
        if isinstance(service, str) and service in SERVICE_KEYS
    }


@router.websocket("/logs/stream")
async def logs_stream(websocket: WebSocket):
    state = websocket.app.state
    origin = websocket.headers.get("origin")
    if origin and origin not in state.settings.allowed_origins:
        await websocket.close(code=1008)
        return

    await websocket.accept()
    runner: ComposeRunner = state.runner
    send_queue: asyncio.Queue[dict] = asyncio.Queue()
    tasks: dict[str, asyncio.Task[None]] = {}

    async def _sender() -> None:
        while True:
            msg = await send_queue.get()
            await websocket.send_json(msg)

    sender_task: asyncio.Task[None] = asyncio.create_task(_sender())

    def _bind_done_callback(service: str):
        def _on_done(task: asyncio.Task[None]) -> None:
            if tasks.get(service) is task:
                tasks.pop(service, None)

        return _on_done

    async def _cancel_removed(removed_services: set[str]) -> None:
        removed_tasks: list[asyncio.Task[None]] = []
        for service in removed_services:
            task = tasks.pop(service, None)
            if task is None:
                continue
            task.cancel()
            removed_tasks.append(task)
        if removed_tasks:
            await asyncio.gather(*removed_tasks, return_exceptions=True)

    try:
        while True:
            raw_message = await websocket.receive_text()
            try:
                payload = json.loads(raw_message)
            except json.JSONDecodeError:
                continue

            desired_services = _filter_log_services(payload.get("services"))
            current_services = set(tasks.keys())

            await _cancel_removed(current_services - desired_services)

            for service in desired_services - current_services:
                task = asyncio.create_task(
                    _stream_container_logs(runner, send_queue, service))
                task.add_done_callback(_bind_done_callback(service))
                tasks[service] = task
    except WebSocketDisconnect:
        pass
    finally:
        remaining_tasks = list(tasks.values())
        for task in remaining_tasks:
            task.cancel()
        if remaining_tasks:
            await asyncio.gather(*remaining_tasks, return_exceptions=True)
        sender_task.cancel()
        await asyncio.gather(sender_task, return_exceptions=True)
