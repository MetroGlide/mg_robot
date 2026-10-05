import asyncio
import json
import logging
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from mg_system_manager.config import SERVICE_KEYS
from mg_system_manager.log_hub import LogHub, Subscriber

logger = logging.getLogger(__name__)

router = APIRouter()

MAX_BUFFERED_ENTRIES = 2000
FLUSH_INTERVAL_S = 0.1


def _filter_log_services(raw_services: Any) -> set[str]:
    if not isinstance(raw_services, list):
        return set()
    return {
        service
        for service in raw_services
        if isinstance(service, str) and service in SERVICE_KEYS
    }


async def _send_batches(websocket: WebSocket, subscriber: Subscriber) -> None:
    """バッファの中身を一定間隔でまとめて送る。

    {"entries": [{service, line} | {service, error}], "dropped": 捨てた件数}
    """
    while True:
        await subscriber.ready.wait()
        await asyncio.sleep(FLUSH_INTERVAL_S)
        entries, dropped = subscriber.drain()
        if entries or dropped:
            await websocket.send_json({"entries": entries, "dropped": dropped})


@router.websocket("/logs/stream")
async def logs_stream(websocket: WebSocket):
    state = websocket.app.state
    origin = websocket.headers.get("origin")
    if origin and not state.settings.is_origin_allowed(origin):
        await websocket.close(code=1008)
        return

    await websocket.accept()
    hub: LogHub = state.log_hub
    subscriber = Subscriber(MAX_BUFFERED_ENTRIES)
    sender = asyncio.create_task(_send_batches(websocket, subscriber))
    try:
        while True:
            try:
                payload = json.loads(await websocket.receive_text())
            except json.JSONDecodeError:
                continue
            if not isinstance(payload, dict):
                continue
            hub.set_services(
                subscriber, _filter_log_services(payload.get("services")))
    except WebSocketDisconnect:
        pass
    finally:
        hub.remove_subscriber(subscriber)
        sender.cancel()
        await asyncio.gather(sender, return_exceptions=True)
