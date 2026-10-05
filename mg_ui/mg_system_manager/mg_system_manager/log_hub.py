"""コンテナのログを購読者へ配る。

サービスごとの `docker logs -f` は購読者の数によらず 1 本だけ起動する。
購読者ごとのバッファには上限があり、あふれた分は古い行から捨てて件数を数える。
"""
import asyncio
import logging
from collections import deque
from typing import Any

from mg_system_manager.docker_ops import ComposeRunner

logger = logging.getLogger(__name__)

RETRY_INTERVAL_S = 5.0
# asyncio の StreamReader は 1 行がこの長さを超えると例外になるため、余裕を持たせる
_STREAM_LIMIT_BYTES = 1 << 20
MAX_LINE_CHARS = 4000


class Subscriber:
    """1 つの WebSocket 接続に対応する、上限付きのバッファ。"""

    def __init__(self, max_entries: int) -> None:
        self._max_entries = max_entries
        self._entries: deque[dict[str, str]] = deque()
        self._dropped = 0
        self.ready = asyncio.Event()

    def push(self, entry: dict[str, str]) -> None:
        if len(self._entries) >= self._max_entries:
            self._entries.popleft()
            self._dropped += 1
        self._entries.append(entry)
        self.ready.set()

    def drain(self) -> tuple[list[dict[str, str]], int]:
        entries = list(self._entries)
        dropped = self._dropped
        self._entries.clear()
        self._dropped = 0
        self.ready.clear()
        return entries, dropped


class LogHub:
    def __init__(self, runner: ComposeRunner) -> None:
        self._runner = runner
        self._subscribers: dict[str, set[Subscriber]] = {}
        self._tasks: dict[str, asyncio.Task[None]] = {}

    def set_services(self, subscriber: Subscriber, services: set[str]) -> None:
        """subscriber が購読するサービスを services に置き換える。"""
        for service in list(self._subscribers):
            if service not in services:
                self._remove(service, subscriber)
        for service in services:
            self._add(service, subscriber)

    def remove_subscriber(self, subscriber: Subscriber) -> None:
        for service in list(self._subscribers):
            self._remove(service, subscriber)

    async def close(self) -> None:
        tasks = list(self._tasks.values())
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        self._tasks.clear()
        self._subscribers.clear()

    def _add(self, service: str, subscriber: Subscriber) -> None:
        subscribers = self._subscribers.setdefault(service, set())
        subscribers.add(subscriber)
        if service not in self._tasks:
            self._tasks[service] = asyncio.create_task(self._stream(service))

    def _remove(self, service: str, subscriber: Subscriber) -> None:
        subscribers = self._subscribers.get(service)
        if subscribers is None:
            return
        subscribers.discard(subscriber)
        if subscribers:
            return
        del self._subscribers[service]
        task = self._tasks.pop(service, None)
        if task is not None:
            task.cancel()

    def _publish(self, service: str, entry: dict[str, Any]) -> None:
        for subscriber in self._subscribers.get(service, ()):
            subscriber.push(entry)

    async def _stream(self, service: str) -> None:
        last_error: str | None = None
        while True:
            # Docker SDK の呼び出しはブロッキングなので、イベントループを止めないようにする
            container = await asyncio.to_thread(
                self._runner.get_container, service)
            ref = (container.name or container.id) if container else None
            if not ref:
                error = ("container not found" if container is None
                         else "container reference missing")
                if error != last_error:
                    self._publish(service, {"service": service, "error": error})
                    last_error = error
                await asyncio.sleep(RETRY_INTERVAL_S)
                continue
            last_error = None
            await self._follow(service, ref)
            await asyncio.sleep(RETRY_INTERVAL_S)

    async def _follow(self, service: str, container_ref: str) -> None:
        proc = await asyncio.create_subprocess_exec(
            "docker", "logs", "-f", "--since", "0s", container_ref,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            limit=_STREAM_LIMIT_BYTES,
        )
        try:
            assert proc.stdout is not None
            while True:
                raw = await proc.stdout.readline()
                if not raw:
                    break
                text = raw.decode(errors="replace").rstrip("\n")
                if text:
                    self._publish(service, {
                        "service": service,
                        "line": text[:MAX_LINE_CHARS],
                    })
            await proc.wait()
        finally:
            if proc.returncode is None:
                proc.terminate()
                await proc.wait()
