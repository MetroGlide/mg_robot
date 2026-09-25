import subprocess
from dataclasses import dataclass, field


@dataclass
class FakeContainer:
    service: str
    status: str = "running"
    project: str = "mg"
    oneoff: bool = False
    stopped_with: list = field(default_factory=list)
    removed: bool = False

    @property
    def name(self) -> str:
        return f"{self.project}-{self.service}-1"

    @property
    def id(self) -> str:
        return self.name

    @property
    def labels(self) -> dict[str, str]:
        return {
            "com.docker.compose.project": self.project,
            "com.docker.compose.service": self.service,
            "com.docker.compose.oneoff": "True" if self.oneoff else "False",
        }

    def stop(self, timeout: int | None = None) -> None:
        self.stopped_with.append(timeout)
        self.status = "exited"

    def remove(self) -> None:
        self.removed = True

    def logs(self, tail: int) -> bytes:
        return b"line1\nline2\n"


def _labels_match(actual: dict[str, str], wanted: list[str]) -> bool:
    for item in wanted:
        key, _, value = item.partition("=")
        if key not in actual:
            return False
        if value and actual[key] != value:
            return False
    return True


class FakeContainers:
    def __init__(self, containers: list[FakeContainer]) -> None:
        self.items = containers

    def list(self, all: bool = False, filters: dict | None = None):
        labels = (filters or {}).get("label", [])
        return [
            c for c in self.items
            if (all or c.status == "running") and _labels_match(c.labels, labels)
        ]


class FakeDockerClient:
    def __init__(self, containers: list[FakeContainer]) -> None:
        self.containers = FakeContainers(containers)


class ComposeCalls:
    """subprocess.run の呼び出しを記録し、結果を差し替えられるようにする。"""

    def __init__(self) -> None:
        self.calls: list[dict] = []
        self.returncode = 0
        self.stdout = ""
        self.stderr = ""

    def __call__(self, command, **kwargs):
        self.calls.append({"command": command, **kwargs})
        return subprocess.CompletedProcess(
            command, self.returncode, self.stdout, self.stderr)
