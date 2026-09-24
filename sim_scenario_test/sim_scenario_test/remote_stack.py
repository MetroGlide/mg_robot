"""別のマシン (実機PCなど) でナビゲーションスタックを起動・停止するクライアント。

シミュレータを動かすマシンから HTTP でスタックの起動・停止を依頼する。
プロトコルは docs/remote_stack.md を参照。標準ライブラリだけで実装している。
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Dict

from sim_scenario_test.loader import load_scenario
from sim_scenario_test.template import expand_all


class RemoteStackError(RuntimeError):
    pass


@dataclass
class StackLaunch:
    """リモートで起動する launch。args は変数展開済み。"""
    package: str
    file: str
    args: Dict[str, str]


def resolve_stack(scenario_file: str, profile_name: str = "") -> StackLaunch:
    """シナリオとプロファイルから、リモートで起動するスタックの launch を求める。"""
    scenario, profile = load_scenario(scenario_file, profile_name)
    if profile.stack is None:
        raise RemoteStackError(f"profile '{profile.name}' has no 'stack' to start remotely")
    # スタックの引数は headless を使わない。sim 側は常にローカルで起動する
    variables = profile.launch_variables(scenario.world, "true", "scenario.world")
    args = expand_all(
        {**profile.stack.args, **scenario.stack_args}, variables, "profile.stack.args")
    return StackLaunch(profile.stack.package, profile.stack.file, args)


class RemoteStackClient:
    """`POST /scenario-stack/{start,stop}` と `GET /scenario-stack/logs` を呼ぶ。"""

    def __init__(self, base_url: str, timeout_sec: float = 120.0):
        self._base_url = base_url.rstrip("/")
        self._timeout_sec = timeout_sec

    def start(self, launch: StackLaunch) -> None:
        self._post("/scenario-stack/start", {
            "package": launch.package, "file": launch.file, "args": launch.args})

    def stop(self) -> None:
        self._post("/scenario-stack/stop", {})

    def logs(self) -> str:
        with self._open(urllib.request.Request(self._base_url + "/scenario-stack/logs")) as res:
            return json.load(res).get("logs", "")

    def _post(self, path: str, body: dict) -> None:
        request = urllib.request.Request(
            self._base_url + path, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        with self._open(request) as res:
            result = json.load(res)
        if not result.get("success"):
            raise RemoteStackError(f"{path}: {result.get('message', '')}")

    def _open(self, request: urllib.request.Request):
        try:
            return urllib.request.urlopen(request, timeout=self._timeout_sec)
        except (urllib.error.URLError, OSError) as e:
            raise RemoteStackError(f"{request.full_url}: {e}") from e
