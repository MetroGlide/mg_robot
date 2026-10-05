"""シナリオと、シナリオテストの結果ディレクトリの読み出し。

結果ディレクトリは sim_scenario_test の CLI が書く。
  <results_root>/<run_id>/progress.json      進み具合 (エントリごとの状態)
  <results_root>/<run_id>/<entry>/result.json シナリオ 1 本の結果 (checks・events)
  <results_root>/<run_id>/<entry>/launch.log  ログ (リモート実行時は stack.log も)
"""
import json
import os
from pathlib import Path
from typing import Any

import yaml

from mg_system_manager.config import SCENARIO_NAME_RE

PROGRESS_FILE = "progress.json"
RESULT_FILE = "result.json"
LOG_FILES = {"launch": "launch.log", "stack": "stack.log"}

# CLI と同じく、補助データのディレクトリはシナリオとして扱わない
_DATA_DIR_NAME = "data"
# ログの末尾を読むときに読み込む最大バイト数
_LOG_MAX_BYTES = 512 * 1024

RUNNING = "running"
FINISHED = "finished"
ABORTED = "aborted"


def list_scenarios(dirs: list[str]) -> list[dict[str, Any]]:
    """シナリオ一覧。group はシナリオを置いたディレクトリ(regression / examples など)の名前。"""
    scenarios = []
    for directory in dirs:
        group = Path(directory).name
        for root, subdirs, names in os.walk(directory):
            subdirs[:] = sorted(d for d in subdirs if d != _DATA_DIR_NAME)
            for name in sorted(n for n in names if n.endswith(".yaml")):
                scenarios.append(_scenario_info(Path(root) / name, group))
    return scenarios


def _scenario_info(path: Path, group: str) -> dict[str, Any]:
    info: dict[str, Any] = {
        "name": path.stem, "group": group, "tags": [], "description": ""}
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as e:
        info["error"] = str(e)
        return info
    if isinstance(raw, dict):
        tags = raw.get("tags", [])
        info["tags"] = [str(t) for t in tags] if isinstance(tags, list) else []
        info["description"] = str(raw.get("description") or "").strip()
    return info


def run_path(root: Path, run_id: str) -> Path | None:
    if not SCENARIO_NAME_RE.match(run_id):
        return None
    return root / run_id


def entry_file(root: Path, run_id: str, entry: str, filename: str) -> Path | None:
    """run の中のファイルのパス。入力が不正、または root の外を指す場合は None。"""
    run = run_path(root, run_id)
    if run is None or not SCENARIO_NAME_RE.match(entry):
        return None
    path = (run / entry / filename).resolve()
    if not path.is_relative_to(root.resolve()):
        return None
    return path


def read_progress(root: Path, run_id: str) -> dict[str, Any] | None:
    run = run_path(root, run_id)
    if run is None:
        return None
    return _read_json(run / PROGRESS_FILE)


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def run_ids(root: Path) -> list[str]:
    """progress.json がある run の id を新しい順に返す(id は日時なので名前の降順)。"""
    try:
        names = os.listdir(root)
    except OSError:
        return []
    return sorted(
        (n for n in names
         if SCENARIO_NAME_RE.match(n) and (root / n / PROGRESS_FILE).is_file()),
        reverse=True)


def run_state(progress: dict[str, Any], active: bool) -> str:
    """run の状態。終わっていないのに実行中のコンテナがなければ中断されたとみなす。"""
    if progress.get("finished"):
        return FINISHED
    return RUNNING if active else ABORTED


def summarize(run_id: str, progress: dict[str, Any], active: bool) -> dict[str, Any]:
    entries = progress.get("entries") or []
    counts: dict[str, int] = {}
    for entry in entries:
        status = str(entry.get("status", ""))
        counts[status] = counts.get(status, 0) + 1
    return {
        "run_id": run_id,
        "started_at": progress.get("started_at"),
        "state": run_state(progress, active),
        "exit_code": progress.get("exit_code"),
        "total": len(entries),
        "counts": counts,
        "options": progress.get("options") or {},
    }


def list_runs(root: Path, limit: int, running: bool) -> list[dict[str, Any]]:
    """過去の run の要約。running のときは最新の run だけを実行中とみなす。"""
    runs = []
    for index, run_id in enumerate(run_ids(root)[:limit]):
        progress = read_progress(root, run_id)
        if progress is not None:
            runs.append(summarize(run_id, progress, running and index == 0))
    return runs


def latest_run(root: Path, running: bool) -> dict[str, Any] | None:
    """最新の run の progress に run_id と state を加えたもの。"""
    ids = run_ids(root)
    if not ids:
        return None
    progress = read_progress(root, ids[0])
    if progress is None:
        return None
    return {**progress, "run_id": ids[0], "state": run_state(progress, running)}


def read_result(path: Path) -> dict[str, Any] | None:
    return _read_json(path)


def tail_lines(path: Path, lines: int) -> str | None:
    """ファイルの末尾 lines 行。大きなログでも末尾の一定サイズだけを読む。"""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - _LOG_MAX_BYTES))
            data = f.read()
    except OSError:
        return None
    text = data.decode(errors="replace")
    return "\n".join(text.splitlines()[-lines:])
