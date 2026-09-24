"""シナリオの一括実行 (シミュレータ起動込み)・結果の集約・JUnit 出力。

シナリオごとに `ros2 launch` を新しいプロセスとして起動するので、走行ごとにシミュレータと
ナビゲーションスタックが作り直される (前の走行の状態を引き継がない)。
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import IO, List, Optional, Sequence

import yaml

from sim_scenario_test.engine.result import EXIT_CODES, ResultStatus

EXIT_ERROR = EXIT_CODES[ResultStatus.ERROR]


@dataclass
class RunRecord:
    name: str
    status: ResultStatus
    message: str
    elapsed_sec: float
    result_dir: str
    checks: List[dict]


DATA_DIR_NAME = "data"


def _yaml_files(directory: str) -> List[str]:
    """ディレクトリ配下 (再帰) のシナリオ YAML。data という名前のディレクトリ (補助データ) は除く。"""
    found: List[str] = []
    for root, dirs, names in os.walk(directory):
        dirs[:] = sorted(d for d in dirs if d != DATA_DIR_NAME)
        found += [os.path.join(root, n) for n in sorted(names) if n.endswith(".yaml")]
    return found


def resolve_scenario(name_or_path: str, search_dirs: Sequence[str]) -> str:
    """パス、または search_dirs 配下の <name>.yaml (再帰検索) をシナリオファイルに解決する。"""
    if os.path.isfile(name_or_path):
        return name_or_path
    for directory in search_dirs:
        for path in _yaml_files(directory):
            if os.path.basename(path) == f"{name_or_path}.yaml":
                return path
    raise FileNotFoundError(
        f"scenario '{name_or_path}' not found (searched: {list(search_dirs)})")


def collect_scenarios(
    paths: Sequence[str], tags: Sequence[str], exclude_tags: Sequence[str] = ()
) -> List[str]:
    """ファイルまたはディレクトリからシナリオ YAML を集める。

    tags 指定時はいずれかを持つものだけ、exclude_tags のいずれかを持つものは除く。
    """
    files: List[str] = []
    for path in paths:
        if os.path.isdir(path):
            files += _yaml_files(path)
        else:
            files.append(path)
    if not tags and not exclude_tags:
        return files
    selected = []
    for path in files:
        with open(path, "r") as f:
            raw = yaml.safe_load(f)
        scenario_tags = set(raw.get("tags", []))
        if tags and not set(tags) & scenario_tags:
            continue
        if set(exclude_tags) & scenario_tags:
            continue
        selected.append(path)
    return selected


# シナリオの内容とは無関係な、起動・環境側の失敗を示すメッセージ (再実行の対象)
_INFRASTRUCTURE_MARKERS = (
    "no result was produced",
    "readiness timeout",
    "simulation clock stalled",
    "timed out after",
)


def is_infrastructure_error(record: "RunRecord") -> bool:
    """シミュレータの起動失敗など、シナリオの内容とは無関係な ERROR かを返す。"""
    return record.status == ResultStatus.ERROR and any(
        marker in record.message for marker in _INFRASTRUCTURE_MARKERS)


def read_result(result_file: str) -> Optional[dict]:
    if not os.path.isfile(result_file):
        return None
    with open(result_file, "r") as f:
        return json.load(f)


def run_scenario(
    scenario_file: str,
    out_dir: str,
    profile: str = "",
    gui: bool = False,
    attach: bool = False,
    timeout_sec: float = 1800.0,
    seed: Optional[int] = None,
    launch_prefix: Sequence[str] = ("ros2", "launch"),
) -> RunRecord:
    """シナリオを 1 本実行して結果を返す。out_dir に result.json / launch.log を残す。"""
    os.makedirs(out_dir, exist_ok=True)
    result_file = os.path.join(out_dir, "result.json")
    if os.path.exists(result_file):
        os.remove(result_file)
    launch_file = "scenario.launch.py" if attach else "scenario_full.launch.py"
    cmd = list(launch_prefix) + [
        "sim_scenario_test", launch_file,
        f"scenario_file:={scenario_file}",
        f"result_file:={result_file}",
    ]
    if profile:
        cmd.append(f"profile:={profile}")
    if seed is not None:
        cmd.append(f"seed:={seed}")
    if not attach:
        cmd.append(f"headless:={'false' if gui else 'true'}")

    name = os.path.splitext(os.path.basename(scenario_file))[0]
    start = time.monotonic()
    timed_out = _run_with_log(cmd, os.path.join(out_dir, "launch.log"), timeout_sec)
    elapsed = time.monotonic() - start

    result = read_result(result_file)
    if result is None:
        reason = (f"timed out after {timeout_sec:.0f}s" if timed_out
                  else "no result was produced (setup or launch failure; see launch.log)")
        return RunRecord(name, ResultStatus.ERROR, reason, elapsed, out_dir, [])
    status = ResultStatus(result["status"])
    failing = [c for c in result["checks"] if c["status"] != "PASSED"]
    message = "; ".join(f"{c['name']}: {c['message']}" for c in failing)
    return RunRecord(result["scenario_name"], status, message, elapsed, out_dir, result["checks"])


def run_scenario_dict(raw: dict, out_dir: str, **kwargs) -> RunRecord:
    """Python の dict で組み立てたシナリオを実行する (pytest からシナリオを書くための入口)。

    dict は YAML と同じ構造。out_dir/scenario.yaml に書き出して run_scenario に渡す。
    """
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "scenario.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(raw, f, allow_unicode=True, sort_keys=False)
    return run_scenario(path, out_dir, **kwargs)


def _run_with_log(cmd: List[str], log_path: str, timeout_sec: float) -> bool:
    """cmd を実行して出力をログに保存しつつ表示する。タイムアウトしたら True を返す。"""
    with open(log_path, "w") as log:
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            start_new_session=True)
        deadline = time.monotonic() + timeout_sec
        timed_out = False
        pump = threading.Thread(target=_pump, args=(proc.stdout, log), daemon=True)
        pump.start()
        while proc.poll() is None:
            if time.monotonic() > deadline:
                timed_out = True
                _terminate(proc)
                break
            time.sleep(0.5)
        pump.join(timeout=5.0)
        # ros2 launch の終了後もシミュレータ本体 (ign gazebo -s) などが残ることがある。
        # 残すと次の実行と同じワールド・トピックで干渉するため、プロセスグループごと片付ける。
        _kill_group(proc.pid)
    return timed_out


def _pump(stream: IO[str], log: IO[str]) -> None:
    for line in stream:
        log.write(line)
        log.flush()
        sys.stdout.write(line)
        sys.stdout.flush()


def _kill_group(pgid: int, grace_sec: float = 3.0) -> None:
    """プロセスグループの残存プロセスを SIGTERM → SIGKILL で終了させる。"""
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + grace_sec
        while time.monotonic() < deadline:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.1)


def _terminate(proc: subprocess.Popen) -> None:
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def exit_code(records: Sequence[RunRecord]) -> int:
    """全体の終了コード。ERROR があれば 2、なければ FAILED があれば 1、すべて成功なら 0。"""
    statuses = {r.status for r in records}
    if ResultStatus.ERROR in statuses or not records:
        return EXIT_CODES[ResultStatus.ERROR]
    if ResultStatus.FAILED in statuses:
        return EXIT_CODES[ResultStatus.FAILED]
    return EXIT_CODES[ResultStatus.PASSED]


def write_junit(records: Sequence[RunRecord], path: str, suite_name: str = "scenarios") -> None:
    suite = ET.Element("testsuite", name=suite_name, tests=str(len(records)))
    failures = errors = 0
    for rec in records:
        case = ET.SubElement(
            suite, "testcase", classname=suite_name, name=rec.name,
            time=f"{rec.elapsed_sec:.1f}")
        if rec.status == ResultStatus.FAILED:
            failures += 1
            ET.SubElement(case, "failure", message=rec.message).text = rec.message
        elif rec.status == ResultStatus.ERROR:
            errors += 1
            ET.SubElement(case, "error", message=rec.message).text = rec.message
    suite.set("failures", str(failures))
    suite.set("errors", str(errors))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    ET.ElementTree(suite).write(path, encoding="utf-8", xml_declaration=True)


def format_summary(records: Sequence[RunRecord]) -> str:
    lines = ["", "=" * 60, "  Summary", "=" * 60]
    for rec in records:
        line = f"  {rec.status.value:<7} {rec.name} ({rec.elapsed_sec:.0f}s)"
        if rec.message:
            line += f" - {rec.message}"
        lines.append(line)
    passed = sum(r.status == ResultStatus.PASSED for r in records)
    lines.append(f"  {passed}/{len(records)} passed")
    return "\n".join(lines)


def default_results_dir() -> str:
    return os.path.join(os.path.expanduser("~"), ".ros", "scenario_results",
                        time.strftime("%Y%m%d_%H%M%S"))
