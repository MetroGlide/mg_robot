#!/usr/bin/env python3
"""
run_localization_queue.py

複数の構成を、評価セット (tools/datasets/localization/sets/) の全条件で再生評価する。
構成 × 条件 × 試行を 1 つの待ち行列にし、空いた枠から 1 試行ずつコンテナで回す
(条件の中の試行を順に回すと、最も長い条件で全体が頭打ちになるため)。

- パッケージは最初に 1 回だけビルドし、そのイメージ (--replay-image) を各試行で使う。
- 空きメモリが --min-free-gb を下回る間は、次の試行を起動しない。
- 試行ごとに上限時間を置き、超えたらコンテナを止めて失敗にする。
- --min-runs を --runs より小さくすると、まず --min-runs 回回し、ずれた事象の有無か
  ずれたまま終わったかが試行で割れた条件だけ、--runs 回まで足す。
- 同じ名前で回し直すと、その変種の前の試行 (run_*) は消える。

使い方:
  tools/scripts/run_localization_queue.py --set focus --runs 5 --jobs 8 \\
      --config "base_sup=--monitor supervisor --no-particles" \\
      --config "a2_sup=--monitor supervisor --no-particles amcl.alpha2=0.1"

--config の値は run_localization_variant.sh のオプションとパラメータ (--runs は渡さない)。
変種の名前は <構成>_<ラベル>。集計は compare_localization_set.py で行う。
ログ: tools/data/variants/set_<構成>/<ラベル>_run<N>.log
"""

import argparse
import os
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from compare_localization import load_runs  # noqa: E402
from compare_localization_set import SET_DIR, host_path  # noqa: E402
from loc_dataset import load_dataset, resolve_dataset_path  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
VARIANT_SCRIPT = os.path.join(ROOT, "tools", "scripts", "run_localization_variant.sh")
LOG_ROOT = os.path.join(ROOT, "tools", "data", "variants")
CONTAINER_DATA = "/root/ros2_data"
# 1 試行の上限時間 = (再生の長さ + 起動・初期姿勢・評価の余裕) × 試行の中のやり直しの回数
DEADLINE_MARGIN_SEC = 900.0
DEADLINE_ATTEMPTS = 2
POLL_SEC = 5.0


@dataclass
class Condition:
    label: str
    dataset: str
    extra: List[str]
    duration: float
    out_dir: str


@dataclass
class Job:
    config: str
    condition: Condition
    run: int
    tries: int = 0
    proc: Optional[subprocess.Popen] = None
    started: float = 0.0
    container: str = ""
    log: object = field(default=None, repr=False)

    @property
    def variant(self) -> str:
        return f"{self.config}_{self.condition.label}"


def parse_config(text: str) -> Tuple[str, List[str]]:
    name, sep, args = text.partition("=")
    if not sep or not name.strip():
        raise SystemExit(f"エラー: --config は <名前>=<引数> で指定してください: {text}")
    parsed = shlex.split(args)
    if "--runs" in parsed or "--run-from" in parsed:
        raise SystemExit(f"エラー: --config に --runs / --run-from は渡せません (待ち行列が決めます): {text}")
    return name.strip(), parsed


def read_conditions(set_name: str, only: Optional[Set[str]]) -> List[Condition]:
    """評価セットの条件 (ラベル、データセット、追加の引数、再生の長さ、出力先) の並び。"""
    path = set_name if os.path.isfile(set_name) else os.path.join(SET_DIR, set_name + ".txt")
    if not os.path.isfile(path):
        raise SystemExit(f"エラー: 評価セットが見つかりません: {set_name}")
    conditions = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            fields = shlex.split(line, comments=True)
            if len(fields) < 2 or (only and fields[0] not in only):
                continue
            values = load_dataset(resolve_dataset_path(fields[1]))
            conditions.append(Condition(
                label=fields[0], dataset=fields[1], extra=fields[2:], duration=float(values["DURATION"]),
                out_dir=os.path.join(values["BAG"], "eval_loc", values["DATASET_NAME"])))
    return conditions


def deadline_sec(duration: float) -> float:
    return (duration + DEADLINE_MARGIN_SEC) * DEADLINE_ATTEMPTS


def outcomes_split(runs: List[Dict]) -> bool:
    """ずれた事象の有無か、ずれたまま終わったかが、試行で割れているか。"""
    outcomes = {(int((r.get("deviations") or {}).get("n", 0)) > 0,
                 bool((r.get("deviations") or {}).get("until_end", False))) for r in runs}
    return len(outcomes) > 1


def mem_available_gb() -> float:
    with open("/proc/meminfo", "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1024.0 / 1024.0
    return 0.0


def docker_ros(image: str, command: str, name: Optional[str] = None) -> int:
    """リポジトリとデータを載せたコンテナで command を実行する (root のファイルの削除やまとめに使う)。"""
    args = ["docker", "run", "--rm", "-v", f"{ROOT}:/app",
            "-v", f"{os.path.expanduser('~/ros2_data')}:{CONTAINER_DATA}"]
    if name:
        args = ["docker", "run", "--name", name, "-v", f"{ROOT}:/app",
                "-v", f"{os.path.expanduser('~/ros2_data')}:{CONTAINER_DATA}"]
    return subprocess.call(args + [image, "bash", "-c", command])


def build_image(image: str, replay_image: str) -> None:
    """パッケージを 1 回だけビルドし、その状態のコンテナをイメージにする。"""
    name = f"mg_loc_build_{os.getpid()}"
    print(f"[queue] パッケージをビルドして {replay_image} を作ります", flush=True)
    status = docker_ros(image, "BUILD_ONLY=true OUT_DIR=/tmp bash /app/tools/scripts/localization_replay.sh", name)
    if status == 0:
        status = subprocess.call(["docker", "commit", name, replay_image], stdout=subprocess.DEVNULL)
    subprocess.call(["docker", "rm", "-f", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if status != 0:
        raise SystemExit("エラー: ビルドに失敗しました。")


def start(job: Job, config_args: List[str], replay_image: str) -> None:
    job.tries += 1
    job.container = f"mg_loc_{job.variant}_r{job.run}_{os.getpid()}"
    log_dir = os.path.join(LOG_ROOT, f"set_{job.config}")
    os.makedirs(log_dir, exist_ok=True)
    job.log = open(os.path.join(log_dir, f"{job.condition.label}_run{job.run}.log"), "w", encoding="utf-8")
    cmd = [VARIANT_SCRIPT, job.variant, "--dataset", job.condition.dataset, *job.condition.extra, *config_args,
           "--runs", str(job.run), "--run-from", str(job.run), "--no-build"]
    env = dict(os.environ, IMAGE=replay_image, CONTAINER_NAME=job.container)
    job.proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=job.log, stderr=subprocess.STDOUT)
    job.started = time.monotonic()
    print(f"[queue] 開始: {job.variant} run {job.run}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="複数の構成を、評価セットの全条件で試行単位の待ち行列にして再生評価する")
    parser.add_argument("--set", default="robustness", help="評価セット (名前またはパス)")
    parser.add_argument("--config", action="append", required=True, help="<名前>=<run_localization_variant.sh の引数>")
    parser.add_argument("--runs", type=int, default=5, help="条件ごとの試行の回数 (最大)")
    parser.add_argument("--min-runs", type=int, default=None,
                        help="まず回す回数。結果が割れた条件だけ --runs 回まで足す (既定: --runs と同じ)")
    parser.add_argument("--jobs", type=int, default=8, help="同時に回す試行の数")
    parser.add_argument("--min-free-gb", type=float, default=4.0, help="次の試行を起動する空きメモリの下限 [GB]")
    parser.add_argument("--start-interval", type=float, default=10.0, help="試行を起動する間隔の下限 [s]")
    parser.add_argument("--retries", type=int, default=1, help="失敗した試行をやり直す回数")
    parser.add_argument("--only", default="", help="この (カンマ区切りの) ラベルの条件だけ回す")
    parser.add_argument("--image", default=os.environ.get("IMAGE", "mg_develop:latest"), help="元の Docker イメージ")
    parser.add_argument("--replay-image", default="mg_loc_replay:latest", help="ビルド済みのイメージの名前")
    parser.add_argument("--dry-run", action="store_true", help="試行の一覧を出すだけで実行しない")
    args = parser.parse_args()

    configs = dict(parse_config(c) for c in args.config)
    min_runs = args.min_runs or args.runs
    if not 1 <= min_runs <= args.runs:
        raise SystemExit("エラー: --min-runs は 1 以上 --runs 以下にしてください。")
    conditions = read_conditions(args.set, set(args.only.split(",")) if args.only else None)

    # 長い試行から先に回すと、最後に長い試行だけが残りにくい
    pending = [Job(config, condition, run) for config in configs for condition in conditions
               for run in range(1, min_runs + 1)]
    pending.sort(key=lambda j: (-j.condition.duration, j.run))
    print(f"[queue] 構成 {len(configs)} × 条件 {len(conditions)} の試行 {len(pending)} 件 "
          f"(並列 {args.jobs}、条件ごとに {min_runs}〜{args.runs} 回)", flush=True)
    if args.dry_run:
        for job in pending:
            print(f"  {job.variant} run {job.run} ({job.condition.duration:.0f} s)")
        return

    variant_dirs = sorted({os.path.join(c.out_dir, f"{config}_{c.label}") for config in configs for c in conditions})
    docker_ros(args.image, "rm -rf " + " ".join(shlex.quote(d + "/run_") + "*" for d in variant_dirs))
    build_image(args.image, args.replay_image)

    running: List[Job] = []
    done: Dict[Tuple[str, str], int] = {}
    failed: List[Job] = []
    last_start = 0.0
    while pending or running:
        for job in list(running):
            status = job.proc.poll()
            if status is None and time.monotonic() - job.started > deadline_sec(job.condition.duration):
                print(f"[queue] 上限時間を超えたので止めます: {job.variant} run {job.run}", flush=True)
                subprocess.call(["docker", "rm", "-f", job.container],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                status = job.proc.wait()
                status = status or 1
            if status is None:
                continue
            running.remove(job)
            job.log.close()
            if status != 0:
                if job.tries <= args.retries:
                    # やり直しでログが上書きされないよう、失敗した回のログを残す
                    kept = job.log.name.replace(".log", f"_try{job.tries}.log")
                    os.replace(job.log.name, kept)
                    print(f"[queue] 失敗したのでやり直します: {job.variant} run {job.run} ({kept})", flush=True)
                    pending.insert(0, job)
                else:
                    print(f"[queue] 失敗: {job.variant} run {job.run} ({job.log.name})", flush=True)
                    failed.append(job)
                continue
            print(f"[queue] 完了: {job.variant} run {job.run}", flush=True)
            key = (job.config, job.condition.label)
            done[key] = done.get(key, 0) + 1
            if done[key] == min_runs < args.runs and \
                    outcomes_split(load_runs(host_path(os.path.join(job.condition.out_dir, job.variant)))):
                print(f"[queue] 結果が割れたので {args.runs} 回まで足します: {job.variant}", flush=True)
                pending[:0] = [Job(job.config, job.condition, run) for run in range(min_runs + 1, args.runs + 1)]
        if pending and len(running) < args.jobs and time.monotonic() - last_start >= args.start_interval \
                and mem_available_gb() >= args.min_free_gb:
            job = pending.pop(0)
            start(job, configs[job.config], args.replay_image)
            running.append(job)
            last_start = time.monotonic()
            continue
        time.sleep(POLL_SEC)

    # 試行ごとの結果を変種ごとにまとめる (出力先はコンテナが作るので、コンテナの中で書く)
    docker_ros(args.image, " && ".join(
        f"python3 /app/tools/scripts/compare_localization.py {shlex.quote(d)} -o {shlex.quote(d + '/summary.md')}"
        f" > /dev/null" for d in variant_dirs if load_runs(host_path(d))) or "true")
    names = " ".join(configs)
    print(f"[queue] すべての試行が終わりました。失敗 {len(failed)} 件。"
          f"集計: python3 tools/scripts/compare_localization_set.py --set {args.set} {names}", flush=True)
    for job in failed:
        print(f"  失敗: {job.variant} run {job.run} ({job.log.name})")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
