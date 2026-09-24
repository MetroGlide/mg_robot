"""シナリオの静的検証・利用可能な型の一覧表示 (シミュレータ不要)。

  scenario_cli.py validate <scenario.yaml>... [--profile NAME]
  scenario_cli.py list-types [--profile NAME]

シミュレータを起動して実行する (終了コード: 0=PASSED, 1=FAILED, 2=ERROR):
  scenario_cli.py run <scenario.yaml> [--profile NAME] [--gui] [--attach] [--results-dir DIR]
  scenario_cli.py run-all <scenario.yaml|dir>... [--tags a,b] [--repeat N] [--results-dir DIR]
  (run / run-all は --remote-stack URL で別のマシンにスタックを起動させられる)
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import List

from sim_scenario_test.errors import ScenarioValidationError
from sim_scenario_test.execution import (
    RunRecord,
    collect_scenarios,
    resolve_scenario,
    default_results_dir,
    exit_code,
    format_summary,
    is_infrastructure_error,
    run_scenario,
    write_junit,
)
from sim_scenario_test.loader import load_scenario
from sim_scenario_test.plugins import load_plugins
from sim_scenario_test.profile import load_profile
from sim_scenario_test.registry import DEFAULT_REGISTRY, KINDS
from sim_scenario_test.remote_stack import RemoteStackClient


def _validate(files: List[str], profile: str) -> int:
    files = collect_scenarios(files, [])
    failed = 0
    for path in files:
        try:
            scenario, prof = load_scenario(path, profile)
        except (ScenarioValidationError, OSError) as e:
            failed += 1
            print(f"NG  {path}\n    {e}")
            continue
        print(f"OK  {path}  ({scenario.name}, profile={prof.name}, run={scenario.run.name})")
    print(f"\n{len(files) - failed}/{len(files)} valid")
    return 1 if failed else 0


def _list_types(profile: str) -> int:
    if profile:
        load_plugins(load_profile(profile, DEFAULT_REGISTRY).plugins)
    for kind in KINDS:
        print(f"[{kind}]")
        for entry in DEFAULT_REGISTRY.entries(kind):
            print(f"  {entry.name:<24} {entry.doc}")
    return 0


def _run(args: argparse.Namespace) -> int:
    if args.remote_stack and args.attach:
        print("--remote-stack and --attach cannot be used together")
        return 2
    remote_stack = RemoteStackClient(args.remote_stack) if args.remote_stack else None
    results_dir = args.results_dir or default_results_dir()
    try:
        targets = [resolve_scenario(f, args.scenario_dir) if not os.path.isdir(f) else f
                   for f in args.files]
    except FileNotFoundError as e:
        print(e)
        return 2
    scenarios = collect_scenarios(
        targets,
        [t for t in args.tags.split(",") if t],
        [t for t in args.exclude_tags.split(",") if t])
    if not scenarios:
        print("no scenarios selected")
        return 2
    records: List[RunRecord] = []
    for repeat in range(args.repeat):
        for path in scenarios:
            name = os.path.splitext(os.path.basename(path))[0]
            suffix = f"_run{repeat + 1}" if args.repeat > 1 else ""
            out_dir = os.path.join(results_dir, name + suffix)
            record = None
            for attempt in range(args.infra_retries + 1):
                record = run_scenario(
                    path, out_dir if attempt == 0 else f"{out_dir}_retry{attempt}",
                    profile=args.profile, gui=args.gui, attach=args.attach,
                    timeout_sec=args.timeout, remote_stack=remote_stack,
                    seed=None if args.seed is None else args.seed + repeat)
                if not is_infrastructure_error(record):
                    break
                print(f"\n[retry] {name}: infrastructure error ({record.message}); "
                      f"attempt {attempt + 1}/{args.infra_retries + 1}\n")
            records.append(record)
    write_junit(records, os.path.join(results_dir, "junit.xml"))
    print(format_summary(records))
    print(f"\nresults: {results_dir}")
    return exit_code(records)


def main(argv: List[str] = None) -> int:
    parser = argparse.ArgumentParser(prog="scenario_cli")
    sub = parser.add_subparsers(dest="command", required=True)
    p_validate = sub.add_parser("validate", help="validate scenario files")
    p_validate.add_argument("files", nargs="+")
    p_validate.add_argument("--profile", default="")
    p_list = sub.add_parser("list-types", help="list registered types")
    p_list.add_argument("--profile", default="")
    for name, help_text in (("run", "run scenario(s) with simulator"),
                            ("run-all", "run a scenario suite")):
        p_run = sub.add_parser(name, help=help_text)
        p_run.add_argument("files", nargs="+")
        p_run.add_argument("--profile", default="")
        p_run.add_argument("--scenario-dir", action="append", default=[],
                           help="directory searched for <name>.yaml when a name is given")
        p_run.add_argument("--gui", action="store_true", help="show simulator GUI")
        p_run.add_argument("--attach", action="store_true",
                           help="use an already running simulator and stack")
        p_run.add_argument("--remote-stack", default="", metavar="URL",
                           help="start the navigation stack on another machine via this "
                                "system manager URL (e.g. http://192.168.0.10:8001); "
                                "the simulator still runs locally")
        p_run.add_argument("--results-dir", default="")
        p_run.add_argument("--timeout", type=float, default=1800.0,
                           help="wall-clock limit per scenario [s]")
        p_run.add_argument("--tags", default="", help="comma separated; any match")
        p_run.add_argument("--exclude-tags", default="",
                           help="comma separated; skip scenarios having any of these tags")
        p_run.add_argument("--repeat", type=int, default=1)
        p_run.add_argument("--infra-retries", type=int, default=1,
                           help="re-run a scenario this many times when it failed for "
                                "infrastructure reasons (simulator did not start etc.)")
        p_run.add_argument("--seed", type=int, default=None,
                           help="override scenario seed (incremented per repeat)")
    args = parser.parse_args(argv)
    if args.command in ("run", "run-all"):
        return _run(args)
    if args.command == "validate":
        return _validate(args.files, args.profile)
    return _list_types(args.profile)


if __name__ == "__main__":
    sys.exit(main())
