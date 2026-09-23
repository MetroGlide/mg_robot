"""シナリオの静的検証・利用可能な型の一覧表示 (シミュレータ不要)。

  scenario_cli.py validate <scenario.yaml>... [--profile NAME]
  scenario_cli.py list-types [--profile NAME]
"""
from __future__ import annotations

import argparse
import sys
from typing import List

from sim_scenario_test.errors import ScenarioValidationError
from sim_scenario_test.loader import load_scenario
from sim_scenario_test.plugins import load_plugins
from sim_scenario_test.profile import load_profile
from sim_scenario_test.registry import DEFAULT_REGISTRY, KINDS


def _validate(files: List[str], profile: str) -> int:
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


def main(argv: List[str] = None) -> int:
    parser = argparse.ArgumentParser(prog="scenario_cli")
    sub = parser.add_subparsers(dest="command", required=True)
    p_validate = sub.add_parser("validate", help="validate scenario files")
    p_validate.add_argument("files", nargs="+")
    p_validate.add_argument("--profile", default="")
    p_list = sub.add_parser("list-types", help="list registered types")
    p_list.add_argument("--profile", default="")
    args = parser.parse_args(argv)
    if args.command == "validate":
        return _validate(args.files, args.profile)
    return _list_types(args.profile)


if __name__ == "__main__":
    sys.exit(main())
