"""プロファイル・シナリオ中の文字列展開。

- `{a.b}` : 変数辞書のドット区切りキーで置換する
- `pkg://<package>/<path>` : ROS パッケージの share ディレクトリ配下の絶対パスに解決する
"""
from __future__ import annotations

import os
import re
from typing import Any, Dict, Mapping

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from sim_scenario_test.errors import ScenarioValidationError

_VAR_PATTERN = re.compile(r"\{([A-Za-z_][A-Za-z0-9_.]*)\}")
_PKG_PREFIX = "pkg://"


def expand(text: str, variables: Mapping[str, Any], where: str) -> str:
    def lookup(match: "re.Match") -> str:
        value: Any = variables
        for key in match.group(1).split("."):
            if not isinstance(value, Mapping) or key not in value:
                raise ScenarioValidationError(
                    f"{where}: undefined variable '{{{match.group(1)}}}'")
            value = value[key]
        return str(value)

    return resolve_package_path(_VAR_PATTERN.sub(lookup, text), where)


def expand_all(values: Dict[str, str], variables: Mapping[str, Any], where: str) -> Dict[str, str]:
    return {k: expand(v, variables, f"{where}.{k}") for k, v in values.items()}


def resolve_package_path(text: str, where: str) -> str:
    if not text.startswith(_PKG_PREFIX):
        return text
    package, _, rel = text[len(_PKG_PREFIX):].partition("/")
    try:
        share = get_package_share_directory(package)
    except PackageNotFoundError as e:
        raise ScenarioValidationError(f"{where}: package '{package}' not found") from e
    return os.path.join(share, rel)
