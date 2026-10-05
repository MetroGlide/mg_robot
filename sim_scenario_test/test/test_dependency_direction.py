"""コアパッケージがロボット固有パッケージ (mg_*) に依存していないことを検査する。"""
from __future__ import annotations

import ast
import pathlib

_PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


def _imported_modules(path: pathlib.Path):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            yield node.module


def test_core_does_not_import_robot_specific_packages():
    offenders = []
    for path in _PACKAGE_ROOT.rglob("*.py"):
        if "test" in path.relative_to(_PACKAGE_ROOT).parts:
            continue
        offenders += [
            f"{path.relative_to(_PACKAGE_ROOT)}: {m}"
            for m in _imported_modules(path) if m.split(".")[0].startswith("mg_")
        ]
    assert offenders == []


def test_core_package_xml_has_no_robot_specific_dependency():
    text = (_PACKAGE_ROOT / "package.xml").read_text()
    assert "<exec_depend>mg_" not in text and "<depend>mg_" not in text
