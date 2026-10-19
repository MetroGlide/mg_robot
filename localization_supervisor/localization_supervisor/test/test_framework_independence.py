"""フレームワークが、特定のロボットのパッケージに依存しないことを確かめる。

package.xml の依存と、Python の import の両方を見る。
"""
import ast
import pathlib
import sys
import xml.etree.ElementTree as ET

import pytest

PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
FORBIDDEN_PREFIXES = ('mg_',)
DEPEND_TAGS = ('depend', 'build_depend', 'exec_depend', 'test_depend', 'build_export_depend')


def _python_files():
    return sorted(
        path for path in PACKAGE_DIR.rglob('*.py')
        if 'test' not in path.relative_to(PACKAGE_DIR).parts[:1])


def _imported_modules(path):
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                yield alias.name
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            yield node.module


def test_package_xml_has_no_robot_specific_dependency():
    root = ET.parse(PACKAGE_DIR / 'package.xml').getroot()
    depends = [element.text.strip() for tag in DEPEND_TAGS for element in root.iter(tag)]
    assert [d for d in depends if d.startswith(FORBIDDEN_PREFIXES)] == []


def test_python_files_exist():
    assert _python_files()


@pytest.mark.parametrize('path', _python_files(), ids=lambda p: str(p.relative_to(PACKAGE_DIR)))
def test_sources_import_no_robot_specific_package(path):
    imported = [name for name in _imported_modules(path) if name.split('.')[0].startswith(FORBIDDEN_PREFIXES)]
    assert imported == []


CORE_ALLOWED = {'numpy', 'scipy', 'yaml'}


@pytest.mark.parametrize('path', sorted((PACKAGE_DIR / 'localization_supervisor' / 'core').rglob('*.py')),
                         ids=lambda p: str(p.relative_to(PACKAGE_DIR)))
def test_core_does_not_depend_on_ros(path):
    # 中核は ROS なしで動く (オフライン評価や他のロボットで使う)。標準ライブラリと numpy・scipy・yaml だけを使う
    imported = {name.split('.')[0] for name in _imported_modules(path)}
    third_party = {name for name in imported if name not in sys.stdlib_module_names}
    assert third_party <= CORE_ALLOWED
