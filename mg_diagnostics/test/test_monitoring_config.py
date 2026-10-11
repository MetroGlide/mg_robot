"""監視の設定 (analyzer・topic_monitor の YAML) の整合を調べる。ROS 不要。

analyzer は診断の名前 `<ノード名>: <タスク名>` の前方一致で振り分ける。ノード名が実際と違うと、
診断は Other に入り、期待した項目は STALE のままになる。
"""
from pathlib import Path

import pytest
import yaml

from mg_utils.monitoring.topic_source_config import parse_source_config

PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = PACKAGE_DIR.parent
ANALYZER_FILES = sorted((PACKAGE_DIR / 'params' / 'analyzers').glob('*.yaml'))
TOPIC_MONITOR_FILES = sorted((PACKAGE_DIR / 'params' / 'topic_monitor').glob('*.yaml'))
SOURCE_SUFFIXES = ('.py', '.cpp')
EXCLUDED_DIRS = {'node_modules', 'build', 'install', 'test', 'tools', '.git'}


MATCH_KEYS = ('startswith', 'regex', 'expected')


def _walk_prefixes(node):
    """analyzer の設定から、診断の名前の照合に使う項目をすべて集める。"""
    if isinstance(node, dict):
        for key, value in node.items():
            if key in MATCH_KEYS and isinstance(value, list):
                yield from value
            else:
                yield from _walk_prefixes(value)


def _launch_and_node_sources():
    texts = []
    for path in REPO_DIR.rglob('*'):
        if path.suffix not in SOURCE_SUFFIXES or EXCLUDED_DIRS & set(path.relative_to(REPO_DIR).parts):
            continue
        texts.append(path.read_text(encoding='utf-8', errors='ignore'))
    return '\n'.join(texts)


@pytest.fixture(scope='module')
def source_text():
    return _launch_and_node_sources()


@pytest.mark.parametrize('path', ANALYZER_FILES, ids=lambda p: p.name)
def test_analyzer_node_names_exist(path, source_text):
    prefixes = list(_walk_prefixes(yaml.safe_load(path.read_text(encoding='utf-8'))))
    missing = []
    for prefix in prefixes:
        node_name, separator, _ = prefix.lstrip('^').partition(': ')
        if not separator:
            continue
        if f"'{node_name}'" not in source_text and f'"{node_name}"' not in source_text:
            missing.append(node_name)
    assert not missing, f'{path.name}: ノード名がソースに見つからない (名前の違い?): {missing}'


@pytest.mark.parametrize('path', TOPIC_MONITOR_FILES, ids=lambda p: p.name)
def test_topic_monitor_sources_are_valid(path):
    params = yaml.safe_load(path.read_text(encoding='utf-8'))['topic_monitor_node']['ros__parameters']
    for source_id in params['monitored_sources']:
        assert source_id in params, f'{source_id} の設定がない'
        parse_source_config(source_id, lambda key, default, p=params[source_id]: p.get(key, default), 3.0)
