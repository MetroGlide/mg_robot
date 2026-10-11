"""BT の XML のリカバリー部分の名前が、「いまの活動」の文言の表に載っていることを確かめる。

XML を変えたときに、リカバリーが表示されなくなるのを防ぐためのテスト。
"""
import glob
import os
import xml.etree.ElementTree as ET

import pytest

from mg_waypoint_navigation.waypoint_sequencer.activity import RECOVERY_NODE_NAMES

_BT_DIR = os.path.join(os.path.dirname(__file__), '..', 'behavior_trees')
_BT_FILES = sorted(glob.glob(os.path.join(_BT_DIR, 'mg_navigate_to_pose*.xml')))

# 回避行動そのものを行う BT ノード (タグ名が表の汎用の名前になる)
_ACTION_TAGS = {'Spin', 'BackUp', 'Wait'}


def _recovery_leaf_names(path):
    """RecoveryFallback の中の、名前付きの Sequence と、回避行動のノードの名前。"""
    root = ET.parse(path).getroot()
    fallback = next(
        e for e in root.iter() if e.attrib.get('name') == 'RecoveryFallback')
    names = set()
    for element in fallback.iter():
        if element.tag == 'Sequence' and 'name' in element.attrib:
            names.add(element.attrib['name'])
        elif element.tag in _ACTION_TAGS:
            names.add(element.tag)
    return names


def test_bt_files_found():
    assert _BT_FILES


@pytest.mark.parametrize('path', _BT_FILES, ids=os.path.basename)
def test_recovery_actions_have_text(path):
    unknown = _recovery_leaf_names(path) - RECOVERY_NODE_NAMES
    assert not unknown, f'文言の表 (RECOVERY_TEXTS) にない BT ノード: {sorted(unknown)}'
