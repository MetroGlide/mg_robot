from __future__ import annotations

import importlib
from typing import List


def load_plugins(modules: List[str]) -> None:
    """プロファイルに列挙されたプラグインモジュールを読み込み、型を登録させる。

    プラグイン機構の性質上ここだけは動的 import とする。
    読み込みに失敗した場合は握りつぶさずにそのまま例外を送出する。
    """
    for module in modules:
        importlib.import_module(module)
