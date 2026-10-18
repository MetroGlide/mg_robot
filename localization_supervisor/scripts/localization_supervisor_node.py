#!/usr/bin/env python3

"""パッケージにインストールされた自己位置の監督ノードを起動するラッパー。

ros2 run / launch はスクリプトを別の場所へインストールするため、実装はインストール済みの
python パッケージ (localization_supervisor.ros.supervisor_node) から import する。
"""
from localization_supervisor.ros.supervisor_node import main


if __name__ == '__main__':
    main()
