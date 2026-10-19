#!/usr/bin/env python3

"""オフライン評価 (rosbag で監督を実時間を待たずに回す) のラッパー。実装は localization_supervisor.ros.offline_runner。"""
from localization_supervisor.ros.offline_runner import main


if __name__ == '__main__':
    main()
