#!/bin/bash
# ROS 2 とワークスペースの環境を読み込んでから、渡されたコマンドに置き換わる。
# exec で置き換えるため、docker stop のシグナル (SIGINT) がコマンドへ直接届く。
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
exec "$@"
