#!/bin/bash
# 元の rosbag のスキャンから、KISS-ICP (mg_drivers の fork) で LiDAR オドメトリを作って記録する (コンテナ内で実行)。
# 出力は eval_window_estimator.py・eval_gate_arbitration.py の --lidar-odom に渡す。
#
# 使い方: tools/scripts/lidar_odom_offline.sh <bag> <開始の秒> <長さの秒> <出力の bag>
# 環境変数: RATE (再生速度。既定 2.0)、MAX_RANGE・MIN_RANGE (KISS-ICP の距離の範囲 [m])
set -e
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
BAG=$1; START=$2; DUR=$3; OUT=$4
RATE=${RATE:-2.0}
SCAN_TOPIC=/scan_top_lidar
SCAN_FRAME=top_lrf_link
rm -rf "$OUT"
# 再生を途中から始めると /tf_static が流れないので、base_footprint -> スキャンの座標系を bag から読んで出す
read -r X Y Z ROLL PITCH YAW < <(cd /app && python3 -c "
import math
from tools.common.bag import read_static_transform
t = read_static_transform('$BAG', 'base_footprint', '$SCAN_FRAME')
r = t[:3, :3]
print(t[0, 3], t[1, 3], t[2, 3], math.atan2(r[2, 1], r[2, 2]), -math.asin(r[2, 0]), math.atan2(r[1, 0], r[0, 0]))
")
echo "base_footprint -> $SCAN_FRAME: x=$X y=$Y z=$Z roll=$ROLL pitch=$PITCH yaw=$YAW"
ros2 run tf2_ros static_transform_publisher --x "$X" --y "$Y" --z "$Z" --roll "$ROLL" --pitch "$PITCH" --yaw "$YAW" \
  --frame-id base_footprint --child-frame-id "$SCAN_FRAME" --ros-args -p use_sim_time:=true &
ros2 run pointcloud_to_laserscan laserscan_to_pointcloud_node --ros-args -p use_sim_time:=true \
  -p target_frame:=base_footprint -p transform_tolerance:=0.05 \
  -r scan_in:=$SCAN_TOPIC -r cloud:=/cloud_top_lidar &
ros2 run kiss_icp odometry_node --ros-args -p use_sim_time:=true \
  -p odom_frame:=lidar_odom -p child_frame:=base_footprint -p max_range:=${MAX_RANGE:-30.0} \
  -p min_range:=${MIN_RANGE:-0.3} -p deskew:=false -p max_points_per_voxel:=50 -p initial_threshold:=2.0 \
  -p min_motion_th:=0.1 -p publish_odom_tf:=false -p publish_alias_tf:=false -p remove_parcentile_rate:=0.15 \
  -r pointcloud_topic:=/cloud_top_lidar -r odometry:=/kissicp/odom &
sleep 3
ros2 bag record -o "$OUT" /kissicp/odom --use-sim-time &
REC=$!
sleep 2
timeout "$(python3 -c "print(int($DUR / $RATE) + 5)")" \
  ros2 bag play "$BAG" --clock 100 --rate "$RATE" --start-offset "$START" --topics $SCAN_TOPIC || true
sleep 3
kill -INT $REC
sleep 2
kill $(jobs -p) 2>/dev/null || true
wait 2>/dev/null || true
ros2 bag info "$OUT" | grep -E "Count|Duration"
