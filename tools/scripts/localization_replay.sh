#!/bin/bash
# コンテナ内で 1 つの変種の再生評価を実行する (run_localization_variant.sh から呼ばれる)。
#
# 自己位置推定のスタック (map_server / AMCL / GNSS ブリッジ / EKF) だけを起動し、
# rosbag のセンサデータ (/odom, /scan_top_lidar, /navpvt, /gps/fix) を再生して、
# 推定結果を記録し、eval_localization.py で評価する。これを RUNS 回繰り返す (AMCL は乱数を使うため)。
#
# 必須の環境変数:
#   BAG              評価に使う rosbag
#   MAP_YAML         AMCL に使う地図 (map.yaml)
#   GNSS_TRANSFORM   地図の gnss_transform.yaml
#   GT_DIR           評価 bag の SLAM 出力 (pose_graph.json / gnss_transform.yaml)。真値
#   VARIANT_DIR      変種のパラメータ置き場 (ekf_global.yaml / nav2_params.yaml / nav_bridge.yaml)
#   OUT_DIR          出力先
# 任意の環境変数:
#   MAP_GT_DIR       地図を作った走行の SLAM 出力 (別走行を評価するとき)
#   RUNS             繰り返す回数 (既定 1)
#   RATE             再生速度 (既定 1.0。実時間での評価が基本)
#   START_OFFSET     再生を始める bag 先頭からの経過秒 (既定 0)
#   DURATION         再生する長さ [s] (既定 0 = 最後まで)
#   INIT_MODE        gt: 真値を初期姿勢として与える (既定) / gnss: 与えず GNSS による初期化に任せる
#   EVAL_SKIP        INIT_MODE=gt のとき、評価の先頭から除く秒数 (既定 10。初期姿勢を与えた直後の過渡)
#   BUILD_PKGS       起動前に増分ビルドするパッケージ
# ROS の setup.bash は未定義の変数を参照するため、set -u は使わない
set -eo pipefail

: "${BAG:?}" "${MAP_YAML:?}" "${GNSS_TRANSFORM:?}" "${GT_DIR:?}" "${VARIANT_DIR:?}" "${OUT_DIR:?}"
MAP_GT_DIR="${MAP_GT_DIR:-}"
RUNS="${RUNS:-1}"
RATE="${RATE:-1.0}"
START_OFFSET="${START_OFFSET:-0}"
DURATION="${DURATION:-0}"
INIT_MODE="${INIT_MODE:-gt}"
EVAL_SKIP="${EVAL_SKIP:-10}"
BUILD_PKGS="${BUILD_PKGS:-slam_gnss_2d mg_msgs mg_bringup mg_navigation mg_drivers}"

PLAY_TOPICS="/odom /scan_top_lidar /navpvt /gps/fix"
RECORD_TOPICS="/tf /odom /ekf_global_odom /amcl_pose /amcl_pose_origin /odom/gps /localization/status /diagnostics"
READY_TIMEOUT_SEC=180

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

mkdir -p "$OUT_DIR"
echo "[replay] パッケージを増分ビルドします: $BUILD_PKGS"
(cd /root/ros2_ws && colcon build --symlink-install --packages-select $BUILD_PKGS \
  > "$OUT_DIR/build.log" 2>&1) || { echo "エラー: ビルドに失敗しました。$OUT_DIR/build.log を確認してください。" >&2; exit 1; }
source /root/ros2_ws/install/setup.bash

# 実機や他のコンテナの ROS 通信と混ざらないようにする
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=$((100 + RANDOM % 100))
export SIMULATION=true
export USE_RVIZ=false
export WAYPOINT_PATH=/dev/null

LAUNCH_PID=""
RECORD_PID=""
PLAY_PID=""

stop_group() {
  # $1: プロセスグループの代表 pid。SIGINT で止め、残っていたら SIGKILL
  local pid="$1"
  [ -z "$pid" ] && return 0
  kill -INT -- "-$pid" 2>/dev/null || return 0
  for _ in $(seq 1 40); do
    kill -0 -- "-$pid" 2>/dev/null || return 0
    sleep 0.5
  done
  kill -KILL -- "-$pid" 2>/dev/null || true
}

cleanup() {
  stop_group "$PLAY_PID"
  stop_group "$RECORD_PID"
  stop_group "$LAUNCH_PID"
}
trap cleanup EXIT

wait_ready() {
  local waited=0
  while [ "$waited" -lt "$READY_TIMEOUT_SEC" ]; do
    if ros2 lifecycle get /amcl 2>/dev/null | grep -q "active" \
       && ros2 node list 2>/dev/null | grep -q "ekf_global_node"; then
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done
  return 1
}

for run in $(seq 1 "$RUNS"); do
  RUN_DIR="$OUT_DIR/run_$run"
  rm -rf "$RUN_DIR"
  mkdir -p "$RUN_DIR"
  echo "[replay] run $run/$RUNS: スタックを起動します"

  setsid ros2 launch mg_bringup bringup_navigation.launch.py \
    use_navigation:=false use_realsense:=false use_slam_gnss_bridge:=true \
    map_path:="$MAP_YAML" planning_map_path:="$MAP_YAML" \
    gnss_transform_file:="$GNSS_TRANSFORM" \
    ekf_params_file:="$VARIANT_DIR/ekf_global.yaml" \
    nav2_params_file:="$VARIANT_DIR/nav2_params.yaml" \
    bridge_params_file:="$VARIANT_DIR/nav_bridge.yaml" \
    > "$RUN_DIR/launch.log" 2>&1 &
  LAUNCH_PID=$!

  if ! wait_ready; then
    echo "エラー: ${READY_TIMEOUT_SEC}秒以内にスタックが起動しませんでした。$RUN_DIR/launch.log を確認してください。" >&2
    exit 1
  fi

  setsid python3 /app/tools/scripts/loc_recorder.py -o "$RUN_DIR/output" $RECORD_TOPICS \
    > "$RUN_DIR/record.log" 2>&1 &
  RECORD_PID=$!
  sleep 3

  echo "[replay] run $run/$RUNS: 再生を開始します (rate=$RATE, offset=$START_OFFSET, duration=$DURATION)"
  PLAY_CMD=(ros2 bag play "$BAG" --clock --disable-keyboard-controls
            --start-offset "$START_OFFSET" -r "$RATE" --topics $PLAY_TOPICS)
  if [ "$DURATION" != "0" ]; then
    LIMIT=$(python3 -c "print(int($DURATION / $RATE) + 5)")
    PLAY_CMD=(timeout -s INT "$LIMIT" "${PLAY_CMD[@]}")
  fi
  setsid "${PLAY_CMD[@]}" > "$RUN_DIR/play.log" 2>&1 &
  PLAY_PID=$!

  if [ "$INIT_MODE" = "gt" ]; then
    INIT_ARGS=(--gt-dir "$GT_DIR")
    [ -n "$MAP_GT_DIR" ] && INIT_ARGS+=(--map-gt-dir "$MAP_GT_DIR")
    timeout 120 python3 /app/tools/scripts/loc_init_pose.py "${INIT_ARGS[@]}" \
      2>&1 | tee "$RUN_DIR/init_pose.log"
  fi

  # 再生の終了を待つ (グループごと起動したので、pid の終了で判定する)
  while kill -0 "$PLAY_PID" 2>/dev/null; do sleep 1; done
  PLAY_PID=""
  sleep 3

  stop_group "$RECORD_PID"; RECORD_PID=""
  stop_group "$LAUNCH_PID"; LAUNCH_PID=""

  # 初期姿勢を与える場合は、その直後の過渡を評価から除く (gnss なら初期化も含めて評価する)
  EVAL_START=0
  [ "$INIT_MODE" = "gt" ] && EVAL_START="$EVAL_SKIP"
  EVAL_ARGS=("$RUN_DIR/output" --gt-dir "$GT_DIR" --start "$EVAL_START" -o "$RUN_DIR/eval_localization.json")
  [ -n "$MAP_GT_DIR" ] && EVAL_ARGS+=(--map-gt-dir "$MAP_GT_DIR")
  python3 /app/tools/scripts/eval_localization.py "${EVAL_ARGS[@]}" > "$RUN_DIR/eval_localization.log" 2>&1
  echo "[replay] run $run/$RUNS: 評価を保存しました: $RUN_DIR/eval_localization.md"
done

# 使ったパラメータを残し、試行ごとの結果をまとめる
mkdir -p "$OUT_DIR/params"
cp "$VARIANT_DIR"/*.yaml "$OUT_DIR/params/"
python3 /app/tools/scripts/compare_localization.py "$OUT_DIR" -o "$OUT_DIR/summary.md"
