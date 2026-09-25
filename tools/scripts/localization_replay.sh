#!/bin/bash
# コンテナ内で 1 つの変種の再生評価を実行する (run_localization_variant.sh から呼ばれる)。
#
# 自己位置推定のスタック (map_server / AMCL / GNSS ブリッジ / EKF) だけを起動し、
# rosbag のセンサデータ (/odom, /scan_top_lidar, /navpvt, /gps/fix) を再生して、
# 推定結果を記録し、eval_localization.py で評価する。これを RUNS 回繰り返す (AMCL は乱数を使うため)。
# 記録した /odom は、ホイールオドメトリの補正ノード (wheel_odom_corrector_node) を通して /odom/raw から
# /odom に出したものになる (実機と同じ構成)。
#
# 必須の環境変数:
#   BAG              評価に使う rosbag
#   MAP_YAML         AMCL に使う地図 (map.yaml)
#   GNSS_TRANSFORM   地図の gnss_transform.yaml
#   GT_DIR           評価 bag の SLAM 出力 (pose_graph.json / gnss_transform.yaml)。真値
#   VARIANT_DIR      変種のパラメータ置き場 (ekf_global.yaml / nav2_params.yaml / nav_bridge.yaml /
#                    wheel_odom_corrector.yaml)
#   OUT_DIR          出力先
# 任意の環境変数:
#   MAP_GT_DIR       地図を作った走行の SLAM 出力 (別走行を評価するとき)
#   FAULTS           故障定義 YAML (tools/datasets/localization/faults/)。センサ入力に故障を注入する
#   RUNS             繰り返す回数 (既定 1)
#   RATE             再生速度 (既定 1.0。実時間での評価が基本)
#   START_OFFSET     再生を始める bag 先頭からの経過秒 (既定 0)
#   DURATION         再生する長さ [s] (既定 0 = 最後まで)
#   INIT_MODE        gt: 真値を初期姿勢として与える (既定) / gnss: 与えず GNSS による初期化に任せる
#   EVAL_SKIP        INIT_MODE=gt のとき、評価の先頭から除く秒数 (既定 10。初期姿勢を与えた直後の過渡)
#   USE_INITIALIZER  GNSS から AMCL の初期姿勢を与えるノードを起動するか (既定: gt なら false、gnss なら true)
#   MONITOR          自己位置の監視ノード none | watchdog (既定: gt なら none、gnss なら watchdog)
#   BUILD_PKGS       起動前に増分ビルドするパッケージ
# ROS の setup.bash は未定義の変数を参照するため、set -u は使わない
set -eo pipefail

: "${BAG:?}" "${MAP_YAML:?}" "${GNSS_TRANSFORM:?}" "${GT_DIR:?}" "${VARIANT_DIR:?}" "${OUT_DIR:?}"
MAP_GT_DIR="${MAP_GT_DIR:-}"
FAULTS="${FAULTS:-}"
RUNS="${RUNS:-1}"
RATE="${RATE:-1.0}"
START_OFFSET="${START_OFFSET:-0}"
DURATION="${DURATION:-0}"
INIT_MODE="${INIT_MODE:-gt}"
EVAL_SKIP="${EVAL_SKIP:-10}"
# 真値を初期姿勢に与えるときは、GNSS による初期化と監視ノードを止めて推定そのものを見る。
# gnss のときは、実機と同じく初期化ノードと監視ノードを動かす
if [ "$INIT_MODE" = "gt" ]; then
  USE_INITIALIZER="${USE_INITIALIZER:-false}"
  MONITOR="${MONITOR:-none}"
else
  USE_INITIALIZER="${USE_INITIALIZER:-true}"
  MONITOR="${MONITOR:-watchdog}"
fi
BUILD_PKGS="${BUILD_PKGS:-slam_gnss_2d mg_msgs mg_bringup mg_navigation mg_drivers}"

PLAY_TOPICS="/odom /scan_top_lidar /navpvt /gps/fix"
RECORD_TOPICS="/tf /odom /odom/raw /ekf_global_odom /amcl_pose /amcl_pose_origin /odom/gps /localization/status /diagnostics"
READY_TIMEOUT_SEC=180
# 初期姿勢が AMCL に届かなかった試行をやり直す最大回数
MAX_ATTEMPTS=2

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

mkdir -p "$OUT_DIR"
echo "[replay] パッケージを増分ビルドします: $BUILD_PKGS"
(cd /root/ros2_ws && colcon build --symlink-install --packages-select $BUILD_PKGS \
  > "$OUT_DIR/build.log" 2>&1) || { echo "エラー: ビルドに失敗しました。$OUT_DIR/build.log を確認してください。" >&2; exit 1; }
source /root/ros2_ws/install/setup.bash

# 実機や他のコンテナの ROS 通信と混ざらないようにする
export ROS_LOCALHOST_ONLY=1
# 並列に実行する他のコンテナと重ならないよう、ホスト側 (run_localization_variant.sh) が割り当てた ID を使う
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + RANDOM % 100))}"
export SIMULATION=true
export USE_RVIZ=false
export WAYPOINT_PATH=/dev/null

LAUNCH_PID=""
RECORD_PID=""
PLAY_PID=""
INJECTOR_PID=""

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
  stop_group "$INJECTOR_PID"
  stop_group "$RECORD_PID"
  stop_group "$LAUNCH_PID"
}
trap cleanup EXIT

wait_ready() {
  local waited=0
  while [ "$waited" -lt "$READY_TIMEOUT_SEC" ]; do
    # ros2 のコマンドは discovery で固まることがあるので、必ずタイムアウトをつける
    if timeout 20 ros2 lifecycle get /amcl 2>/dev/null | grep -q "active" \
       && timeout 20 ros2 node list 2>/dev/null | grep -q "ekf_global_node"; then
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done
  return 1
}

# 1 回分の試行。初期姿勢が AMCL に届かず、試行が無効になったときは 1 を返す。
run_once() {
  local run="$1"
  local run_dir="$OUT_DIR/run_$run"
  rm -rf "$run_dir"
  mkdir -p "$run_dir"
  echo "[replay] run $run/$RUNS: スタックを起動します"

  setsid ros2 launch mg_bringup bringup_navigation.launch.py \
    use_navigation:=false use_realsense:=false use_slam_gnss_bridge:=true \
    use_odom_corrector:=true \
    use_gnss_amcl_initializer:="$USE_INITIALIZER" localization_monitor:="$MONITOR" \
    map_path:="$MAP_YAML" planning_map_path:="$MAP_YAML" \
    gnss_transform_file:="$GNSS_TRANSFORM" \
    ekf_params_file:="$VARIANT_DIR/ekf_global.yaml" \
    nav2_params_file:="$VARIANT_DIR/nav2_params.yaml" \
    bridge_params_file:="$VARIANT_DIR/nav_bridge.yaml" \
    odom_corrector_params_file:="$VARIANT_DIR/wheel_odom_corrector.yaml" \
    supervisor_params_file:="$VARIANT_DIR/localization_supervisor.yaml" \
    > "$run_dir/launch.log" 2>&1 &
  LAUNCH_PID=$!

  if ! wait_ready; then
    echo "[replay] run $run/$RUNS: ${READY_TIMEOUT_SEC}秒以内にスタックが起動しませんでした (この試行は無効。$run_dir/launch.log)" >&2
    stop_group "$LAUNCH_PID"; LAUNCH_PID=""
    return 1
  fi

  setsid python3 /app/tools/scripts/loc_recorder.py -o "$run_dir/output" $RECORD_TOPICS \
    > "$run_dir/record.log" 2>&1 &
  RECORD_PID=$!
  sleep 3

  echo "[replay] run $run/$RUNS: 再生を開始します (rate=$RATE, offset=$START_OFFSET, duration=$DURATION)"
  # 故障を注入するときは、対象のトピックを /fault/ 以下へ付け替えて再生し、注入ノードが中継する
  local remaps=(/odom:=/odom/raw)
  if [ -n "$FAULTS" ]; then
    remaps=(/odom:=/fault/odom /navpvt:=/fault/navpvt /scan_top_lidar:=/fault/scan_top_lidar)
    setsid python3 /app/tools/scripts/fault_injector.py "$FAULTS" > "$run_dir/fault_injector.log" 2>&1 &
    INJECTOR_PID=$!
    sleep 2
  fi
  local play_cmd=(ros2 bag play "$BAG" --clock --disable-keyboard-controls
                  --start-offset "$START_OFFSET" -r "$RATE" --topics $PLAY_TOPICS
                  --remap "${remaps[@]}")
  if [ "$DURATION" != "0" ]; then
    local limit
    limit=$(python3 -c "print(int($DURATION / $RATE) + 5)")
    play_cmd=(timeout -s INT "$limit" "${play_cmd[@]}")
  fi
  setsid "${play_cmd[@]}" > "$run_dir/play.log" 2>&1 &
  PLAY_PID=$!

  if [ "$INIT_MODE" = "gt" ]; then
    local init_args=(--gt-dir "$GT_DIR")
    [ -n "$MAP_GT_DIR" ] && init_args+=(--map-gt-dir "$MAP_GT_DIR")
    timeout 120 python3 /app/tools/scripts/loc_init_pose.py "${init_args[@]}" \
      2>&1 | tee "$run_dir/init_pose.log"
  fi

  # 再生の終了を待つ (グループごと起動したので、pid の終了で判定する)
  while kill -0 "$PLAY_PID" 2>/dev/null; do sleep 1; done
  PLAY_PID=""
  sleep 3

  stop_group "$RECORD_PID"; RECORD_PID=""
  stop_group "$INJECTOR_PID"; INJECTOR_PID=""
  stop_group "$LAUNCH_PID"; LAUNCH_PID=""

  # AMCL は初期姿勢を受け取るたびに "Setting pose (<スタンプ>)" を出す。AMCL 自身の初期値のスタンプは 0
  if [ "$INIT_MODE" = "gt" ] && ! grep -qE "Setting pose \([1-9]" "$run_dir/launch.log"; then
    echo "[replay] run $run/$RUNS: 初期姿勢が AMCL に届きませんでした (この試行は無効)" >&2
    return 1
  fi

  # 初期姿勢を与える場合は、その直後の過渡を評価から除く (gnss なら初期化も含めて評価する)
  local eval_start=0
  [ "$INIT_MODE" = "gt" ] && eval_start="$EVAL_SKIP"
  local eval_args=("$run_dir/output" --gt-dir "$GT_DIR" --start "$eval_start"
                   -o "$run_dir/eval_localization.json")
  [ -n "$MAP_GT_DIR" ] && eval_args+=(--map-gt-dir "$MAP_GT_DIR")
  [ -n "$FAULTS" ] && eval_args+=(--faults "$FAULTS")
  # この関数は until の条件として呼ばれ set -e が効かないため、失敗を明示的に検出する
  python3 /app/tools/scripts/eval_localization.py "${eval_args[@]}" > "$run_dir/eval_localization.log" 2>&1 \
    || { echo "エラー: 評価に失敗しました。$run_dir/eval_localization.log を確認してください。" >&2; exit 1; }
  echo "[replay] run $run/$RUNS: 評価を保存しました: $run_dir/eval_localization.md"
}

for run in $(seq 1 "$RUNS"); do
  attempt=1
  until run_once "$run"; do
    if [ "$attempt" -ge "$MAX_ATTEMPTS" ]; then
      echo "エラー: run $run が $MAX_ATTEMPTS 回続けて無効でした。" >&2
      exit 1
    fi
    attempt=$((attempt + 1))
    echo "[replay] run $run/$RUNS: やり直します ($attempt/$MAX_ATTEMPTS)"
  done
done

# 使ったパラメータを残し、試行ごとの結果をまとめる
mkdir -p "$OUT_DIR/params"
cp "$VARIANT_DIR"/*.yaml "$OUT_DIR/params/"
python3 /app/tools/scripts/compare_localization.py "$OUT_DIR" -o "$OUT_DIR/summary.md"
