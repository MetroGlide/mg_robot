#!/bin/bash
# 自己位置推定 (オドメトリ + AMCL + GNSS の EKF 融合) のパラメータ変種を、rosbag の再生で評価する。
#
# 使い方:
#   tools/scripts/run_localization_variant.sh <名前> [オプション] [プレフィックス.キー=値 ...]
#
# オプション:
#   --dataset <名前|パス>    データセット (tools/datasets/localization/。既定 map043837_eval051635_short)
#   --runs N                 繰り返す回数 (既定 3。AMCL は乱数を使うため分布で比べる)
#   --rate R                 再生速度 (既定 1.0。実時間で評価するのが基本)
#   --init gt|gnss           初期姿勢 (既定 gt: 真値を与える / gnss: GNSS による初期化に任せる)
#   --start S / --duration D 評価する区間 (データセットの値を上書き)
#   --cpus N                 コンテナに使わせる CPU 数 (実機相当の負荷にしたいとき)
#
# 書き換えるパラメータ (プレフィックス.キー=値。キーはドット区切りで、1 行の値だけ書き換えられる):
#   ekf.<key>     mg_drivers/params/ekf_global.yaml の ekf_global_node
#   amcl.<key>    mg_navigation/params/nav2_params.yaml の amcl
#   bridge.<key>  slam_gnss_2d/params/nav_bridge.yaml の slam_gnss_nav_bridge
#
# 例:
#   tools/scripts/run_localization_variant.sh baseline
#   tools/scripts/run_localization_variant.sh lag_on --runs 3 ekf.smooth_lagged_data=true ekf.history_length=1.0
#   tools/scripts/run_localization_variant.sh light_amcl amcl.max_particles=2000 amcl.max_beams=240 --cpus 4
#
# 出力: <評価 bag>/eval_loc/<データセット>/<名前>/{params/, run_N/, summary.md}
# 環境変数: IMAGE (使う Docker イメージ。既定 mg_develop:latest)
set -euo pipefail

usage() {
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//' >&2
  exit 1
}

if [ $# -lt 1 ] || [ "${1:0:1}" = "-" ]; then
  usage
fi
NAME="$1"
shift

DATASET="map043837_eval051635_short"
RUNS=3
RATE=1.0
INIT_MODE=gt
CPUS=""
START_OVERRIDE=""
DURATION_OVERRIDE=""
EKF_UPDATES=()
AMCL_UPDATES=()
BRIDGE_UPDATES=()

while [ $# -gt 0 ]; do
  case "$1" in
    --dataset) DATASET="$2"; shift 2 ;;
    --runs) RUNS="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --init) INIT_MODE="$2"; shift 2 ;;
    --start) START_OVERRIDE="$2"; shift 2 ;;
    --duration) DURATION_OVERRIDE="$2"; shift 2 ;;
    --cpus) CPUS="$2"; shift 2 ;;
    -h|--help) usage ;;
    ekf.*=*) EKF_UPDATES+=("ekf_global_node:${1#ekf.}"); shift ;;
    amcl.*=*) AMCL_UPDATES+=("amcl:${1#amcl.}"); shift ;;
    bridge.*=*) BRIDGE_UPDATES+=("slam_gnss_nav_bridge:${1#bridge.}"); shift ;;
    *) echo "エラー: 解釈できない引数です: $1" >&2; usage ;;
  esac
done

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

eval "$(python3 tools/scripts/loc_dataset.py "$DATASET")"
[ -n "$START_OVERRIDE" ] && START_OFFSET="$START_OVERRIDE"
[ -n "$DURATION_OVERRIDE" ] && DURATION="$DURATION_OVERRIDE"

# 変種のパラメータファイルを、リポジトリの値から複製して書き換える
VARIANT_REL="tools/data/variants/loc_${NAME}"
mkdir -p "$VARIANT_REL"
patch() {
  # $1: 元ファイル, $2: 出力ファイル名, 以降: node:key=value
  local src="$1" dst="$VARIANT_REL/$2"
  shift 2
  python3 tools/scripts/patch_params.py "$src" "$dst" "$@"
}
patch mg_drivers/params/ekf_global.yaml ekf_global.yaml "${EKF_UPDATES[@]}"
patch mg_navigation/params/nav2_params.yaml nav2_params.yaml "${AMCL_UPDATES[@]}"
patch slam_gnss_2d/slam_gnss_2d/params/nav_bridge.yaml nav_bridge.yaml "${BRIDGE_UPDATES[@]}"

OUT_DIR="${BAG}/eval_loc/${DATASET_NAME}/${NAME}"
IMAGE="${IMAGE:-mg_develop:latest}"

DOCKER_ARGS=(run --rm --network host
  -v "$ROOT":/app -v "$HOME/ros2_data":/root/ros2_data
  -e BAG="$BAG" -e MAP_YAML="$MAP_YAML" -e GNSS_TRANSFORM="$GNSS_TRANSFORM"
  -e GT_DIR="$GT_DIR" -e MAP_GT_DIR="$MAP_GT_DIR"
  -e VARIANT_DIR="/app/$VARIANT_REL" -e OUT_DIR="$OUT_DIR"
  -e RUNS="$RUNS" -e RATE="$RATE" -e START_OFFSET="$START_OFFSET" -e DURATION="$DURATION"
  -e INIT_MODE="$INIT_MODE")
[ -n "$CPUS" ] && DOCKER_ARGS+=(--cpus "$CPUS")

echo "[$NAME] データセット $DATASET_NAME を再生して評価します (試行 $RUNS 回)"
docker "${DOCKER_ARGS[@]}" "$IMAGE" bash /app/tools/scripts/localization_replay.sh
echo "[$NAME] 結果: ${OUT_DIR/#\/root\/ros2_data/$HOME/ros2_data}/summary.md"
