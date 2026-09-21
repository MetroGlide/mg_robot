#!/bin/bash
# パラメータの変種を指定してオフラインSLAMを実行し、RTK 比較評価までを行う。
#
# 使い方:
#   tools/scripts/run_slam_variant.sh <名前> [key.path=value ...]
# 例:
#   tools/scripts/run_slam_variant.sh deskew_fwd deskew.enabled=true deskew.direction=1
#
# - params/slam_gnss_2d.yaml を複製して key.path=value で書き換え、tools/data/variants/<名前>.yaml に保存する
# - .env の ROSBAG_FILE をオフライン実行し、地図と pose_graph を <bag>/eval/<名前>/ に保存する
# - eval_slam.py の評価結果と処理時間を標準出力に表示し、<bag>/eval/<名前>/ にも保存する
# 環境変数: EXTRA_OPTS (launch 引数の追加。例: "start_time:=750.0 end_time:=1000.0")
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "使い方: $0 <名前> [key.path=value ...]" >&2
  exit 1
fi
NAME="$1"
shift

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

VARIANT_DIR="tools/data/variants"
mkdir -p "$VARIANT_DIR"
python3 tools/scripts/patch_params.py \
  slam_gnss_2d/slam_gnss_2d/params/slam_gnss_2d.yaml "$VARIANT_DIR/$NAME.yaml" "$@"

BAG="$(docker compose -f compose.yaml config | grep -m1 'ROSBAG_FILE:' | awk '{print $2}')"
if [ -z "$BAG" ]; then
  echo "エラー: ROSBAG_FILE が .env から解決できませんでした。" >&2
  exit 1
fi
HOST_OUT="${HOME}/ros2_data${BAG#/root/ros2_data}/eval/$NAME"
CONTAINER="mg_robot-offline-slam-gnss-2d-1"

docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
OPTS="params_file:=/app/$VARIANT_DIR/$NAME.yaml ${EXTRA_OPTS:-}" \
  docker compose -f compose.yaml up -d offline-slam-gnss-2d >/dev/null 2>&1

# 出力先はコンテナ (root) 側の所有なので、作成と書き込みはコンテナ経由で行う
docker exec "$CONTAINER" mkdir -p "$BAG/eval/$NAME"

echo "[$NAME] オフラインSLAMを実行中..."
for _ in $(seq 1 400); do
  if docker logs "$CONTAINER" 2>&1 | grep -q "process_frame breakdown"; then
    break
  fi
  if docker logs "$CONTAINER" 2>&1 | grep -q "slam_offline_node.*process has died"; then
    echo "エラー: slam_offline_node が異常終了しました。ログ:" >&2
    docker logs "$CONTAINER" 2>&1 | grep -B3 "process has died" | grep -v robot_state | cut -c1-300 >&2
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
    exit 1
  fi
  if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]; then
    echo "エラー: コンテナが終了しました。ログ末尾:" >&2
    docker logs --tail 20 "$CONTAINER" >&2
    exit 1
  fi
  sleep 3
done

if ! docker logs "$CONTAINER" 2>&1 | grep -q "process_frame breakdown"; then
  echo "エラー: 20分以内に処理が完了しませんでした。" >&2
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  exit 1
fi
docker logs "$CONTAINER" 2>&1 | grep "\[timing\]" | sed 's/^.*\] \[slam_gnss_2d[^]]*\]: //' \
  | docker exec -i "$CONTAINER" bash -c "cat > $BAG/eval/$NAME/timing.log" || true
docker exec -i "$CONTAINER" bash -c "cat > $BAG/eval/$NAME/params.yaml" < "$VARIANT_DIR/$NAME.yaml"

docker exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash && \
  source /root/ros2_ws/install/setup.bash && \
  ros2 run slam_gnss_2d save_slam_map_cli -d $BAG/eval/$NAME" >/dev/null 2>&1
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

make -s bag-eval-slam SLAM_DIR="$BAG/eval/$NAME" OUT_DIR="$BAG/eval/$NAME" >/dev/null 2>&1

echo "===== [$NAME] 処理時間 ====="
cat "$HOST_OUT/timing.log"
echo "===== [$NAME] 評価 ====="
cat "$HOST_OUT/eval_slam.md"
