#!/bin/bash
# 評価セット (tools/datasets/localization/sets/) の全条件で、自己位置推定の変種を再生評価する。
# 条件ごとに run_localization_variant.sh を、並列数を制限して実行する。
#
# 使い方:
#   tools/scripts/run_localization_set.sh <名前> [--set <名前|パス>] [--jobs N] [--only ラベル,...] \
#       [run_localization_variant.sh のオプションとパラメータ ...]
#
# オプション:
#   --set <名前|パス>   評価セット (tools/datasets/localization/sets/<名前>.txt。既定 robustness)
#   --jobs N            同時に実行する条件の数 (既定 4。1 条件で Docker コンテナ 1 つ)
#   --only ラベル,...   指定したラベルの条件だけ実行する
#
# 残りの引数は、各条件の行の引数の後ろに付けて run_localization_variant.sh に渡す (例: --runs 3 --monitor supervisor)。
# 変種の名前は <名前>_<ラベル>。集計は compare_localization_set.py で行う。
# ログ: tools/data/variants/set_<名前>/<ラベル>.log
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

SET="robustness"
JOBS=4
ONLY=""
PASS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --set) SET="$2"; shift 2 ;;
    --jobs) JOBS="$2"; shift 2 ;;
    --only) ONLY="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) PASS+=("$1"); shift ;;
  esac
done

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SET_FILE="$SET"
[ -f "$SET_FILE" ] || SET_FILE="tools/datasets/localization/sets/$SET.txt"
[ -f "$SET_FILE" ] || { echo "エラー: 評価セットが見つかりません: $SET" >&2; exit 1; }

LOG_DIR="tools/data/variants/set_$NAME"
mkdir -p "$LOG_DIR"

run_one() {
  local label="$1" dataset="$2"
  shift 2
  if tools/scripts/run_localization_variant.sh "${NAME}_${label}" --dataset "$dataset" "$@" "${PASS[@]}" \
      > "$LOG_DIR/$label.log" 2>&1; then
    echo "[$NAME] $label: 完了"
  else
    echo "[$NAME] $label: 失敗 ($LOG_DIR/$label.log)"
  fi
}

running=0
while read -r label dataset rest; do
  [ -z "$label" ] || [ "${label:0:1}" = "#" ] && continue
  if [ -n "$ONLY" ] && [[ ",$ONLY," != *",$label,"* ]]; then
    continue
  fi
  read -r -a extra <<< "${rest:-}"
  if [ "$running" -ge "$JOBS" ]; then
    wait -n || true
    running=$((running - 1))
  fi
  echo "[$NAME] $label: 開始 ($dataset)"
  run_one "$label" "$dataset" "${extra[@]}" &
  running=$((running + 1))
done < "$SET_FILE"
wait
echo "[$NAME] すべての条件が終わりました。集計: python3 tools/scripts/compare_localization_set.py --set $SET $NAME"
