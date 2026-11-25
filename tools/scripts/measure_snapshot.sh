#!/bin/bash
# 計測 (シナリオテスト・再生) を、指定のコミットのコードの書き出しから make で実行する。
# シナリオのイメージは /app (リポジトリ) のソースを参照するので、作業ツリーで流すと、計測の途中で編集したコードが
# 後から起動するシナリオに混ざる。書き出しから流せば、計測の間も作業ツリーで実装を続けられる。
#
# 使い方:
#   tools/scripts/measure_snapshot.sh [<コミット>] -- <make の引数 ...>
#
# 例:
#   tools/scripts/measure_snapshot.sh HEAD -- build svc=scenario-test
#   tools/scripts/measure_snapshot.sh HEAD -- scenario-test-all TAGS=anomaly KNOWN=1 REPEAT=3 SHARDS=3
#
# 書き出し先は <MEASURE_ROOT>/<コミットの短い名前> (MEASURE_ROOT の既定はリポジトリの隣の mg_robot_measure)。
# 同じコミットの書き出しがあれば使い回す。.env はリポジトリのものを写す。
# イメージの名前は compose で固定なので、build はリポジトリのイメージを書き換える。計測の間は作り直さないこと。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
COMMIT="HEAD"
if [ $# -gt 0 ] && [ "$1" != "--" ]; then
  COMMIT="$1"
  shift
fi
if [ $# -eq 0 ] || [ "$1" != "--" ]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//' >&2
  exit 1
fi
shift

REV="$(git -C "$ROOT" rev-parse --short "$COMMIT")"
MEASURE_ROOT="${MEASURE_ROOT:-$(dirname "$ROOT")/mg_robot_measure}"
DIR="$MEASURE_ROOT/$REV"
if [ ! -f "$DIR/.measured_rev" ]; then
  mkdir -p "$DIR"
  git -C "$ROOT" archive "$REV" | tar -x -C "$DIR"
  echo "$REV" > "$DIR/.measured_rev"
fi
cp "$ROOT/.env" "$DIR/.env"
echo "[measure] $REV のコードの書き出し ($DIR) で make $*" >&2
exec make -C "$DIR" "$@"
