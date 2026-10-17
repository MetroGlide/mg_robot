# エントリポイント。ターゲットは分類ごとに make/*.mk へ分けている。
# 一覧は make help、共通の引数は make help-args、全体の対応表は doc/commands.md を参照。
#
#   make/common.mk     共通の変数・関数 (COMPOSE の解決、USE_GPU、.env の読み取り)
#   make/run.mk        実行 (実機・シミュレーション・再生・RViz2・Web UI)
#   make/build.mk      ビルド (Docker イメージ)
#   make/container.mk  コンテナ操作 (シェル・ログ・状態・停止)
#   make/test.mk       テスト (pytest・UI 検査・シナリオテスト)
#   make/tools.mk      ツール (rosbag の解析・可視化・評価)
#   make/help.mk       ヘルプ

.DEFAULT_GOAL := help

include make/common.mk
include make/help.mk
include make/run.mk
include make/build.mk
include make/container.mk
include make/test.mk
include make/tools.mk
