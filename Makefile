USE_GPU := $(shell grep -E '^USE_GPU=' .env 2>/dev/null | cut -d= -f2 | tr -d '[:space:]')
USE_GPU ?= none

COMPOSE_BASE := docker compose -f compose.yaml

ifeq ($(USE_GPU),nvidia)
  COMPOSE := $(COMPOSE_BASE) -f compose.gpu.nvidia.yaml
else ifeq ($(USE_GPU),amd)
  COMPOSE := $(COMPOSE_BASE) -f compose.gpu.amd.yaml
else
  COMPOSE := $(COMPOSE_BASE)
endif

# make <service> DETACH=1 でバックグラウンド起動
_up_flags = $(if $(DETACH),-d,)
_compose_opts = $(if $(OPTS),OPTS="$(OPTS)" )

.PHONY: slam navigation rosbag-replay obstacle-detection-replay gazebo-simulation develop \
        scenario-test scenario-test-all scenario-test-attach scenario-validate scenario-env scenario-env-stop \
        slam-gnss-2d offline-slam-gnss-2d \
        shell shell-develop logs ps restart \
        build build-all build-no-cache build-robot build-real build-robot-no-cache build-real-no-cache build-sim \
        _collect-deps \
        rviz2 rviz2-slam rviz2-navigation down xhost config \
        test bag-summary bag-plot-gnss bag-plot-scans bag-eval-slam \
        diagnostics system-manager foxglove-bridge web-ui web-ui-dev ui-all ui-dev-all ui-lint ui-test

# --- サービス起動 ---

slam:
	$(_compose_opts)$(COMPOSE) up $(_up_flags) slam

navigation:
	$(_compose_opts)$(COMPOSE) up $(_up_flags) navigation

rosbag-replay:
	$(_compose_opts)$(COMPOSE) run --rm -it rosbag-replay

# rosbag-replay と別端末で起動する。点群復元 + 障害物検出 + RViz
# make obstacle-detection-replay OPTS="rviz:=false use_color:=true"
obstacle-detection-replay:
	$(_compose_opts)$(COMPOSE) run --rm -it obstacle-detection-replay

slam-gnss-2d:
	$(_compose_opts)$(COMPOSE) up $(_up_flags) slam-gnss-2d

offline-slam-gnss-2d:
	$(if $(BAG),ROSBAG_FILE=$(BAG) )$(if $(ROSBAG_FILE),ROSBAG_FILE=$(ROSBAG_FILE) )$(_compose_opts)$(COMPOSE) up $(_up_flags) offline-slam-gnss-2d

gazebo-simulation:
	$(_compose_opts)$(COMPOSE) up $(_up_flags) gazebo-simulation

# --- シナリオテスト (詳細: mg_scenario_test/README.md) ---
# シナリオは名前 (mg_scenario_test/scenarios/<name>.yaml) またはパスで指定する。
# 終了コード: 0=PASSED, 1=FAILED, 2=ERROR。結果は ${ROS2_DATA_PATH}/scenario_results/<日時>/ に保存される。
_scenario_dir = /app/mg_scenario_test/scenarios
_scenario_dirs = --scenario-dir $(_scenario_dir)/regression --scenario-dir $(_scenario_dir)/examples
_scenario_results = --results-dir /root/ros2_data/scenario_results/$$(date +%Y%m%d_%H%M%S)
# ROBOT=<実機PCのIP> を指定すると、ナビゲーションスタックを実機PC (mg_system_manager :8001) に起動させ、
# シミュレータだけをこの PC で起動する。両方のPCの .env に MG_REMOTE_PEER・SCENARIO_ROS_DOMAIN_ID を設定しておく
_scenario_domain_id := $(shell grep -E '^SCENARIO_ROS_DOMAIN_ID=' .env 2>/dev/null | cut -d= -f2 | tr -d '[:space:]')
_remote_env = $(if $(ROBOT),-e ROS_DOMAIN_ID=$(or $(_scenario_domain_id),42) -e CYCLONEDDS_URI=file:///app/docker/cyclonedds/remote.xml -e MG_REMOTE_PEER=$(ROBOT),)
_remote_arg = $(if $(ROBOT),--remote-stack http://$(ROBOT):8001)
_scenario_run = $(COMPOSE) run --rm $(_remote_env) -e SCENARIO_ARGS

# make scenario-test SCENARIO=nav_basic_goal [GUI=1] [PROFILE=mg01] [ROBOT=<実機PCのIP>]
# シミュレータ・ナビゲーションごと起動して 1 本実行する (デフォルトはヘッドレス)。
# SCENARIO は regression/・examples/ 配下のシナリオ名 (拡張子なし)、またはコンテナ内のパス
scenario-test:
	$(_scenario_run)="run $(SCENARIO) $(_scenario_dirs) $(_scenario_results) $(_remote_arg) $(if $(GUI),--gui) $(if $(PROFILE),--profile $(PROFILE))" scenario-test

# make scenario-test-all [TIER=smoke|full] [TAGS=a,b] [REPEAT=N] [EXAMPLES=1] [GUI=1] [ROBOT=<実機PCのIP>]
# 回帰テスト (scenarios/regression) を、1 本ごとにスタックを起動し直して実行する。
# TIER=smoke は smoke タグのみ (変更ごとの確認用)、省略または full は全件。EXAMPLES=1 で examples/ も含める。
# known_issue タグ (既知の問題で失敗するシナリオ) は既定で除外する。KNOWN=1 で含める。GUI=1 でシミュレータの GUI を表示する
scenario-test-all:
	$(_scenario_run)="run-all $(_scenario_dir)/regression $(if $(EXAMPLES),$(_scenario_dir)/examples) $(_scenario_results) $(_remote_arg) $(if $(KNOWN),,--exclude-tags known_issue) $(if $(filter smoke,$(TIER)),--tags smoke) $(if $(TAGS),--tags $(TAGS)) $(if $(REPEAT),--repeat $(REPEAT)) $(if $(GUI),--gui) $(if $(PROFILE),--profile $(PROFILE))" scenario-test

# make scenario-test-attach SCENARIO=... (make scenario-env の起動が前提。シミュレータを起動し直さずに繰り返し実行する)
scenario-test-attach:
	$(_scenario_run)="run $(SCENARIO) $(_scenario_dirs) $(_scenario_results) --attach $(if $(PROFILE),--profile $(PROFILE))" scenario-test

# make scenario-env [GUI=1] [PROFILE=mg01] [WORLD=warehouse]
# attach モード用に、プロファイルのシミュレータとナビゲーションスタックをバックグラウンドで起動したままにする
scenario-env:
	SCENARIO_ENV_ARGS="profile:=$(or $(PROFILE),mg01) world:=$(or $(WORLD),warehouse) headless:=$(if $(GUI),false,true)" $(COMPOSE) up -d --force-recreate scenario-env

scenario-env-stop:
	$(COMPOSE) stop scenario-env

# make scenario-validate  (シミュレータ不要。同梱シナリオの YAML を検証する)
scenario-validate:
	$(_scenario_run)="validate $(_scenario_dir)" --no-deps scenario-test

develop:
	$(COMPOSE) up -d develop
	$(if $(ATTACH),$(COMPOSE) exec develop bash,)

# --- シェルアクセス ---

shell:
ifndef svc
	$(error svc is required. Usage: make shell svc=<service-name>)
endif
	$(COMPOSE) exec $(svc) bash

shell-develop:
	$(COMPOSE) up -d develop
	$(COMPOSE) exec develop bash

# --- ログ ---

logs:
ifndef svc
	$(error svc is required. Usage: make logs svc=<service-name>)
endif
	$(COMPOSE) logs -f $(svc)

# --- コンテナ状態 ---

ps:
	$(COMPOSE) ps

restart:
ifndef svc
	$(error svc is required. Usage: make restart svc=<service-name>)
endif
	$(COMPOSE) restart $(svc)

# --- ビルド ---

_collect-deps:
	bash docker/collect_deps.sh

build: _collect-deps
ifndef svc
	$(error svc is required. Usage: make build svc=<service-name>)
endif
	$(COMPOSE_BASE) build $(svc)

# 実機向け一括ビルド（Gazeboシミュレータを除外: runtime, develop, web-ui のみ）
build-robot: _collect-deps
	$(COMPOSE_BASE) build slam develop web-ui

build-real: build-robot

# 実機向けキャッシュ無効ビルド
build-robot-no-cache: _collect-deps
	$(COMPOSE_BASE) build --no-cache slam develop web-ui

build-real-no-cache: build-robot-no-cache

# シミュレータ含む一括ビルド
build-sim: _collect-deps
	$(COMPOSE_BASE) build slam develop gazebo-simulation web-ui

build-all: build-sim

build-no-cache: _collect-deps
ifndef svc
	$(error svc is required. Usage: make build-no-cache svc=<service-name>)
endif
	$(COMPOSE_BASE) build --no-cache $(svc)

# --- 停止 ---

down:
	$(COMPOSE) down

# --- 再最適化 ---
# 実行例: make reoptimize [INPUT_DIR=/app/maps/latest] [SAVE_DIR=/app/maps/latest_opt] [BAG_PATH=/app/bags/my_bag]
# ※ .env に各環境変数を設定している場合は引数なしで実行可能
reoptimize:
	$(if $(INPUT_DIR),INPUT_DIR=$(INPUT_DIR) )$(if $(SAVE_DIR),SAVE_DIR=$(SAVE_DIR) )$(if $(BAG_PATH),BAG_PATH=$(BAG_PATH) )$(_compose_opts)$(COMPOSE) run --rm reoptimize-slam

# --- rosbag 解析・可視化ツール群 ---
# 保存先の切り替え:
#   デフォルト: rosbag ディレクトリに出力
#   TO_TOOLS=1 または OUT_DIR=tools: tools/data/ に出力
#   OUT_DIR=<dir>: 指定ディレクトリに出力
#   OUT=<file>: 指定ファイルパスに出力
define _resolve_bag_output_opts
$(strip \
  $(if $(OUT),-o "$(OUT)", \
    $(if $(filter 1 true,$(TO_TOOLS)$(TOOLS)),--output-dir /app/tools/data, \
      $(if $(filter tools,$(OUT_DIR)),--output-dir /app/tools/data, \
        $(if $(OUT_DIR),--output-dir "$(OUT_DIR)",--output-to-bag-dir) \
      ) \
    ) \
  ) \
)
endef

# --- rosbag 統計サマリー ---
# 実行例:
#   make bag-summary
#   make bag-summary BAG=/root/ros2_data/rosbag/TC2026/20260913/record_all_20260913_055508
#   make bag-summary TO_TOOLS=1
#   make bag-summary OPTS="--info-only"
bag-summary:
	$(COMPOSE) run --rm --no-deps $(if $(BAG),-e BAG="$(BAG)" )$(if $(BAG_PATH),-e BAG_PATH="$(BAG_PATH)" )develop bash -c \
	  "source /opt/ros/humble/setup.bash && \
	   source /root/ros2_ws/install/setup.bash && \
	   TARGET_BAG=\"\$${BAG:-\$${BAG_PATH:-\$$ROSBAG_FILE}}\" && \
	   if [ -z \"\$$TARGET_BAG\" ]; then \
	     echo 'エラー: 解析対象の rosbag が指定されていません。.env に ROSBAG_FILE を設定するか、BAG=/path/to/bag を指定してください。' >&2; \
	     exit 1; \
	   fi && \
	   python3 /app/tools/scripts/rosbag_summary.py \"\$$TARGET_BAG\" $(_resolve_bag_output_opts) --all $(OPTS)"

# --- GNSS軌跡・Fix状態の可視化 ---
# 実行例:
#   make bag-plot-gnss
#   make bag-plot-gnss CIRCLES=1
#   make bag-plot-gnss CIRCLES=1 SCALE=5
#   make bag-plot-gnss TO_TOOLS=1 SCALE=10
#   make bag-plot-gnss BAG=/path/to/bag OPTS="--circle-step 1"
bag-plot-gnss:
	$(COMPOSE) run --rm --no-deps $(if $(BAG),-e BAG="$(BAG)" )$(if $(BAG_PATH),-e BAG_PATH="$(BAG_PATH)" )develop bash -c \
	  "source /opt/ros/humble/setup.bash && \
	   source /root/ros2_ws/install/setup.bash && \
	   TARGET_BAG=\"\$${BAG:-\$${BAG_PATH:-\$$ROSBAG_FILE}}\" && \
	   if [ -z \"\$$TARGET_BAG\" ]; then \
	     echo 'エラー: 解析対象の rosbag が指定されていません。.env に ROSBAG_FILE を設定するか、BAG=/path/to/bag を指定してください。' >&2; \
	     exit 1; \
	   fi && \
	   python3 /app/tools/scripts/plot_gnss_trajectory.py \"\$$TARGET_BAG\" $(_resolve_bag_output_opts) $(if $(filter 1 true,$(CIRCLES)$(ACC_CIRCLES)),--accuracy-circles )$(if $(SCALE),--circle-scale $(SCALE) )$(if $(CIRCLE_SCALE),--circle-scale $(CIRCLE_SCALE) )$(OPTS)"

# --- LiDARスキャン点群の可視化 ---
# 実行例:
#   make bag-plot-scans
#   make bag-plot-scans TO_TOOLS=1 NODES=1:20
#   make bag-plot-scans BAG=/path/to/bag
bag-plot-scans:
	$(COMPOSE) run --rm --no-deps $(if $(BAG),-e BAG="$(BAG)" )$(if $(BAG_PATH),-e BAG_PATH="$(BAG_PATH)" )develop bash -c \
	  "source /opt/ros/humble/setup.bash && \
	   source /root/ros2_ws/install/setup.bash && \
	   TARGET_BAG=\"\$${BAG:-\$${BAG_PATH:-\$$ROSBAG_FILE}}\" && \
	   if [ -z \"\$$TARGET_BAG\" ]; then \
	     echo 'エラー: 解析対象の rosbag が指定されていません。.env に ROSBAG_FILE を設定するか、BAG=/path/to/bag を指定してください。' >&2; \
	     exit 1; \
	   fi && \
	   python3 /app/tools/scripts/plot_lidar_scans.py \"\$$TARGET_BAG\" $(_resolve_bag_output_opts) $(if $(NODES),--nodes $(NODES) )$(OPTS)"

# --- SLAM 出力の RTK(GNSS) 比較評価 ---
# SLAM 出力ディレクトリ (pose_graph.json / gnss_transform.yaml / map.yaml) を NavPVT と比較する。
# 実行例:
#   make bag-eval-slam SLAM_DIR=/root/ros2_data/rosbag/TC2026/20260913/record_slam_20260913_043837/20260921_1145
#   make bag-eval-slam SLAM_DIR=<dir> TO_TOOLS=1 OPTS="--time-offset 0.2"
bag-eval-slam:
	$(COMPOSE) run --rm --no-deps $(if $(BAG),-e BAG="$(BAG)" )$(if $(BAG_PATH),-e BAG_PATH="$(BAG_PATH)" )develop bash -c \
	  "source /opt/ros/humble/setup.bash && \
	   source /root/ros2_ws/install/setup.bash && \
	   TARGET_BAG=\"\$${BAG:-\$${BAG_PATH:-\$$ROSBAG_FILE}}\" && \
	   if [ -z \"\$$TARGET_BAG\" ]; then \
	     echo 'エラー: 解析対象の rosbag が指定されていません。.env に ROSBAG_FILE を設定するか、BAG=/path/to/bag を指定してください。' >&2; \
	     exit 1; \
	   fi && \
	   if [ -z \"$(SLAM_DIR)\" ]; then \
	     echo 'エラー: SLAM_DIR=<pose_graph.json と gnss_transform.yaml を含むディレクトリ> を指定してください。' >&2; \
	     exit 1; \
	   fi && \
	   python3 /app/tools/scripts/eval_slam.py \"\$$TARGET_BAG\" --slam-dir \"$(SLAM_DIR)\" $(_resolve_bag_output_opts) $(OPTS)"

# --- テスト ---
# 全テスト: make test
# 特定パッケージ: make test pkg=mg_waypoint_navigation
# pytestオプション: make test OPTS="-k test_bag"
test:
	$(COMPOSE) run --rm --no-deps develop bash -c \
	  "cd /app && \
	   PYTHONPATH=\$$(find /app -maxdepth 1 -mindepth 1 -type d | tr '\n' ':') \
	   python3 -m pytest $(if $(pkg),$(pkg)/test/,) -v $(OPTS)"

# --- ユーティリティ ---

rviz2:
	$(COMPOSE) up $(_up_flags) rviz2

rviz2-slam:
	$(COMPOSE) up $(_up_flags) rviz2-slam

rviz2-navigation:
	$(COMPOSE) up $(_up_flags) rviz2-navigation

foxglove-bridge:
	$(COMPOSE) up $(_up_flags) foxglove-bridge

diagnostics:
	$(COMPOSE) up $(_up_flags) diagnostics

system-manager:
	$(COMPOSE) up $(_up_flags) system-manager

web-ui:
	$(COMPOSE) up $(_up_flags) web-ui

ui-all:
	$(COMPOSE) up $(_up_flags) system-manager web-ui foxglove-bridge diagnostics

ui-dev-all:
	$(COMPOSE) up $(_up_flags) system-manager web-ui-dev foxglove-bridge diagnostics

web-ui-dev:
	$(COMPOSE) up web-ui-dev

# mg_ui の検査: フロントエンドの型チェックと lint (node コンテナ内で実行)
ui-lint:
	$(COMPOSE) run --rm --no-deps web-ui-dev sh -c \
	  "npm install --no-audit --no-fund && npx tsc -b && npm run lint"

# mg_ui のテスト: フロントエンド(vitest)と system_manager(pytest)
ui-test:
	$(COMPOSE) run --rm --no-deps web-ui-dev sh -c \
	  "npm install --no-audit --no-fund && npm test"
	$(MAKE) test pkg=mg_ui/mg_system_manager

xhost:
	xhost +local:docker

config:
	$(COMPOSE) config

