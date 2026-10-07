##@ 実行 (実機・シミュレーション・再生・可視化)
# 共通の引数:
#   DETACH=1       バックグラウンドで起動する (既定はフォアグラウンド)
#   OPTS="k:=v"    launch 引数を追加する
#   USE_GPU=nvidia|amd  Gazebo 系で .env の USE_GPU を一時的に上書きする

.PHONY: slam navigation slam-gnss-2d offline-slam-gnss-2d reoptimize rosbag-replay \
        obstacle-detection-replay gazebo-simulation develop \
        rviz2 rviz2-slam rviz2-navigation \
        foxglove-bridge diagnostics system-manager web-ui web-ui-dev ui-all ui-dev-all

# slam_toolbox の SLAM を起動する。
#   DETACH=1 / OPTS="..."
slam: ## slam_toolbox の SLAM を起動 [DETACH OPTS]
	$(_compose_opts)$(COMPOSE) up $(_up_flags) slam

# 自己位置推定 + Nav2 + ウェイポイントシーケンサを起動する。
#   DETACH=1 / OPTS="..."
navigation: ## ナビゲーションスタックを起動 [DETACH OPTS]
	$(_compose_opts)$(COMPOSE) up $(_up_flags) navigation

# GNSS 拘束付き 2D SLAM (SLAM ノードと RViz2 のみ。センサドライバは含まない)。
#   DETACH=1 / OPTS="..."
slam-gnss-2d: ## GNSS 拘束付き 2D SLAM を起動 [DETACH OPTS]
	$(_compose_opts)$(COMPOSE) up $(_up_flags) slam-gnss-2d

# rosbag から GNSS 拘束付き 2D SLAM をオフラインで再処理する。
#   BAG=<bag> (または ROSBAG_FILE=<bag>)  対象の rosbag。省略時は .env の ROSBAG_FILE
#   DETACH=1 / OPTS="..."
offline-slam-gnss-2d: ## rosbag からオフライン SLAM [BAG DETACH OPTS]
	$(if $(BAG),ROSBAG_FILE=$(BAG) )$(if $(ROSBAG_FILE),ROSBAG_FILE=$(ROSBAG_FILE) )$(_compose_opts)$(COMPOSE) up $(_up_flags) offline-slam-gnss-2d

# 保存済みの SLAM 出力を再最適化する。.env に各変数があれば引数なしで実行できる。
#   INPUT_DIR=<dir>  入力する SLAM 出力 (例: /app/maps/latest)
#   SAVE_DIR=<dir>   保存先 (例: /app/maps/latest_opt)
#   BAG_PATH=<bag>   使う rosbag (例: /app/bags/my_bag)
#   OPTS="..."
reoptimize: ## SLAM 出力を再最適化 [INPUT_DIR SAVE_DIR BAG_PATH OPTS]
	$(if $(INPUT_DIR),INPUT_DIR=$(INPUT_DIR) )$(if $(SAVE_DIR),SAVE_DIR=$(SAVE_DIR) )$(if $(BAG_PATH),BAG_PATH=$(BAG_PATH) )$(_compose_opts)$(COMPOSE) run --rm reoptimize-slam

# .env の ROSBAG_FILE を --clock 付きで再生する (SIMULATION=true で動く)。
#   OPTS="..."  ros2 bag play への追加オプション (例: OPTS="-r 0.5")
rosbag-replay: ## rosbag を再生 [OPTS]
	$(_compose_opts)$(COMPOSE) run --rm -it rosbag-replay

# rosbag-replay と別端末で起動する。点群復元 + 障害物検出 + RViz。
#   OPTS="rviz:=false use_color:=true"
obstacle-detection-replay: ## 点群復元 + 障害物検出 (rosbag-replay と別端末) [OPTS]
	$(_compose_opts)$(COMPOSE) run --rm -it obstacle-detection-replay

# Gazebo Fortress のシミュレータを起動する。
#   DETACH=1 / OPTS="..." / USE_GPU=nvidia|amd
gazebo-simulation: ## Gazebo シミュレータを起動 [DETACH OPTS USE_GPU]
	$(_compose_opts)$(COMPOSE) up $(_up_flags) gazebo-simulation

# develop コンテナをバックグラウンドで起動する。
#   ATTACH=1  起動後に bash でアタッチする
develop: ## develop コンテナを起動 [ATTACH]
	$(COMPOSE) up -d develop
	$(if $(ATTACH),$(COMPOSE) exec develop bash,)

# --- RViz2 (GUI を使う前にホストで make xhost を実行する) ---

rviz2: ## RViz2 を起動 (.env の RVIZ_CONFIG を使用) [DETACH]
	$(COMPOSE) up $(_up_flags) rviz2

rviz2-slam: ## RViz2 (slam_toolbox 用の設定) [DETACH]
	$(COMPOSE) up $(_up_flags) rviz2-slam

rviz2-navigation: ## RViz2 (ナビゲーション用の設定) [DETACH]
	$(COMPOSE) up $(_up_flags) rviz2-navigation

# --- Web UI と管理 (mg_ui) ---

foxglove-bridge: ## foxglove_bridge を起動 (:8765) [DETACH]
	$(COMPOSE) up $(_up_flags) foxglove-bridge

diagnostics: ## 診断ノードを起動 [DETACH]
	$(COMPOSE) up $(_up_flags) diagnostics

system-manager: ## system_manager (管理 API :8001) を起動 [DETACH]
	$(COMPOSE) up $(_up_flags) system-manager

web-ui: ## ビルド済みの Web UI を配信 (:8080) [DETACH]
	$(COMPOSE) up $(_up_flags) web-ui

# Vite 開発サーバ (:5173)。常にフォアグラウンドで起動する。
web-ui-dev: ## Web UI の開発サーバを起動 (:5173)
	$(COMPOSE) up web-ui-dev

ui-all: ## 本番用に system-manager・web-ui・foxglove-bridge・diagnostics を一式起動 [DETACH]
	$(COMPOSE) up $(_up_flags) system-manager web-ui foxglove-bridge diagnostics

ui-dev-all: ## 開発用に system-manager・web-ui-dev・foxglove-bridge・diagnostics を一式起動 [DETACH]
	$(COMPOSE) up $(_up_flags) system-manager web-ui-dev foxglove-bridge diagnostics
