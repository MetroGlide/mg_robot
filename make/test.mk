##@ テスト (単体テスト・UI 検査・シナリオテスト)

.PHONY: test ui-lint ui-test \
        scenario-test scenario-test-all scenario-test-attach scenario-validate scenario-env scenario-env-stop scenario-plot

# develop コンテナで pytest を実行する (Python のテストのみ。C++ の gtest は colcon test)。
#   pkg=<パッケージ>  特定パッケージのテストだけ実行する (例: pkg=mg_waypoint_navigation。
#                     複数のパッケージをまとめたディレクトリの中は、パスで指定する: pkg=localization_supervisor/localization_supervisor)
#   OPTS="..."        pytest のオプション (例: OPTS="-k test_bag")
# PYTHONPATH には、直下のディレクトリと、その中のパッケージ (package.xml のあるディレクトリ) を入れる。
test: ## pytest を実行 [pkg=<パッケージ> OPTS]
	$(COMPOSE) run --rm --no-deps develop bash -c \
	  "cd /app && \
	   PYTHONPATH=\$$(find /app -maxdepth 1 -mindepth 1 -type d | tr '\n' ':')\$$(find /app -mindepth 3 -maxdepth 3 -name package.xml -printf '%h:') \
	   python3 -m pytest $(if $(pkg),$(pkg)/test/,) -v $(OPTS)"

# mg_ui のフロントエンドの型チェックと lint (node コンテナ内で実行)。
ui-lint: ## Web UI の型チェックと lint
	$(COMPOSE) run --rm --no-deps web-ui-dev sh -c \
	  "npm install --no-audit --no-fund && npx tsc -b && npm run lint"

# mg_ui のテスト: フロントエンド (vitest) と system_manager (pytest)。
ui-test: ## Web UI (vitest) と system_manager (pytest) のテスト
	$(COMPOSE) run --rm --no-deps web-ui-dev sh -c \
	  "npm install --no-audit --no-fund && npm test"
	$(MAKE) test pkg=mg_ui/mg_system_manager

# --- シナリオテスト (詳細: mg_scenario_test/README.md) ---
# シナリオは名前 (mg_scenario_test/scenarios/<name>.yaml) またはパスで指定する。
# 終了コード: 0=PASSED, 1=FAILED, 2=ERROR。結果は ${ROS2_DATA_PATH}/scenario_results/<日時>/ に保存される。
_scenario_dir = /app/mg_scenario_test/scenarios
_scenario_dirs = --scenario-dir $(_scenario_dir)/regression --scenario-dir $(_scenario_dir)/examples
_scenario_results = --results-dir /root/ros2_data/scenario_results/$$(date +%Y%m%d_%H%M%S)
# ROBOT=<実機PCのIP> を指定すると、ナビゲーションスタックを実機PC (mg_system_manager :8001) に起動させ、
# シミュレータだけをこの PC で起動する。両方のPCの .env に MG_REMOTE_PEER・SCENARIO_ROS_DOMAIN_ID を設定しておく
_scenario_domain_id := $(call env_value,SCENARIO_ROS_DOMAIN_ID)
_remote_env = $(if $(ROBOT),-e ROS_DOMAIN_ID=$(or $(_scenario_domain_id),42) -e CYCLONEDDS_URI=file:///app/docker/cyclonedds/remote.xml -e MG_REMOTE_PEER=$(ROBOT),)
_remote_arg = $(if $(ROBOT),--remote-stack http://$(ROBOT):8001)
# LOCALIZATION_PRESET=<名前> / LOCALIZATION_PROFILE=<名前> を指定すると、.env の値に代えて、自己位置推定のプリセットを
# 重ねた / 自己位置監視のプロファイルを重ねたスタックで実行する (ROBOT 指定時は実機PCの .env が使われる)
# LOCALIZATION_GATE=<true|false|observe> は、自己位置監視ノードの中で AMCL 検証ゲートを使うか
# LOCALIZATION_CONFIG=<名前> は、自己位置推定の構成 (監視・自己位置監視の並走・ AMCL 検証ゲート・プリセット・プロファイルの組) を選ぶ。
# 中身は mg_navigation/mg_navigation/localization_config.py (launch の既定は supervised)。個別の変数を書けば、そちらが
# 優先する (プリセット・プロファイルを外すときは none)。比べるのは legacy (v2.1) と supervised (最新)
#   (mg_navigation/doc/localization_failure_ledger.md の「比べる構成」)
# 構成を選んだときは、.env の個別の値を混ぜないよう、書いていない個別の変数を空 (構成に従う) で渡す
_loc_var = $(if $($(1)),-e $(1)=$($(1)),$(if $(LOCALIZATION_CONFIG),-e $(1)=,))
_preset_env = $(if $(LOCALIZATION_CONFIG),-e LOCALIZATION_CONFIG=$(LOCALIZATION_CONFIG),) \
              $(call _loc_var,LOCALIZATION_MONITOR) $(call _loc_var,LOCALIZATION_PRESET) \
              $(call _loc_var,LOCALIZATION_PROFILE) $(call _loc_var,LOCALIZATION_GATE)
_scenario_run = $(COMPOSE) run --rm $(_remote_env) $(_preset_env) -e SCENARIO_ARGS

# シミュレータ・ナビゲーションごと起動して 1 本実行する (既定はヘッドレス)。
#   SCENARIO=<名前|パス>  必須。regression/・examples/ 配下のシナリオ名 (拡張子なし)、またはコンテナ内のパス
#   GUI=1                 シミュレータの GUI を表示する
#   PROFILE=<名前>        プロファイル (例: mg01)
#   ROBOT=<実機PCのIP>    ナビゲーションスタックを実機PCで実行する
scenario-test: ## シナリオを 1 本実行 [SCENARIO=<名前> GUI PROFILE ROBOT LOCALIZATION_CONFIG LOCALIZATION_MONITOR LOCALIZATION_PRESET LOCALIZATION_PROFILE LOCALIZATION_GATE]
	$(_scenario_run)="run $(SCENARIO) $(_scenario_dirs) $(_scenario_results) $(_remote_arg) $(if $(GUI),--gui) $(if $(PROFILE),--profile $(PROFILE))" scenario-test

# 回帰テスト (scenarios/regression) を、1 本ごとにスタックを起動し直して実行する。
#   TIER=smoke|full  smoke は smoke タグのみ (変更ごとの確認用)。省略または full は全件
#   TAGS=a,b         タグで絞り込む
#   REPEAT=N         繰り返し回数
#   EXAMPLES=1       examples/ も含める
#   KNOWN=1          known_issue タグ (既知の問題で失敗するシナリオ) も含める (既定は除外)
#   GUI=1 / PROFILE=<名前> / ROBOT=<実機PCのIP>  scenario-test と同じ
#   SHARDS=N         N 本に分けて並列に実行する。結果は同じディレクトリにまとめ、進み具合と junit は分け札ごと
#                    (progress_<I>of<N>.json)。ROS_DOMAIN_ID と Gazebo の区画を分けて隔離する。ROBOT・GUI とは使えない。
#                    始める前に資源を確かめ、足りなければ始めない (1 本あたり CPU 5 コア・メモリ 2 GB を見込み、
#                    今の負荷の平均と合わせて CPU の 7 割、空きのメモリから余裕 4 GB を残す。負荷で時間がずれると
#                    結果が変わるため)。並列の計測の間は、ビルドや重いテストを同時に流さない
_run_all_args = run-all $(_scenario_dir)/regression $(if $(EXAMPLES),$(_scenario_dir)/examples) $(_remote_arg) $(if $(KNOWN),,--exclude-tags known_issue) $(if $(filter smoke,$(TIER)),--tags smoke) $(if $(TAGS),--tags $(TAGS)) $(if $(REPEAT),--repeat $(REPEAT)) $(if $(GUI),--gui) $(if $(PROFILE),--profile $(PROFILE))
scenario-test-all: ## 回帰テストを一括実行 [TIER TAGS REPEAT EXAMPLES KNOWN GUI PROFILE ROBOT SHARDS LOCALIZATION_CONFIG LOCALIZATION_MONITOR LOCALIZATION_PRESET LOCALIZATION_PROFILE LOCALIZATION_GATE]
ifeq ($(SHARDS),)
	$(_scenario_run)="$(_run_all_args) $(_scenario_results)" scenario-test
else
	$(if $(ROBOT)$(GUI),$(error SHARDS は ROBOT・GUI と使えません))
	@cores=$$(nproc); load=$$(cut -d' ' -f1 /proc/loadavg); \
	avail=$$(awk '/MemAvailable/ {print int($$2 / 1048576)}' /proc/meminfo); \
	if awk -v l=$$load -v n=$(SHARDS) -v c=$$cores 'BEGIN {exit !(l + 5 * n > 0.7 * c)}' \
	   || [ $$avail -lt $$((2 * $(SHARDS) + 4)) ]; then \
	  echo "資源が足りないので始めません: 負荷の平均 $$load + 見込み $$((5 * $(SHARDS))) コア (CPU $$cores の 7 割まで)、空きのメモリ $$avail GB (必要 $$((2 * $(SHARDS) + 4)) GB)。SHARDS を減らすか、ほかの処理の終わりを待ってください" >&2; \
	  exit 1; \
	fi
	@ts=$$(date +%Y%m%d_%H%M%S); \
	for i in $$(seq 1 $(SHARDS)); do \
	  $(_scenario_run)="$(_run_all_args) --results-dir /root/ros2_data/scenario_results/$$ts --shard $$i/$(SHARDS)" \
	    -e ROS_DOMAIN_ID=$$((60 + i)) -e IGN_PARTITION=scenario_shard$$i -e GZ_PARTITION=scenario_shard$$i \
	    scenario-test 2>&1 | sed -u "s/^/[$$i] /" & \
	done; \
	wait; \
	echo "results: /root/ros2_data/scenario_results/$$ts"
endif

# make scenario-env の起動が前提。シミュレータを起動し直さずに繰り返し実行する。
#   SCENARIO=<名前|パス>  必須 / PROFILE=<名前>
scenario-test-attach: ## 起動済みの scenario-env に接続して 1 本実行 [SCENARIO=<名前> PROFILE]
	$(_scenario_run)="run $(SCENARIO) $(_scenario_dirs) $(_scenario_results) --attach $(if $(PROFILE),--profile $(PROFILE))" scenario-test

# attach モード用に、プロファイルのシミュレータとナビゲーションスタックを起動したままにする。
#   GUI=1 / PROFILE=<名前> (既定 mg01) / WORLD=<名前> (既定 warehouse)
scenario-env: ## attach 用にシミュレータ+ナビゲーションを起動したままにする [GUI PROFILE WORLD]
	SCENARIO_ENV_ARGS="profile:=$(or $(PROFILE),mg01) world:=$(or $(WORLD),warehouse) headless:=$(if $(GUI),false,true)" $(COMPOSE) up -d --force-recreate scenario-env

scenario-env-stop: ## scenario-env を停止
	$(COMPOSE) stop scenario-env

# シミュレータ不要。同梱シナリオの YAML を検証する。
scenario-validate: ## シナリオ YAML を静的検証 (シミュレータ不要)
	$(_scenario_run)="validate $(_scenario_dir)" --no-deps scenario-test

# シナリオの結果を図にする (tools/scripts/plot_scenario.py)。パスは develop コンテナ内 (ホストの ~/ros2_data は /root/ros2_data)。
#   RESULT=<dir>                  1 回分の結果、または日時のディレクトリ。各結果に scenario_plot.png を出す (記録 bag/ が要る)
#   COMPARE="<dir>=<名前> ..."     構成ごとの日時のディレクトリを、シナリオごとに並べる (最初の dir に compare.png)
#   例: make scenario-plot RESULT=/root/ros2_data/scenario_results/20261115_120000
#       make scenario-plot COMPARE="/root/ros2_data/scenario_results/A=legacy /root/ros2_data/scenario_results/B=supervised"
scenario-plot: ## シナリオの結果を図にする [RESULT=<dir> | COMPARE="<dir>=<名前> ..."]
	$(COMPOSE) run --rm --no-deps develop python3 /app/tools/scripts/plot_scenario.py \
	  $(if $(COMPARE),compare $(COMPARE),run $(RESULT))
