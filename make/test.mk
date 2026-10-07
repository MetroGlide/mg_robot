##@ テスト (単体テスト・UI 検査・シナリオテスト)

.PHONY: test ui-lint ui-test \
        scenario-test scenario-test-all scenario-test-attach scenario-validate scenario-env scenario-env-stop

# develop コンテナで pytest を実行する (Python のテストのみ。C++ の gtest は colcon test)。
#   pkg=<パッケージ>  特定パッケージのテストだけ実行する (例: pkg=mg_waypoint_navigation)
#   OPTS="..."        pytest のオプション (例: OPTS="-k test_bag")
test: ## pytest を実行 [pkg=<パッケージ> OPTS]
	$(COMPOSE) run --rm --no-deps develop bash -c \
	  "cd /app && \
	   PYTHONPATH=\$$(find /app -maxdepth 1 -mindepth 1 -type d | tr '\n' ':') \
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
_scenario_run = $(COMPOSE) run --rm $(_remote_env) -e SCENARIO_ARGS

# シミュレータ・ナビゲーションごと起動して 1 本実行する (既定はヘッドレス)。
#   SCENARIO=<名前|パス>  必須。regression/・examples/ 配下のシナリオ名 (拡張子なし)、またはコンテナ内のパス
#   GUI=1                 シミュレータの GUI を表示する
#   PROFILE=<名前>        プロファイル (例: mg01)
#   ROBOT=<実機PCのIP>    ナビゲーションスタックを実機PCで実行する
scenario-test: ## シナリオを 1 本実行 [SCENARIO=<名前> GUI PROFILE ROBOT]
	$(_scenario_run)="run $(SCENARIO) $(_scenario_dirs) $(_scenario_results) $(_remote_arg) $(if $(GUI),--gui) $(if $(PROFILE),--profile $(PROFILE))" scenario-test

# 回帰テスト (scenarios/regression) を、1 本ごとにスタックを起動し直して実行する。
#   TIER=smoke|full  smoke は smoke タグのみ (変更ごとの確認用)。省略または full は全件
#   TAGS=a,b         タグで絞り込む
#   REPEAT=N         繰り返し回数
#   EXAMPLES=1       examples/ も含める
#   KNOWN=1          known_issue タグ (既知の問題で失敗するシナリオ) も含める (既定は除外)
#   GUI=1 / PROFILE=<名前> / ROBOT=<実機PCのIP>  scenario-test と同じ
scenario-test-all: ## 回帰テストを一括実行 [TIER TAGS REPEAT EXAMPLES KNOWN GUI PROFILE ROBOT]
	$(_scenario_run)="run-all $(_scenario_dir)/regression $(if $(EXAMPLES),$(_scenario_dir)/examples) $(_scenario_results) $(_remote_arg) $(if $(KNOWN),,--exclude-tags known_issue) $(if $(filter smoke,$(TIER)),--tags smoke) $(if $(TAGS),--tags $(TAGS)) $(if $(REPEAT),--repeat $(REPEAT)) $(if $(GUI),--gui) $(if $(PROFILE),--profile $(PROFILE))" scenario-test

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
