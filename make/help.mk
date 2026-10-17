##@ ヘルプ

.PHONY: help help-args

# "##@ 見出し" でカテゴリを、"ターゲット: ## 説明" で 1 行説明を書くと一覧に出る。
help: ## ターゲット一覧を表示 (引数なしの make も同じ)
	@awk 'BEGIN {FS = ":.*## "; printf "使い方: make <ターゲット> [引数=値 ...]\n"} \
	  /^##@/ {printf "\n\033[1m%s\033[0m\n", substr($$0, 5)} \
	  /^[a-zA-Z0-9_-]+:.*## / {printf "  \033[36m%-28s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@printf '\n共通の引数は make help-args、各ターゲットの引数の詳細は make/*.mk のコメントを参照。\n'

help-args: ## 共通の引数の一覧を表示
	@printf '%s\n' \
	  'DETACH=1          バックグラウンドで起動する (サービスを起動するターゲット)' \
	  'ATTACH=1          起動後に bash でアタッチする (develop)' \
	  'OPTS="..."        launch 引数 (key:=value)、解析ツール・pytest のオプションを渡す' \
	  'svc=<名前>        対象の compose サービス (build / shell / logs / restart など)' \
	  'USE_GPU=nvidia|amd  .env の USE_GPU を一時的に上書きする (Gazebo 系)' \
	  'BAG=<bag>         解析対象の rosbag (bag-* ターゲット。省略時は .env の ROSBAG_FILE)' \
	  'TO_TOOLS=1        解析結果を tools/data/ に出力する (OUT_DIR=<dir>、OUT=<file> でも指定可)' \
	  '' \
	  '環境変数の一覧: doc/environment.md / ターゲットと compose サービスの対応: doc/commands.md'
