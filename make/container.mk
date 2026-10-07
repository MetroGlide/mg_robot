##@ コンテナ操作 (シェル・ログ・状態)

.PHONY: shell shell-develop logs logs-all logs-export ps restart down config xhost

# 起動中のコンテナの bash に入る。
#   svc=<サービス名>  必須 (例: svc=slam)
shell: ## 起動中のコンテナの bash に入る [svc=<名前>]
	$(call require,svc,make shell svc=<service-name>)
	$(COMPOSE) exec $(svc) bash

shell-develop: ## develop コンテナの bash に入る (未起動なら起動)
	$(COMPOSE) up -d develop
	$(COMPOSE) exec develop bash

# サービスのログを追う (Docker の json-file ログ。ローテーション設定は doc/docker.md)。
#   svc=<サービス名>  必須
#   TAIL=<行数>       最初に表示する末尾の行数 (既定: 全行)
#   SINCE=<期間>      この期間以降だけ表示する (例: SINCE=10m)
logs: ## ログを追う [svc=<名前> TAIL SINCE]
	$(call require,svc,make logs svc=<service-name>)
	$(COMPOSE) logs -f $(if $(TAIL),--tail $(TAIL) )$(if $(SINCE),--since $(SINCE) )$(svc)

# 全サービスのログを時刻順に追う。
#   TAIL=<行数>  サービスごとの末尾の行数 (既定: 100)
#   SINCE=<期間> この期間以降だけ表示する
logs-all: ## 全サービスのログをまとめて追う [TAIL SINCE]
	$(COMPOSE) logs -f --tail $(or $(TAIL),100) $(if $(SINCE),--since $(SINCE))

# 現在のコンテナのログをファイルに書き出す。コンテナを削除する (down) とログも消えるため、その前に実行する。
# ${HOME}/ros2_data/logs/<日時>/<サービス>.log に保存する。
#   svc=<サービス名>  対象サービス (省略時はコンテナが存在する全サービス)
#   OUT_DIR=<dir>     保存先の親ディレクトリ (既定: ${HOME}/ros2_data/logs)
logs-export: ## コンテナのログをファイルに書き出す [svc OUT_DIR]
	@d="$(or $(OUT_DIR),$(HOME)/ros2_data/logs)/$$(date +%Y%m%d_%H%M%S)"; \
	mkdir -p "$$d"; \
	for s in $(or $(svc),$$($(COMPOSE) ps -a --format '{{.Service}}' | sort -u)); do \
	  $(COMPOSE) logs --no-color --timestamps "$$s" > "$$d/$$s.log" 2>&1 && echo "$$d/$$s.log"; \
	done

ps: ## コンテナの状態を表示
	$(COMPOSE) ps

# サービスを再起動する。
#   svc=<サービス名>  必須
restart: ## サービスを再起動 [svc=<名前>]
	$(call require,svc,make restart svc=<service-name>)
	$(COMPOSE) restart $(svc)

down: ## すべてのサービスを停止して削除
	$(COMPOSE) down

config: ## compose の設定を展開して確認
	$(COMPOSE) config

xhost: ## GUI 用に xhost +local:docker を実行 (RViz2・Gazebo の前に)
	xhost +local:docker
