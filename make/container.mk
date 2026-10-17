##@ コンテナ操作 (シェル・ログ・状態)

.PHONY: shell shell-develop logs logs-all logs-export logs-clean log-collector _ensure-log-collector \
        ps restart down config xhost

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

# 全サービスのコンテナのログを、ホストの ${HOME}/ros2_data/logs/<サービス名>/<日付>.log に常時書き出す
# log-collector を起動する (doc/monitoring.md)。イメージが無ければ取得する (ネットワークが要る。make build-robot でも取得する)。
# 一度起動すれば、ホストの再起動後も残る。make slam / navigation / slam-gnss-2d も、
# イメージがあれば起動する。make down で止まる。
log-collector: ## ログの常時書き出し (log-collector) を起動
	$(COMPOSE) up -d log-collector

# スタックの起動前に log-collector を起動する (イメージが無ければ、案内を表示して続ける)。
_ensure-log-collector:
	@if docker image inspect timberio/vector:0.43.1-alpine >/dev/null 2>&1; then \
	  $(COMPOSE) up -d log-collector; \
	else \
	  echo '[log-collector] イメージが無いので、ログをファイルに保存しません。ネットワークのある環境で make build-robot か make log-collector を一度実行してください'; \
	fi

# 古いログを削除する。削除したパスを表示する。
#   - ROS のファイルログ (ROS_LOG_DIR=/root/ros2_data/ros_log。ホストの ${HOME}/ros2_data/ros_log)
#   - 書き出したログ (${HOME}/ros2_data/logs/<サービス名>/<日付>.log と、make logs-export の <日時>/<サービス>.log)
#   DAYS=<日数>  この日数より古いものを削除する (既定: 14)
logs-clean: ## 古いログ (ROS のファイルログ・書き出したログ) を削除 [DAYS=14]
	$(COMPOSE) run --rm --no-deps develop bash -c \
	  "find /root/ros2_data/ros_log -mindepth 1 -maxdepth 1 -mtime +$(or $(DAYS),14) -print -exec rm -rf {} + ; \
	   find /root/ros2_data/logs -mindepth 2 -maxdepth 2 -type f -name '*.log' -mtime +$(or $(DAYS),14) -print -delete"


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
