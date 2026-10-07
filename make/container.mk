##@ コンテナ操作 (シェル・ログ・状態)

.PHONY: shell shell-develop logs ps restart down config xhost

# 起動中のコンテナの bash に入る。
#   svc=<サービス名>  必須 (例: svc=slam)
shell: ## 起動中のコンテナの bash に入る [svc=<名前>]
	$(call require,svc,make shell svc=<service-name>)
	$(COMPOSE) exec $(svc) bash

shell-develop: ## develop コンテナの bash に入る (未起動なら起動)
	$(COMPOSE) up -d develop
	$(COMPOSE) exec develop bash

# サービスのログを追う。
#   svc=<サービス名>  必須
logs: ## ログを追う [svc=<名前>]
	$(call require,svc,make logs svc=<service-name>)
	$(COMPOSE) logs -f $(svc)

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
