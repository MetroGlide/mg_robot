##@ ビルド (Docker イメージ)
# ビルドは collect_deps.sh (docker/deps/ の準備) を先に実行する。詳細: doc/docker.md

.PHONY: _collect-deps build build-no-cache build-robot build-real build-robot-no-cache \
        build-real-no-cache build-sim build-all

_collect-deps:
	bash docker/collect_deps.sh

# 指定したサービスのイメージをビルドする。
#   svc=<サービス名>  必須 (例: svc=slam)
build: _collect-deps ## 指定サービスのイメージをビルド [svc=<名前>]
	$(call require,svc,make build svc=<service-name>)
	$(COMPOSE_BASE) build $(svc)

# キャッシュを使わずに指定サービスのイメージをビルドする。
#   svc=<サービス名>  必須
build-no-cache: _collect-deps ## キャッシュ無効で指定サービスをビルド [svc=<名前>]
	$(call require,svc,make build-no-cache svc=<service-name>)
	$(COMPOSE_BASE) build --no-cache $(svc)

# 実機向け一括ビルド (Gazebo シミュレータを除外: runtime, develop, web-ui のみ)
build-robot: _collect-deps ## 実機向けに一括ビルド (Gazebo を除外)
	$(COMPOSE_BASE) build slam develop web-ui

build-real: build-robot ## build-robot の別名

# 実機向けキャッシュ無効ビルド
build-robot-no-cache: _collect-deps ## 実機向けに一括ビルド (キャッシュ無効)
	$(COMPOSE_BASE) build --no-cache slam develop web-ui

build-real-no-cache: build-robot-no-cache ## build-robot-no-cache の別名

# シミュレータ含む一括ビルド
build-sim: _collect-deps ## シミュレータ込みで一括ビルド
	$(COMPOSE_BASE) build slam develop gazebo-simulation web-ui

build-all: build-sim ## build-sim の別名
