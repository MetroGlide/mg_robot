# 共通の変数・関数。他の .mk から参照される。

# .env から KEY=VALUE の値を取り出す。使い方: $(call env_value,KEY)
env_value = $(shell grep -E '^$(1)=' .env 2>/dev/null | cut -d= -f2 | tr -d '[:space:]')

# GPU の種類 (none | nvidia | amd)。.env の USE_GPU を既定とし、
# コマンドラインの USE_GPU=nvidia|amd で一時的に上書きできる。
USE_GPU := $(call env_value,USE_GPU)
USE_GPU ?= none

COMPOSE_BASE := docker compose -f compose.yaml

ifeq ($(USE_GPU),nvidia)
  COMPOSE := $(COMPOSE_BASE) -f compose.gpu.nvidia.yaml
else ifeq ($(USE_GPU),amd)
  COMPOSE := $(COMPOSE_BASE) -f compose.gpu.amd.yaml
else
  COMPOSE := $(COMPOSE_BASE)
endif

# DETACH=1 でバックグラウンド起動する (サービスを起動するターゲット共通)
_up_flags = $(if $(DETACH),-d,)
# OPTS="..." を launch 引数 / ツールのオプションとしてコンテナへ渡す
_compose_opts = $(if $(OPTS),OPTS="$(OPTS)" )

# 引数が必須のターゲットで使う。使い方: $(call require,svc,make shell svc=<service-name>)
define require
$(if $($(1)),,$(error $(1) is required. Usage: $(2)))
endef
