.PHONY: all build build-no-cache run offline reoptimize test shell down

COMPOSE := docker compose -f compose.yaml

all: build

build:
	$(COMPOSE) build dev

build-no-cache:
	$(COMPOSE) build --no-cache dev

run:
	$(COMPOSE) run --rm slam-gnss-2d

offline:
	$(if $(BAG),ROSBAG_FILE=$(BAG) )$(COMPOSE) run --rm offline-slam

reoptimize:
	$(if $(INPUT),INPUT_DIR=$(INPUT) )$(if $(BAG),ROSBAG_FILE=$(BAG) )$(if $(SAVE),SAVE_DIR=$(SAVE) )$(COMPOSE) run --rm reoptimize

test:
	$(COMPOSE) run --rm test

shell:
	$(COMPOSE) run --rm -it dev

down:
	$(COMPOSE) down
