# mg_system_manager

Web UI から、docker compose のサービスを操作するための API サーバ (FastAPI)。`python3 -m mg_system_manager` で、ポート 8001 で動く。compose の `system-manager` サービス (`make system-manager`) が起動する。
**認証はない。信頼できるネットワークでのみ使うこと。**

## 構成

```
mg_system_manager/
  __main__.py        uvicorn で起動 (0.0.0.0:8001)
  app.py             FastAPI の作成。CORS、ルーターの登録
  config.py          環境変数の設定 (Settings)、サービスの定義 (SERVICES)、入力の検証の正規表現
  dependencies.py    ルーターに渡す部品 (設定、ComposeRunner、設定の保存)
  docker_ops.py      ComposeRunner: docker compose の実行、コンテナの検索、サービスごとの排他
  log_hub.py         コンテナのログの配信 (サービスごとに docker logs を 1 本、接続ごとに上限付きのバッファ)
  settings_store.py  UI 設定の保存 (キー単位のマージ)
  scenario_results.py  シナリオの一覧と、シナリオテストの結果 (progress.json・result.json・ログ) の読み出し
  ros_cmd.py         コンテナ内で ROS 2 のコマンドを実行する
  responses.py       操作の結果を {success, message} にして、ログに残す
  routers/           エンドポイント
test/                pytest (fastapi.testclient。httpx が必要)
```

- `docker compose` は、`.env` の `USE_GPU` に応じて GPU 用の compose ファイルを重ねて実行する (Makefile と同じ)。
- コンテナは、compose のプロジェクト (`COMPOSE_PROJECT_NAME`、未設定ならディレクトリ名) で絞り込む。`docker compose run` で作られた one-off のコンテナも対象で、同じサービスに複数あるときは、動作中のものを優先する。
- 同じサービスへの操作は、同時に 1 つだけ。実行中に別の操作が来ると、`another operation is in progress` を返す。ハードウェアを使うサービス (`hardware=True`) と、シナリオテスト用のスタックの起動は、互いに排他する。
- すべての操作の応答は、`{"success": bool, "message": str}` の形。

## 操作できるサービス (`config.py` の `SERVICES`)

UI の System ページでの表示グループ (`layer`) は、Core、Function、Tool。`SERVICES` に足すと、System ページに自動で反映される。

| サービス | 表示名 | グループ | ハードウェア |
| :--- | :--- | :--- | :--- |
| `foxglove-bridge` | Foxglove Bridge | Core | |
| `diagnostics` | Diagnostics | Core | |
| `navigation` | Navigation | Function | ○ |
| `slam` | SLAM | Function | ○ |
| `slam-gnss-2d` | SLAM GNSS 2D | (なし) | ○ |
| `waypoint-editor` | Waypoint Editor | Tool | |
| `gazebo-simulation` | Gazebo Simulation | Tool | |
| `rviz2`、`rviz2-navigation`、`rviz2-slam` | RViz2 ほか | Tool | |
| `scenario-test`、`scenario-env`、`scenario-remote-stack` | シナリオテスト | (なし) | |
| `map-preview`、`reoptimize-slam` | 地図のプレビュー、再最適化 | (なし) | |

## エンドポイント

### サービスの操作 (`routers/services.py`)

| メソッドとパス | 内容 |
| :--- | :--- |
| `GET /status` | すべてのサービスのコンテナの状態 (1 秒のキャッシュ) |
| `GET /services` | サービスの一覧とグループ |
| `POST /<service>/start`・`stop`・`restart` | `SERVICES` の各サービスの起動・停止・再起動 |

### ログ (`routers/logs.py`)

| パス | 内容 |
| :--- | :--- |
| `WS /logs/stream` | コンテナのログの配信 |

### 地図 (`routers/maps.py`)

| メソッドとパス | 内容 |
| :--- | :--- |
| `GET /navigation/maps` | 環境変数 `MAP_PATH` の `map_list.txt` (1 行 1 ファイル。`MAP_PATH` からの相対パス。空行と `#` で始まる行は無視) から、地図の一覧を返す |
| `POST /map/common/save` | 地図を保存する (`map_dir`、`map_name`) |
| `POST /slam_gnss_2d/map/save` | `slam_gnss_2d` の地図の保存 (`save_slam_map` のサービスを呼ぶ) |
| `GET /slam_gnss_2d/maps` | 保存済みの `slam_gnss_2d` の地図の一覧 |
| `POST /slam_gnss_2d/preview/start`・`stop` | `map-preview` の起動・停止 |
| `POST /slam_gnss_2d/reoptimize/start`・`stop` | `reoptimize-slam` の起動・停止 |

- `MAP_PATH` は、コンテナの起動時の値。`.env` を変えたら、`system-manager` を起動し直す。`map_list.txt` がなければ `success: false` を返し、UI にそのまま表示する。
- 地図の切り替えは、sequencer の `~/load_map` を呼ぶ (読み込み済みの地図の記録を更新するため)。

### rosbag の再生 (`routers/rosbag.py`)

| メソッドとパス | 内容 |
| :--- | :--- |
| `GET /rosbag-replay/env` | `.env` の `ROSBAG_FILE` と `ROSBAG_TOPICS` |
| `POST /rosbag-replay/start`・`stop` | `rosbag-replay` の起動・停止 (`file`、`topics`) |

### シミュレーション (`routers/simulation.py`)

| メソッドとパス | 内容 |
| :--- | :--- |
| `POST /simulation/reset-pose` | Gazebo のロボットの位置を戻す (`ign service` の `set_pose`。`SIMULATION_WORLD`、`SIMULATION_ROBOT_NAME` を使う) |

### シナリオテスト (`routers/scenario_test.py`、`routers/scenario_stack.py`)

`scenario-test` と `scenario-env` は、`SERVICES` の汎用ルート (`/<service>/start` など) と重ならないよう、`/scenario/` の下に置いている。使い方は [mg_scenario_test の README](../../../mg_scenario_test/README.md#web-ui-から実行する)。

| メソッドとパス | 内容 |
| :--- | :--- |
| `GET /scenario/list` | シナリオの一覧 |
| `POST /scenario/run/start`・`stop` | 実行の開始・停止 |
| `GET /scenario/status` | 実行の状態 |
| `GET /scenario/runs`、`/scenario/runs/{run_id}` | 結果の一覧、1 回の実行の詳細 |
| `GET /scenario/runs/{run_id}/{entry}/result`・`log` | シナリオ 1 本の結果とログ |
| `POST /scenario/env/start`・`stop` | attach 用の環境 (`scenario-env`) の起動・停止 |
| `POST /scenario-stack/start`・`stop`、`GET /scenario-stack/logs` | 実機 PC 側のナビゲーションスタック (`scenario-remote-stack`) の起動・停止・ログ |

### 設定 (`routers/settings.py`)

| メソッドとパス | 内容 |
| :--- | :--- |
| `GET /settings` | UI の設定 |
| `PATCH /settings` | 指定したキーだけを更新する。値が `null` のキーは削除する |

設定の保存先は `UI_DATA_DIR` (既定 `/root/ros2_data/mg_ui_local`)。

## 設定 (環境変数)

compose の `system-manager` で設定している。

| 変数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `PROJECT_DIR` | `/app` | コンテナ内のプロジェクトのパス |
| `HOST_PROJECT_DIR`、`HOST_HOME` | | ホストのパス (docker compose に渡す) |
| `COMPOSE_PROJECT_NAME` | ディレクトリ名 | compose のプロジェクト名 |
| `SIMULATION_WORLD`、`SIMULATION_ROBOT_NAME` | `warehouse`、`mg` | `reset-pose` が使う Gazebo のワールドとロボット名 |
| `UI_DATA_DIR` | `/root/ros2_data/mg_ui_local` | UI の設定の保存先 |
| `SCENARIO_DIRS`、`SCENARIO_RESULTS_DIR`、`SCENARIO_PROFILE`、`SCENARIO_ROS_DOMAIN_ID` | 回帰・サンプルのシナリオ、`/root/ros2_data/scenario_results`、`mg01`、`42` | シナリオテスト |
| `SCENARIO_STACK_ALLOWED_PACKAGES` | `mg_bringup` | `scenario-remote-stack` で起動してよいパッケージ |
| `MAP_PATH` | | 地図の一覧 (`map_list.txt`) の場所 |
| `SYSTEM_MANAGER_ALLOW_ORIGINS` | | 許可するオリジンの追加 (カンマ区切り) |

## CORS

localhost、プライベート IP、Tailscale、`.local` のホストの `:8080` と `:5173` を許可する。他のオリジンは、`SYSTEM_MANAGER_ALLOW_ORIGINS` で足す。

## 機能を足す

- 操作の API: `routers/` に足す。入力は `config.py` の正規表現 (`PATH_RE`、`MAP_NAME_RE` など) で検証する。テストを `test/` に書く。
- 操作できる compose サービス: `config.py` の `SERVICES` に足す。
- 実行: `make test pkg=mg_ui/mg_system_manager` (`make ui-test` にも含まれる)。テストには `httpx` が必要で、`requirements.txt` に書いてある。既存の develop イメージに入っていないときは、`make build svc=develop` で再ビルドする。
