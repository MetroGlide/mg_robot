# 環境変数とデータディレクトリ

## `.env`

```bash
cp .env.example .env
```

`.env` は、全コンテナの `env_file` として読み込まれる。`USE_GPU` と `SCENARIO_ROS_DOMAIN_ID` は、Makefile も `.env` から直接読む。
変数の値と既定値は [`.env.example`](../.env.example) が正で、ここには意味だけを書く。

### 共通

| 変数 | 内容 |
| :--- | :--- |
| `COMPOSE_PROJECT_NAME` | compose のプロジェクト名 (コンテナ名の接頭辞) |
| `SIMULATION` | `true` でシミュレーション用の設定 (センサのドライバを起動せず、`use_sim_time` を使う)。launch の `simulation` 引数の既定値 |
| `USE_RVIZ` | RViz2 を起動するか。launch の `rviz` 引数の既定値。`slam`・`navigation`・`rosbag-replay` は `false` に固定される |
| `RVIZ_CONFIG` | `make rviz2` が使う RViz2 の設定ファイル (コンテナ内のパス) |
| `USE_GPU` | `none` / `nvidia` / `amd` ([docker.md](./docker.md)) |
| `RMW_IMPLEMENTATION` | RMW 実装 |
| `ROS_DOMAIN_ID` | ROS 2 のドメイン ID |
| `ROS_LOCALHOST_ONLY` | `1` で ROS 2 の通信をローカルに限る |

### ログ

Docker のログのローテーション設定。compose の `${LOG_MAX_SIZE:-10m}` などで参照される。詳細は [docker.md](./docker.md#ログ)。

| 変数 | 内容 |
| :--- | :--- |
| `LOG_MAX_SIZE` | ログ 1 ファイルの上限 |
| `LOG_MAX_FILE` | 残す世代数 |

### データのパス

パスはすべて**コンテナ内**のもの。ホストの `${HOME}/ros2_data` が `/root/ros2_data` にマウントされる。

| 変数 | 内容 |
| :--- | :--- |
| `ROS2_DATA_PATH` | データのルート |
| `MAP_PATH` | 地図・`gnss_transform.yaml`・ウェイポイントを置くディレクトリ |
| `WAYPOINT_PATH` | ウェイポイントのファイル。`waypoint_sequencer` と `waypoint-editor` が読む |
| `LOCALIZATION_MAP_PATH`、`PLANNING_MAP_PATH` | 測位用・計画用の地図。現在の `bringup_navigation.launch.py` は使っていない ([地図の指定](#地図の指定)) |
| `ROSBAG_PATH` | rosbag の保存先。`record_bag.launch.py` が使う (必須) |
| `ROSBAG_FILE` | 再生・解析する bag。`rosbag-replay`、`offline-slam-gnss-2d`、`bag-*` が使う |
| `ROSBAG_TOPICS` | `rosbag-replay` で再生するトピック (空白区切り)。未設定なら全部。録画時の `/diagnostics`・`/diagnostics_agg`・`/events`・`/rosout` は、`/replay/` 以下に出す (いまのスタックの診断・イベントに混ざらないように) |

### SLAM の再最適化

| 変数 | 内容 |
| :--- | :--- |
| `INPUT_DIR` | 再最適化する SLAM の出力ディレクトリ |
| `SAVE_DIR` | 結果の保存先 |
| `BAG_PATH` | 再最適化に使う bag |

`make reoptimize` は、これらを引数 (`make reoptimize INPUT_DIR=... SAVE_DIR=...`) でも渡せる。

### 開発 PC と実機 PC をまたぐシナリオテスト

両方の PC の `.env` に設定する。詳細は [mg_scenario_test の README](../mg_scenario_test/README.md)。

| 変数 | 内容 |
| :--- | :--- |
| `SCENARIO_ROS_DOMAIN_ID` | シナリオテスト用の ROS ドメイン。通常の `ROS_DOMAIN_ID` と分け、実機のドライバに `/cmd_vel` が届かないようにする |
| `MG_REMOTE_PEER` | 相手 PC の IP (開発 PC には実機 PC の IP、実機 PC には開発 PC の IP) |
| `MG_DDS_INTERFACE` | 使うネットワークインターフェース (IP またはインターフェース名) |

### `.env.example` に無い変数

| 変数 | 使う場所 | 内容 |
| :--- | :--- | :--- |
| `SLAM_MAP_DIR` | `map-preview` | プレビューする SLAM の出力ディレクトリ。`system-manager` が設定して起動する |
| `ROS_LOG_DIR`、`RCUTILS_CONSOLE_OUTPUT_FORMAT` | 全サービス | compose の `x-common-env` で設定済み。ROS のファイルログを `/root/ros2_data/ros_log` に残し、コンソールログに時刻を付ける ([docker.md](./docker.md#共通のコンテナ設定)) |
| `PUBLISH_GAZEBO_TF` | `mg_simulation` の launch | `true` で Gazebo の TF を ROS に出す |
| `UI_DATA_DIR`、`SCENARIO_RESULTS_DIR`、`SYSTEM_MANAGER_ALLOW_ORIGINS` ほか | `mg_system_manager` | compose の `system-manager` で設定済み ([mg_ui の README](../mg_ui/README.md)) |

## 地図の指定

`make navigation` が読む地図は、`mg_bringup/launch/navigation/bringup_navigation.launch.py` の引数で決まる。

| 引数 | 内容 |
| :--- | :--- |
| `map_path` | 測位用の地図 |
| `planning_map_path` | 計画用の地図 |
| `waypoints_load_path` | ウェイポイントのファイル (`WAYPOINT_PATH` を使う) |
| `gnss_transform_file` | GNSS と地図の変換 (`MAP_PATH` の `gnss_transform.yaml` を使う) |

引数の既定の地図は launch ファイルが正。

- `LOCALIZATION_MAP_PATH` と `PLANNING_MAP_PATH` を読む行は、コメントアウトされている。別の地図を使うときは、`OPTS` で引数を渡す。

  ```bash
  make navigation OPTS="map_path:=/root/ros2_data/map/foo.yaml planning_map_path:=/root/ros2_data/map/foo_plan.yaml"
  ```

- 地図は走行中に Web UI から切り替えることもできる ([mg_waypoint_navigation の README](../mg_waypoint_navigation/README.md))。

## データディレクトリの構成

`${HOME}/ros2_data` (コンテナ内では `/root/ros2_data`) の下に、次のものが作られる。

| パス | 内容 | 作るもの |
| :--- | :--- | :--- |
| `map/` | 地図 (`*.yaml`・`*.pgm`)、`gnss_transform.yaml`、`waypoint.yaml`、`map_list.txt` (地図の一覧。`waypoint-editor` と `bag-plot-gnss-map` が読む) | SLAM の保存、ウェイポイントの編集 |
| `map/latest`、`map/latest_opt` | SLAM の出力 (再最適化の入力・結果の既定) | SLAM、`make reoptimize` |
| `rosbag/` | 収録した bag と、`summary.md` などの解析結果 | `record_bag.launch.py`、`tools/scripts/record.sh`、`make bag-*` |
| `scenario_results/<日時>/` | シナリオテストの結果 | `make scenario-test*` |
| `ros_log/` | ROS のファイルログ (`ROS_LOG_DIR`)。古いものは `make logs-clean` で削除 | 全サービス |
| `logs/<サービス>/<日付>.log` | 全サービスのコンテナのログ (常時書き出し) | `log-collector` (`make log-collector`) |
| `logs/<日時>/<サービス>.log` | `make logs-export` で書き出した Docker のログ | `make logs-export` |
| `mg_ui_local/` | `system-manager` の設定の保存先 (`UI_DATA_DIR`) | `system-manager` |

`tools/data/` (`TO_TOOLS=1` の出力先) は、リポジトリ内にあり、Git の追跡から外れている。
