# コマンド一覧

`Makefile` のターゲットと、`compose.yaml` のサービスの対応表。
すべてリポジトリのルートで実行する。

`make` (引数なし) または `make help` でターゲット一覧、`make help-args` で共通の引数を表示する。
`Makefile` はエントリポイントで、ターゲットは分類ごとに `make/` 配下へ分けている。各ターゲットの引数は、そのターゲットの上のコメントに書いてある。

| ファイル | 内容 |
| :--- | :--- |
| `make/common.mk` | 共通の変数・関数 (`COMPOSE` の解決、`USE_GPU`、`.env` の読み取り) |
| `make/run.mk` | 実行 (実機・シミュレーション・再生・RViz2・Web UI) |
| `make/build.mk` | ビルド (Docker イメージ) |
| `make/container.mk` | コンテナ操作 (シェル・ログ・状態・停止) |
| `make/test.mk` | テスト (pytest・UI 検査・シナリオテスト) |
| `make/tools.mk` | ツール (rosbag の解析・可視化・評価) |
| `make/help.mk` | ヘルプ |

ターゲットを追加するときは、行末に `## 1 行説明 [主な引数]` を付けると `make help` に載る。カテゴリ見出しは `##@ 見出し`。

## 共通オプション

| オプション | 使えるターゲット | 内容 |
| :--- | :--- | :--- |
| `DETACH=1` | サービスを起動するターゲット (`slam` など) | バックグラウンドで起動する |
| `ATTACH=1` | `develop` | 起動後に bash でアタッチする |
| `OPTS="..."` | 起動系、`bag-*`、`test` | launch 引数 (`key:=value`) や、解析ツール・pytest のオプションを渡す |
| `svc=<サービス名>` | `build`、`build-no-cache`、`shell`、`logs`、`restart` | 対象の compose サービス |
| `USE_GPU=nvidia\|amd` | Gazebo 系 | `.env` の `USE_GPU` を一時的に上書きする |
| `TO_TOOLS=1` | `bag-*` | 出力を `tools/data/` に保存する。`OUT_DIR=<dir>`、`OUT=<file>` でも指定できる |

## 起動

実機と、シミュレーション・再生用のサービス。

| コマンド | compose サービス | 起動するもの |
| :--- | :--- | :--- |
| `make slam` | `slam` | `mg_bringup` の `bringup_slam_toolbox.launch.py` (slam_toolbox) |
| `make slam-gnss-2d` | `slam-gnss-2d` | `slam_gnss_2d` の `bringup_slam_gnss_2d.launch.py` (GNSS 拘束付き 2D SLAM)。SLAM ノードと RViz2 だけを起動し、センサのドライバは含まない |
| `make offline-slam-gnss-2d BAG=<bag>` | `offline-slam-gnss-2d` | `mg_bringup` の `offline_slam_gnss_2d.launch.py` (rosbag からの再処理。`BAG` は `ROSBAG_FILE` でも指定できる) |
| `make reoptimize` | `reoptimize-slam` | `slam_gnss_2d` の `reoptimize.launch.py`。`INPUT_DIR`・`SAVE_DIR`・`BAG_PATH` を渡せる |
| `make navigation` | `navigation` | `mg_bringup` の `bringup_navigation.launch.py` (自己位置推定 + Nav2 + ウェイポイントシーケンサ) |
| `make rosbag-replay` | `rosbag-replay` | `ros2 bag play $ROSBAG_FILE --clock`。`SIMULATION=true` で動く |
| `make obstacle-detection-replay` | `obstacle-detection-replay` | `mg_drivers` の `obstacle_detection_replay.launch.py` (rosbag-replay と別端末で実行) |
| `make gazebo-simulation` | `gazebo-simulation` | `mg_simulation` の `bringup.launch.py` (Gazebo Fortress) |
| (make ターゲットなし) | `map-preview` | `mg_slam` の `map_preview.launch.py` (`SLAM_MAP_DIR` の地図をプレビューする) |
| (make ターゲットなし) | `waypoint-editor` | `mg_waypoint_navigation` の `waypoint_editor.launch.py` |

- `slam`・`navigation`・`rosbag-replay` は `USE_RVIZ=false` で起動する。RViz2 は別途起動する。
- フォアグラウンドで起動するのが既定。`DETACH=1` を付けるとバックグラウンドで起動する。

## 可視化

| コマンド | compose サービス | 内容 |
| :--- | :--- | :--- |
| `make rviz2-slam` | `rviz2-slam` | `mg_slam/rviz/rviz.rviz` |
| (make ターゲットなし) | `rviz2-slam-gnss-2d` | `slam_gnss_2d/slam_gnss_2d/rviz/slam_gnss_2d.rviz` |
| `make rviz2-navigation` | `rviz2-navigation` | `mg_navigation/rviz/rviz.rviz` |
| `make rviz2` | `rviz2` | `.env` の `RVIZ_CONFIG` で指定した設定 (未設定なら RViz2 の既定) |
| `make xhost` | - | `xhost +local:docker` (GUI を使う前に実行する) |

## Web UI と管理 (mg_ui)

| コマンド | compose サービス | 内容 |
| :--- | :--- | :--- |
| `make foxglove-bridge` | `foxglove-bridge` | ROS と Web UI の通信 (8765) |
| `make web-ui` | `web-ui` | ビルド済みの Web UI を配信 (8080) |
| `make web-ui-dev` | `web-ui-dev` | Vite 開発サーバ (5173) |
| `make system-manager` | `system-manager` | 管理 API (8001) |
| `make diagnostics` | `diagnostics` | `mg_diagnostics` の診断ノード |
| `make ui-all` | `system-manager`・`web-ui`・`foxglove-bridge`・`diagnostics` | 本番用に一式を起動 |
| `make ui-dev-all` | `system-manager`・`web-ui-dev`・`foxglove-bridge`・`diagnostics` | 開発用に一式を起動 |
| `make ui-lint` | `web-ui-dev` | フロントエンドの型チェックと lint |
| `make ui-test` | `web-ui-dev` と `develop` | フロントエンド (vitest) と system_manager (pytest) のテスト |

詳細は [mg_ui の README](../mg_ui/README.md)。

## シナリオテスト

Gazebo とナビゲーションを起動して合否を判定する。詳細は [mg_scenario_test の README](../mg_scenario_test/README.md)。

| コマンド | 内容 |
| :--- | :--- |
| `make scenario-validate` | シナリオ YAML の静的検証 (シミュレータ不要) |
| `make scenario-test SCENARIO=<名前\|パス>` | スタックを起動して 1 本実行する。`GUI=1` で表示、`PROFILE=`、`ROBOT=<実機 PC の IP>` を指定できる |
| `make scenario-test-all [TIER=smoke]` | 回帰テストを 1 本ごとにスタックを起動し直して実行する。`TAGS`、`REPEAT`、`EXAMPLES=1`、`KNOWN=1`、`GUI=1` も指定できる |
| `make scenario-env [GUI=1] [PROFILE=] [WORLD=]` | attach 用にシミュレータとナビゲーションを起動したままにする (compose サービス `scenario-env`) |
| `make scenario-test-attach SCENARIO=<名前>` | `scenario-env` に接続して 1 本実行する |
| `make scenario-env-stop` | `scenario-env` を止める |
| (make ターゲットなし) | compose サービス `scenario-remote-stack`: 実機 PC 側のナビゲーションスタック。`system-manager` の API から起動される |

## 開発とテスト

| コマンド | 内容 |
| :--- | :--- |
| `make develop` | develop コンテナをバックグラウンドで起動する (`ATTACH=1` でアタッチ) |
| `make shell-develop` | develop コンテナの bash に入る (未起動なら起動する) |
| `make shell svc=<名前>` | 起動中のコンテナの bash に入る |
| `make logs svc=<名前> [TAIL=] [SINCE=]` | ログを追う ([docker.md](./docker.md#ログ)) |
| `make logs-all [TAIL=] [SINCE=]` | 全サービスのログをまとめて追う |
| `make logs-export [svc=<名前>] [OUT_DIR=]` | コンテナのログを `${HOME}/ros2_data/logs/<日時>/` に書き出す (`down` の前に実行する) |
| `make logs-clean-ros [DAYS=14]` | ROS のファイルログ (`${HOME}/ros2_data/ros_log`) のうち、古いものを削除する |
| `make ps` | 起動中のコンテナを表示する |
| `make restart svc=<名前>` | サービスを再起動する |
| `make down` | すべてのサービスを止める |
| `make config` | compose の設定を展開して確認する |
| `make test [pkg=<パッケージ>] [OPTS="-k ..."]` | develop コンテナで pytest を実行する |

- `make test` は Python のテストだけを実行する。C++ の gtest は `colcon test` で実行する ([mg_drivers の README](../mg_drivers/README.md) など)。
- Web UI のテストは `make ui-test`。

## ビルド

| コマンド | 内容 |
| :--- | :--- |
| `make build-robot` | 実機向け: `slam`・`develop`・`web-ui` のイメージ |
| `make build-robot-no-cache` (`build-real-no-cache`) | 同上、キャッシュ無効 |
| `make build-sim` (`build-all`) | 上記 + `gazebo-simulation` |
| `make build svc=<名前>` | 指定したサービスのイメージ |
| `make build-no-cache svc=<名前>` | 同上、キャッシュ無効 |

詳細は [docker.md](./docker.md)。

## rosbag の解析

対象の bag は、`BAG=<パス>` または `.env` の `ROSBAG_FILE` (`BAG_PATH` でも可) で指定する。出力は既定で bag と同じディレクトリに保存される。
各ツールの詳細は [tools の README](../tools/README.md)。

| コマンド | 内容 |
| :--- | :--- |
| `make bag-summary` | 通信の健全性、GNSS の Fix 率と精度、オドメトリの積算距離を `summary.md` に出力する |
| `make bag-plot-gnss [CIRCLES=1] [SCALE=10]` | GNSS の軌跡・Fix 状態・精度を画像にする |
| `make bag-plot-gnss-map` | `MAP_PATH/map_list.txt` の地図群に、`gnss_transform.yaml` で変換した GNSS を重ねる (`MAP_LIST=`、`GNSS_TRANSFORM=` で変更できる) |
| `make bag-plot-scans [NODES=1:20]` | LiDAR スキャンの点群を 2D 画像にする |
| `make bag-eval-slam SLAM_DIR=<dir>` | SLAM の出力を GNSS (RTK) と比較して評価する |
| `make bag-eval-localization [GT_DIR=] [MAP_GT_DIR=] [FAULTS=]` | 自己位置推定 (EKF 融合) を評価する |

## コンテナ内で ROS 2 のコマンドを使うとき

```bash
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
```
