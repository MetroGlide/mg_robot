# MG-01 (MetroGlide)

チーム MetroGlide の自律移動台車ロボット MG-01 のソフトウェア。ROS 2 (Humble) を Docker 上で動かす。
GNSS を使った地図作成 (SLAM)、地図上での自己位置推定、ウェイポイントに沿った自律走行、Web UI からの操作、Gazebo によるシナリオテストを含む。

## 主な機能

- **地図作成**: GNSS 拘束付きの 2D SLAM ([slam_gnss_2d](./slam_gnss_2d/README.md))。slam_toolbox 版もある ([mg_slam](./mg_slam/README.md))
- **自己位置推定**: ホイールオドメトリ・AMCL・GNSS を EKF で融合し、ずれを検知して復旧する ([mg_navigation](./mg_navigation/README.md))
- **自律走行**: Nav2 とウェイポイントシーケンサ ([mg_waypoint_navigation](./mg_waypoint_navigation/README.md))
- **障害物検出**: RealSense の点群からの 3D 障害物検出 ([mg_drivers](./mg_drivers/README.md))
- **操作・監視**: ブラウザの Web UI ([mg_ui](./mg_ui/README.md))
- **シミュレーションとテスト**: Gazebo Fortress と、シナリオによる回帰テスト ([mg_simulation](./mg_simulation/README.md)、[mg_scenario_test](./mg_scenario_test/README.md))
- **データ解析**: rosbag の統計・GNSS 軌跡・自己位置推定の評価 ([tools](./tools/README.md))

## システム構成

```
センサ (LiDAR / GNSS / RealSense / ホイール)
   └─► mg_drivers ─► 自己位置推定 (AMCL + GNSS + オドメトリ → EKF)
                          └─► Nav2 ─► collision_monitor ─► velocity_smoother ─► モータ
                                ▲
              waypoint_sequencer ◄── Web UI (foxglove_bridge / system_manager)
```

ノードの接続、launch の階層、座標系 (TF)、センサの取り付け位置は [doc/system_architecture.md](./doc/system_architecture.md)。

## 必要なもの

- Docker Engine 24.0 以上、Docker Compose v2.20 以上 (BuildKit 有効)
- Gazebo を GPU で動かす場合は、Nvidia Container Toolkit (Nvidia) または ROCm 対応のデバイス (AMD)

ローカルに ROS 2 環境は不要。ビルド・実行・テストはすべてコンテナ内で行う。

## クイックスタート

Gazebo のシミュレーションで、ナビゲーションを動かすまで。

```bash
cp .env.example .env            # 必要に応じて編集する (USE_GPU など)
make build-sim                  # イメージをビルド (初回は時間がかかる)
make xhost                      # GUI を使うとき
make gazebo-simulation          # 端末 1: シミュレータ
make navigation                 # 端末 2: 自己位置推定 + Nav2 + ウェイポイントシーケンサ
```

別の端末で、`make rviz2-navigation` (RViz2) や `make ui-all` (Web UI。`http://localhost:8080`) を起動して確認する。
実機で動かすときは、`make build-robot` でビルドし、`.env` で `SIMULATION=false` にする ([doc/operation.md](./doc/operation.md))。

## 使い方

実機で地図を作って走らせ、結果を解析するまでの流れ。詳細は [doc/operation.md](./doc/operation.md)。

| 手順 | コマンドの例 | 詳細 |
| :--- | :--- | :--- |
| 地図を作る | `make slam-gnss-2d` | [slam_gnss_2d](./slam_gnss_2d/README.md) |
| ウェイポイントを作る | waypoint-tool で作成 | [waypoint_format](./mg_waypoint_navigation/doc/waypoint_format.md) |
| 走らせる | `make navigation`、`make ui-all` | [mg_navigation](./mg_navigation/README.md)、[mg_ui](./mg_ui/README.md) |
| ログを解析する | `make bag-summary`、`make bag-eval-localization BAG=<bag>` | [tools](./tools/README.md) |

make ターゲットの全一覧は [doc/commands.md](./doc/commands.md)、`.env` の変数とデータの置き場は [doc/environment.md](./doc/environment.md)。

## 開発

```bash
make develop                    # develop コンテナを起動
make shell-develop              # コンテナの bash に入る
make test                       # 全パッケージの pytest
make test pkg=mg_navigation     # 特定のパッケージ
make ui-lint && make ui-test    # Web UI の型チェック・lint・テスト
make scenario-test-all TIER=smoke   # シミュレータでの回帰テスト (変更のたびに実行)
```

- コーディング規約: Python は PEP 8、C++ は Google C++ Style Guide。
- launch の引数は `LaunchArgumentCreator` で定義する ([mg_utils](./mg_utils/README.md))。
- AI エージェント向けのルールは [AGENTS.md](./AGENTS.md)。
- パッケージごとの設計・仕様は、各パッケージの `README.md` と `doc/` に書く。

## リポジトリ構成

| ディレクトリ | 役割 |
| :--- | :--- |
| [mg_bringup](./mg_bringup/README.md) | slam / navigation を束ねるトップレベルの launch |
| [mg_description](./mg_description/README.md) | URDF と RViz の設定 |
| [mg_drivers](./mg_drivers/README.md) | センサ・モータのドライバ、オドメトリの補正、3D 障害物検出 |
| [mg_navigation](./mg_navigation/README.md) | Nav2 の設定、自己位置推定の補助ノード |
| [mg_waypoint_navigation](./mg_waypoint_navigation/README.md) | ウェイポイントシーケンサ |
| [slam_gnss_2d](./slam_gnss_2d/README.md) | GNSS 拘束付き 2D SLAM (C++)、GNSS ブリッジ |
| [mg_slam](./mg_slam/README.md) | slam_toolbox の launch、地図プレビュー |
| [mg_ui](./mg_ui/README.md) | Web UI と system_manager |
| [mg_diagnostics](./mg_diagnostics/README.md) | `/diagnostics` への診断の配信 |
| [mg_msgs](./mg_msgs/README.md) | メッセージ・サービスの定義 |
| [mg_utils](./mg_utils/README.md) | launch 引数の補助、点群の変換、rosbag の収録 launch |
| [mg_simulation](./mg_simulation/README.md) | Gazebo Fortress のワールドと launch |
| [mg_simulator_client](./mg_simulator_client/README.md) | Unity シミュレータとの接続 |
| [mg_scenario_test](./mg_scenario_test/README.md) | MG-01 用のシナリオテスト |
| [sim_scenario_test](./sim_scenario_test/README.md) | ロボットに依存しない、Gazebo + Nav2 のシナリオテスト基盤 |
| [nav2_pkg](./nav2_pkg/README.md) | 修正を加えた Nav2 のパッケージ |
| [geometry2_pkg](./geometry2_pkg/README.md) | apt の版の不具合を避けるため、修正済みの上流の版を取り込んだ geometry2 のパッケージ (`tf2`) |
| [tools](./tools/README.md) | rosbag の解析・可視化、評価のスクリプト |
| `docker/` | Dockerfile と、ビルド補助のスクリプト ([doc/docker.md](./doc/docker.md)) |
| `doc/` | プロジェクト全体のドキュメント |

## ドキュメント

| ドキュメント | 内容 |
| :--- | :--- |
| [doc/system_architecture.md](./doc/system_architecture.md) | ノードの接続、launch の階層、TF、センサ、ポート |
| [doc/operation.md](./doc/operation.md) | 地図作成から走行、解析までの運用手順 |
| [doc/commands.md](./doc/commands.md) | make ターゲットと compose サービスの一覧 |
| [doc/environment.md](./doc/environment.md) | `.env` の変数、地図の指定、データの置き場 |
| [doc/docker.md](./doc/docker.md) | Docker イメージの構成、ビルド、GPU |

各パッケージの README から、そのパッケージの `doc/` へたどれる。
