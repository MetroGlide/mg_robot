# AIエージェント向け開発ガイドライン (AGENTS.md)

本ドキュメントは、AntigravityやGitHub CopilotをはじめとするすべてのAIエージェントが本プロジェクトで作業を行う際のガイドラインとコンテキストをまとめたものです。

---

## 共通基本ルール

すべてのAIエージェントは、以下のルールを厳守してください。

1. **日本語でのコミュニケーション**
   - チャットでの回答、コードへのコメント、ドキュメントの記述（Artifacts含む）はすべて**日本語**で行ってください。
2. **計画の事前合意**
   - ユーザーから「計画」や「方針」を求められた場合、勝手にコード編集などの実装フェーズへ進まないでください。まずは提案を行い、ユーザーの合意を得てから次のステップに進んでください。
3. **ファイル・Git操作の制限**
   - 明示的な指示がない限り、ファイルの削除や `git` 操作（コミット、プッシュなど）は行わないでください。
4. **差分記録コメントの禁止**
   - コメントはコード自体の意図やロジックのみを簡潔に記載してください。変更履歴や差分記録（例: `// Modified by AI`）をコメントとして残すことは禁止します。
5. **最小限の変更**
   - 指示された内容と無関係な箇所の変更や、不要なコード整形、指示のないリンターエラーの修正は行わないでください。
6. **後方互換性**
   - 特段の指示がない限り、後方互換性は考慮しません。
7. **PEP8 / Google C++ Style Guide**
   - PythonコードはPEP8、C++コードはGoogle C++ Style Guideに従ってください。
8. **環境不備の隠蔽禁止と厳格なImport（PEP8準拠）**
   - パッケージやメッセージの依存関係は事前に解決されていることを前提とします。
   - `ImportError` を `try-except` で握りつぶすような実装や、関数内での遅延importは**一切禁止**します。すべてのimportはファイル先頭で行い、エラーになる場合はそのままクラッシュさせてください。

---

## プロジェクト概要

チーム MetroGlide による自律移動台車ロボット MG-01 の開発プロジェクト。詳細は [README.md](./README.md) を参照。

## 実行環境

**ローカルにROS2環境はありません。全操作はDockerコンテナ内で行います。**

```bash
make develop          # developコンテナ起動（バックグラウンド）
make shell-develop    # developコンテナにbashアクセス
make shell svc=slam   # 実行中コンテナにアクセス
make build-robot      # 実機向け一括ビルド（Gazeboシミュレータ完全除外）
make build svc=slam   # イメージビルド（collect_deps.sh自動実行）
make test             # 全テスト実行
make test pkg=<pkg>   # 特定パッケージのテスト
make bag-summary      # .env指定のrosbagを解析し同ディレクトリにsummary.mdを出力
make bag-plot-gnss    # GNSS軌跡・Fix状態・精度を可視化（CIRCLES=1で精度円、SCALE=10で倍率指定、TO_TOOLS=1でtools/data/保存）
make bag-plot-scans   # LiDARスキャン点群を2D画像化（NODES=1:20等でノード指定）
make scenario-validate  # シナリオYAMLの静的検証（シミュレータ不要）
make scenario-test SCENARIO=<名前|パス>  # Gazebo+Nav2を起動して1本実行（ヘッドレス、GUI=1で表示）。詳細は mg_scenario_test/README.md
make scenario-test-all TAGS=smoke REPEAT=3  # シナリオをスタック起動し直しで一括実行
```

コンテナ内でROS2コマンドを使う場合:

```bash
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
```

ホスト→コンテナのマウント: `/home/chuson/ros_workspace/mg_robot/` → `/app/`

## コーディングスタイル

- Python: PEP8 / C++: Google C++ Style Guide
- パッケージ: モノレポ構成（`/app/` 以下が各ROSパッケージ）

## パッケージ構成

| パッケージ               | 役割                                                           |
| ------------------------ | -------------------------------------------------------------- |
| `mg_bringup`             | slam/navigationを束ねるトップレベルlaunch群                    |
| `mg_description`         | URDF・RViz設定                                                 |
| `mg_diagnostics`         | `/diagnostics`トピックへの正常性診断配信                       |
| `mg_drivers`             | LiDAR/DepthCam/GPS/IMU/モータドライバ群                        |
| `mg_msgs`                | カスタムメッセージ・サービス定義                               |
| `mg_navigation`          | Nav2ラッパー(collision_monitor/behavior_server設定, AMCL watchdog, GNSS初期化) |
| `mg_scenario_test`       | MG-01用のシナリオテスト（プロファイル・プラグイン・シナリオ。[README](./mg_scenario_test/README.md)） |
| `sim_scenario_test`      | ロボット非依存のGazebo+Nav2シナリオテスト基盤（mg_*に依存しない。[README](./sim_scenario_test/README.md)） |
| `mg_simulation`          | Gazebo Fortress ワールド・launch設定                           |
| `mg_simulator_client`    | シミュレータ操作クライアント                                   |
| `mg_slam`                | slam_toolbox + KISS-ICP launch・パラメータ                     |
| `mg_ui`                  | Web UI / TUI / system_manager の3サブパッケージ                |
| `mg_utils`               | `LaunchArgumentCreator` ヘルパー                               |
| `mg_waypoint_navigation` | FSMベースのウェイポイントシーケンサ(Nav2 ActionClientラッパー) |
| `nav2_pkg`               | **カスタム修正済み**のNav2（アップストリームと差分あり）       |

## 重要なコード規約

### launch ファイル

launch引数は必ず `LaunchArgumentCreator` 経由で定義する（[mg_utils/launch_argument.py](./mg_utils/mg_utils/launch_argument.py)）:

```python
from mg_utils.launch_argument import LaunchArgumentCreator

def generate_launch_description():
    arg = LaunchArgumentCreator()
    simulation = arg.create("simulation", default="true")
    return LaunchDescription([
        *arg.get_created_declare_launch_args(),
        ...
    ])
```

`SIMULATION` 環境変数で実機/シミュを切り替え: `EnvironmentVariable("SIMULATION")`

### パラメータYAML

```yaml
node_name:
  ros__parameters:
    param_key: value
```

## カスタムメッセージ・サービス

定義ファイル: [mg_msgs/msg/](./mg_msgs/msg/)、[mg_msgs/srv/](./mg_msgs/srv/)

## mg_ui

[mg_ui/README.md](./mg_ui/README.md) を参照。

| サブパッケージ      | 技術                                            |
| ------------------- | ----------------------------------------------- |
| `mg_web_ui`         | React 18 + TypeScript + Vite + Tailwind CSS     |
| `mg_tui`            | Python TUI ([README](./mg_ui/mg_tui/README.md)) |
| `mg_system_manager` | FastAPI + Docker SDK（ROS2非依存）              |

**mg_web_ui フロントエンド:**

- ROS通信: foxglove_bridge `ws://localhost:8765`
- 主要フック: `useFoxgloveClient`, `useTopicSubscriber`, `useServiceCaller`, `useNav2Status`, `useSystemManagerClient`
- トピック・サービス定義: [ros/topics.ts](./mg_ui/mg_web_ui/frontend/src/ros/topics.ts), [ros/services.ts](./mg_ui/mg_web_ui/frontend/src/ros/services.ts)
- foxglove経由でROS型を扱う場合はschema名が必要 → [ros/schemas.ts](./mg_ui/mg_web_ui/frontend/src/ros/schemas.ts) を参照

## テスト

```bash
make test             # Dockerコンテナ内で全パッケージテスト
make test pkg=mg_waypoint_navigation
```

## rosbag データと統計サマリー (summary.md)

デバッグや走行ログ・センサデータの調査を行う際は、以下のルールとツールを活用してください。

1. **ディレクトリ内の既存統計情報 (`summary.md`) の確認**:
   - 各 rosbag ディレクトリ内には、事前に解析された統計サマリーファイル（`summary.md`）が配置されている場合があります。
   - **巨大な rosbag を都度全件走査・解析し直す前に、まず該当ディレクトリ内に `summary.md` が存在するか確認し、その内容を参照してください。**
   - `summary.md` には、トピック一覧、周波数、通信ドロップ警告、GNSSのRTK Fix率や精度統計（min/max/mean/median/95%）、精度ヒストグラム、オドメトリ積算距離などがまとめられています。

2. **統計サマリーの生成・更新**:
   - `.env` で指定された rosbag (`ROSBAG_FILE` または `BAG_PATH`) のサマリーを同じディレクトリに生成/更新する場合:
     ```bash
     make bag-summary
     ```
   - 任意の rosbag パスを指定して生成する場合:
     ```bash
     make bag-summary BAG=/root/ros2_data/rosbag/TC2026/20260913/record_all_20260913_055508
     ```
   - これにより、対象 rosbag と同じディレクトリに `summary.md` が保存され、コンソールにも要約が表示されます。

## 開発・デバッグ用ツール群 (`tools/`)

データ解析、GNSS軌跡可視化、LiDARスキャン確認などの各種デバッグ用ツールが `tools/scripts/` に整備されています。
**デバッグやデータ調査を行う際は、都度使い捨てスクリプトを作成せず、まず `tools/` 配下に用意されている既存ツール群を活用してください。**

詳細は [tools/README.md](./tools/README.md) を参照。

| スクリプト | 役割 |
| :--- | :--- |
| `tools/scripts/rosbag_summary.py`<br>(`make bag-summary`) | rosbagの通信健全性・GNSS Fix率・精度統計・ヒストグラム出力 |
| `tools/scripts/plot_gnss_trajectory.py`<br>(`make bag-plot-gnss`) | GNSS軌跡・Fix状態・精度の可視化、オドメトリ比較、地図画像オーバーレイ |
| `tools/scripts/plot_lidar_scans.py`<br>(`make bag-plot-scans`) | LiDARスキャン点群の2D画像化・SLAMノード調査 |
| `tools/scripts/generate_static_transforms.py` | 地図とGNSSの対応点から剛体変換 (x,y,yaw) を算出 |
| `tools/scripts/rosbag_modify_base.py` | rosbag内の特定トピック修正・書き換え |
| `tools/scripts/bag_to_json.py` | rosbagの指定トピック/全メッセージのJSONダンプ |

### 解析結果・可視化画像の保存先切り替え

`make bag-*` コマンドおよび解析ツールは、出力先の切り替えに対応しています：
- **デフォルト**: rosbag と同じディレクトリに保存（例: `summary.md`, `gnss_trajectory.png`, `lidar_scans_plot.png`）
- **`TO_TOOLS=1` または `OUT_DIR=tools`**: `tools/data/` ディレクトリに保存（Git追跡除外されているため、一時調査・確認に推奨）
- **`OUT_DIR=<dir>`**: 任意ディレクトリに保存
- **`OUT=<file>`**: 指定ファイルパスに直接保存
