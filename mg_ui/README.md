# mg_ui

MG-01 の状態の表示と操作のための UI。ブラウザ (タブレットを含む) から、ロボットの状態の確認、ウェイポイントの走行、SLAM、シナリオテスト、サービスの起動・停止ができる。

## パッケージ構成

| パッケージ | 内容 |
| :--- | :--- |
| `mg_web_ui` | React のフロントエンドと、静的配信の HTTP ノード (`http_server_node.py`、`web_ui.launch.py`) |
| `mg_system_manager` | docker compose のサービスの操作、地図の保存、UI の設定、ログの配信を行う FastAPI のサーバ |

```
ブラウザ ── WebSocket :8765 ──► foxglove_bridge ──► ROS 2 (トピック・サービス)
   │
   ├── HTTP :8080 ──► http_server_node (フロントエンドの配信)
   └── HTTP/WebSocket :8001 ──► system_manager ──► docker compose (サービスの起動・停止、ログ)
```

正常性の診断は [mg_diagnostics](../mg_diagnostics/README.md) が配信する (UI のトップと System ページが表示する)。

## 画面

### 新 UI (`/ops`)

地図を全面に置き、計器と操作を重ねる UI。ライト/ダークのテーマを端末ごとに切り替えられる。旧ページ (下の表) への移行の途中で、両方を残している
([移行の ToDo](./mg_web_ui/doc/ui_migration_todo.md))。上部バーのユースケース (Waypoint / SLAM)、センサ、システムで切り替える。

| パス | 内容 |
| :--- | :--- |
| `/ops/waypoint` | ウェイポイントの走行。CPU・自己位置・GNSS・速度、走行の進捗 (開始・停止・一時停止)、センサ・トピック・ノードの状態、手動のゴールと初期姿勢の指定 |
| `/ops/slam` | slam_toolbox の SLAM の起動・停止、地図の保存 |
| `/ops/sensors` | RViz ライクなセンサビュー (全レイヤー、2D/3D、カメラ・深度画像) |
| `/ops/system` | サービスの起動・停止、診断、ログ |

地図はタッチパッドで操作する (2 本指のスクロールで移動、ピンチでズーム)。受信が途切れた値は `--` と経過秒の表示になり、ロボットと切断中は画面の上に帯が出て、操作が無効になる。

### 旧ページ

| ページ | パス | 内容 |
| :--- | :--- | :--- |
| TOP | `/` | ロボットの状態、診断 |
| Waypoint Nav | `/waypoint` | ウェイポイントの走行、一時停止、手動のゴール (BT の選択)、地図の切り替え、AMCL・GNSS の入/切 (Actions) |
| SLAM Toolbox | `/slam` | slam_toolbox の SLAM |
| SLAM-GNSS-2D | `/slam-gnss-2d` | GNSS 拘束付き SLAM、ポーズグラフ、地図の保存・プレビュー・再最適化 ([表示の説明](./mg_web_ui/doc/slam_gnss_2d_visualization.md)) |
| Scenario Test | `/scenario-test` | シナリオテストの実行と結果 ([mg_scenario_test](../mg_scenario_test/README.md#web-ui-から実行する)) |
| System | `/system` | サービスの起動・停止、コンテナの状態、ログ |
| Setting | `/setting` | UI の設定 (可視化のレイヤーなど) |

## 使い方

### 本番

前提: `make slam` または `make navigation` が起動済みであること。操作系の機能を使うには、`make diagnostics` と `make system-manager` も必要。

```bash
make build svc=web-ui    # 初回と、フロントエンドを変えたとき
make ui-all              # system-manager・web-ui・foxglove-bridge・diagnostics を起動
```

タブレットやブラウザから `http://<ロボットの IP>:8080` を開く。個別に起動するときは、`make web-ui`、`make foxglove-bridge`、`make system-manager`、`make diagnostics`。

`web_ui.launch.py` の引数: `port` (既定 8080)、`dist_dir` (空なら、インストールした `frontend/dist`)。

### 開発 (変更をすぐ反映する)

```bash
make ui-dev-all          # web-ui の代わりに Vite の開発サーバ (HMR、ポート 5173) を起動
make web-ui-dev          # 開発サーバだけ
```

`http://localhost:5173` (または `http://<ロボットの IP>:5173`) を開く。foxglove_bridge は、`make ui-dev-all`、`make ui-all`、`make foxglove-bridge` のどれかで起動しておく (`make web-ui` は foxglove_bridge を含まない)。

### 検査とテスト

```bash
make ui-lint                              # フロントエンドの型チェック (tsc) と ESLint
make ui-test                              # フロントエンド (vitest) と system_manager (pytest)
make test pkg=mg_ui/mg_system_manager     # system_manager のテストだけ
```

## ポート

| ポート | 内容 |
| :--- | :--- |
| 8765 | foxglove_bridge (WebSocket) |
| 8080 | フロントエンドの配信 |
| 5173 | Vite の開発サーバ |
| 8001 | system_manager |

## ROS との通信

- トピックの購読と publish、サービスの呼び出しは、foxglove_bridge (`ws://localhost:8765`) を通す。定義は `frontend/src/ros/` ([topics.ts](./mg_web_ui/frontend/src/ros/topics.ts)、[services.ts](./mg_web_ui/frontend/src/ros/services.ts)、[schemas.ts](./mg_web_ui/frontend/src/ros/schemas.ts))。
- 主なフック: `useFoxgloveClient`、`useTopicSubscriber`、`useServiceCaller`、`useNav2Status`、`useSystemManagerClient`。
- foxglove_bridge は、UI が使わない機能 (`connectionGraph`、`parameters`、`assets`、`sysinfo`) を止めて起動する ([doc/tuning.md](./doc/tuning.md))。

## 開発

- フロントエンド: [mg_web_ui/doc/frontend_development.md](./mg_web_ui/doc/frontend_development.md) (構成と依存の向き、ROS 通信のルール、機能・ページ・レイヤーの足し方)
- system_manager: [mg_system_manager/doc/system_manager.md](./mg_system_manager/doc/system_manager.md) (構成、エンドポイント、環境変数、機能の足し方)
- 負荷を下げる設定と、既知の制約: [doc/tuning.md](./doc/tuning.md)

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [mg_web_ui/doc/frontend_development.md](./mg_web_ui/doc/frontend_development.md) | フロントエンドの開発ガイド |
| [mg_web_ui/doc/slam_gnss_2d_visualization.md](./mg_web_ui/doc/slam_gnss_2d_visualization.md) | SLAM-GNSS-2D ページの表示の説明 |
| [mg_system_manager/doc/system_manager.md](./mg_system_manager/doc/system_manager.md) | system_manager の仕様 |
| [doc/tuning.md](./doc/tuning.md) | 負荷を下げる設定、既知の制約 |
