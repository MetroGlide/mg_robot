# mg_ui

MG-01 の状態表示・操作 UI パッケージ群。

## パッケージ構成

| パッケージ          | 説明                                                                        |
| ------------------- | --------------------------------------------------------------------------- |
| `mg_web_ui`         | React フロントエンド + foxglove_bridge + HTTP 静的配信ノード                |
| `mg_system_manager` | docker compose のサービス操作・地図保存・UI 設定・ログ配信を行う FastAPI サーバ |

正常性診断は `mg_diagnostics/`（プロジェクトルート）で管理しています。

## 前提条件

- `make slam` または `make navigation` が起動済みであること
- `make diagnostics` と `make system-manager` が起動済みであること（操作系機能を使う場合）

## 通常起動（本番）

```bash
# 本番ビルド（初回または frontend 変更時）
make build svc=web-ui

# ブラウザ UI 起動（foxglove_bridge:8765 + HTTP:8080）
make web-ui

# タブレット / ブラウザからアクセス
# http://<ロボットIP>:8080
```

## 開発モード（フロントエンド変更を即時反映）

```bash
# Vite devサーバー起動（HMR 有効、ポート 5173）
make web-ui-dev

# ブラウザからアクセス
# http://localhost:5173  or  http://<ロボットIP>:5173
```

> foxglove_bridge は `make web-ui` または `make slam` / `make navigation` 側で起動していること。

## 診断・システム管理

```bash
make diagnostics     # /diagnostics トピックへの正常性診断配信
make system-manager  # docker compose のサービス操作などを行う API サーバ (ポート 8001)
```

## 検査・テスト

```bash
make ui-lint   # フロントエンドの型チェック (tsc) と ESLint
make ui-test   # フロントエンド (vitest) と system_manager (pytest)
make test pkg=mg_ui/mg_system_manager   # system_manager のテストだけ
```

- フロントエンドの整形は Prettier（`frontend/.prettierrc.json`）を使う。**新規・変更したファイルにだけ適用**し、既存ファイルを一括整形しない。
- ESLint の `react-hooks/exhaustive-deps` は警告にしている。新しく書くコードでは警告を出さない。

## 開発ガイド

### mg_web_ui の構成と依存の向き

```
frontend/src/
  ros/          通信層（React に依存しない）: foxgloveConnection, codec, topics, services, schemas
  hooks/        通信層を React から使うフック（useFoxgloveClient, useTopicSubscriber など）
  contexts/     設定・状態の Provider
  components/
    layout/     ページ骨格（NavBar, RobotPageLayout, SideAccordion, SectionCard）
    panels/     計器・ログなどの表示パネル
    status/     コンテナ状態・サービス操作のカード
    sections/   複数の操作をまとめた節（rosbag 再生など）
    ros-viewer/ three.js による 2D/3D ビューワー（hooks/ と layers/）
    ui/         ボタンなどの汎用部品
  pages/        ルーティングされるページ
  utils/, types/
```

- import の向きは `pages → components / hooks / contexts → ros / utils / types`。**逆向きは禁止**。特に `ros/` から `hooks/` や `components/` を import しない。
- import はファイル先頭に書く（遅延 import・`try/catch` での握りつぶしは禁止。ルートの AGENTS.md）。
- 再エクスポートだけのファイルは作らない。実体のパスから import する。

### ROS 通信のルール

- 購読は `useTopicSubscriber(client, topic, schemaName)` を使う。購読は `FoxgloveConnection` がリスナーの登録として保持し、
  未接続・再接続・ノードの再起動（チャネル id の変更）があっても自動で張り直す。`client.status` で購読を出し分けなくてよい。
- **高頻度のトピック（scan・点群・画像・costmap・tf・odom など）は、表示するときだけコンポーネントをマウントして購読する**。
  非表示のレイヤーが購読し続けないようにする。タブが非表示の間は自動で購読が止まる。
- 連続して publish するトピック（`/cmd_vel` など）は、使う前に `client.advertise(topic, schemaName)` を呼んでおく。
  呼ばずに初めて publish すると、DDS のマッチングを待つため最初のメッセージが約 0.4 秒遅れる。
- publish するメッセージは `ros/schemas.ts` にスキーマを追加する。cdr でエンコードできない場合は例外になる（壊れたデータは送らない）。
- 新しいトピックは `ros/topics.ts`、サービスは `ros/services.ts` に定義する。

### 新しい機能を追加するとき

| 追加するもの                 | 場所                                                                                     |
| ---------------------------- | ---------------------------------------------------------------------------------------- |
| ROS トピックの表示           | `ros/topics.ts` に定義 → `useTopicSubscriber` で購読 → レイヤーまたはパネルとして表示       |
| ビューワーのレイヤー         | `components/ros-viewer/layers/` に追加し、`VisualizationContext` のレイヤー定義に登録      |
| system_manager の操作 API    | `mg_system_manager/routers/` に追加。入力は `config.py` の正規表現で検証する。テストを書く  |
| 操作対象の compose サービス  | `mg_system_manager/config.py` の `SERVICES` に追加（UI の System ページには自動で反映）    |
| UI 設定の保存                | `saveSettings(key, value)`（キー単位で保存される）                                          |

### mg_system_manager

`python3 -m mg_system_manager`（ポート 8001）。compose の `system-manager` サービスが起動する。

```
mg_system_manager/
  config.py         環境変数の設定、サービス定義（SERVICES）、入力検証の正規表現
  docker_ops.py     ComposeRunner: docker compose の実行、コンテナ検索、サービスごとの排他
  log_hub.py        コンテナログの配信（サービスごとに docker logs を 1 本、接続ごとに上限付きバッファ）
  settings_store.py UI 設定の保存（キー単位のマージ）
  routers/          エンドポイント
```

- コンテナは compose のプロジェクト（`COMPOSE_PROJECT_NAME`、未設定ならディレクトリ名）で絞り込む。
  `docker compose run` で作られた one-off コンテナも対象で、同じサービスに複数ある場合は動作中のものを優先する。
- 同じサービスへの操作は同時に 1 つだけ。実行中に別の操作が来たら `another operation is in progress` を返す。
  ハードウェアを使うサービス（`hardware=True`）とシナリオテスト用スタックの起動は互いに排他する。
- CORS は、localhost・プライベート IP・Tailscale・`.local` のホストの `:8080` / `:5173` を許可する。
  他のオリジンは環境変数 `SYSTEM_MANAGER_ALLOW_ORIGINS`（カンマ区切り）で追加する。
- 認証はない。信頼できるネットワークでのみ使うこと。

### 負荷を下げる設定と戻し方

実機 PC・タブレットの負荷を下げるために入れている設定。負荷が増える方向の変更は行っていない。

| 設定                                   | 効果                                                                 | 調整・元に戻す方法                                                       |
| -------------------------------------- | -------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| タブ非表示中の購読停止                 | 見ていないタブがトピックを受信しない(bridge の送信量とデコードが減る) | 自動。`FoxgloveConnection.setPaused`                                      |
| ビューワーの描画を要求ベースにする     | 常時 60fps だった描画を 20fps 程度にする                              | `RosViewer.tsx` の `RENDER_INTERVAL_MS`(大きいほど軽い)                    |
| 描画解像度の上限                       | 高 DPI 端末での描画負荷を抑える                                       | `RosViewer.tsx` の `MAX_DEVICE_PIXEL_RATIO`                                |
| 可視化プリセット(軽量・標準・すべて)   | 表示するレイヤーを減らして購読と描画を減らす                          | Settings の Visualization                                                 |
| 画像を canvas に直接描画               | dataURL への再エンコードをやめる                                      | —                                                                        |
| 地図・コストマップのテクスチャの使い回し | 更新のたびの GPU メモリの確保・解放をなくす                            | —                                                                        |
| foxglove_bridge の capabilities・sysinfo | UI が使わない機能(connectionGraph・parameters・assets・sysinfo)を止める | `compose.yaml` の foxglove-bridge の `capabilities` と `sysinfo` の 2 行を削除 |
| コンテナ状態の取得を軽くする           | Docker への問い合わせを減らす(1 秒キャッシュ、inspect を省略)          | `docker_ops.py` の `_STATUS_CACHE_TTL_S`                                   |
| ログ配信                               | `docker logs` を共有し、バッファに上限を付けてまとめて送る             | `routers/logs.py` の `MAX_BUFFERED_ENTRIES`・`FLUSH_INTERVAL_S`            |
| ページ単位の遅延読み込み               | 初期に読み込む JS を 1.5MB から 0.27MB にする                          | `App.tsx` の `lazy`                                                       |

計測するときは、ブラウザの Performance(メインスレッドの占有率・FPS)と、実機 PC で `top`(foxglove_bridge と system_manager の CPU)、
`nethogs` / `iftop`(bridge の送信量)を、変更前後で比べる。

### 既知の制約

- system_manager のテストには `httpx` が必要で、`requirements.txt` に追加してある。
  既存の develop イメージには入っていないため、`make build svc=develop` で再ビルドするまで `make ui-test` の pytest は失敗する。

- ジョイスティックは、操作中にタブの切替・ページ遷移・切断が起きたときは速度 0 を送る。
  ただしドラッグ中に通信が切れたりブラウザが落ちたりした場合は UI から停止を送れず、
  `motor_driver_node` に `cmd_vel` のタイムアウトがないため、最後の速度が保持される。
