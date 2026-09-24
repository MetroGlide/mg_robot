# mg_scenario_test

MG-01 用の [sim_scenario_test](../sim_scenario_test/README.md) アダプタです。
Gazebo 上で MG-01 のナビゲーションをシナリオ YAML で自動テストします。

| 内容 | 場所 |
|---|---|
| ロボットプロファイル (sim・スタックの起動、座標系、ワールドと地図) | `profiles/mg01.yaml` |
| MG 固有のプラグイン (waypoint_sequencer 用ドライバ・action・trigger、`collision_monitor` の monitor、MG の waypoint 形式) | `mg_scenario_test/` |
| シナリオ | `scenarios/` ([一覧](scenarios/README.md)) |
| ユニットテスト | `test/` |
| シミュレータを使う pytest 結合テストの例 | `sim_tests/` |

汎用の仕組み (シナリオの書式・型・CLI・結果) は `sim_scenario_test` 側のドキュメントを参照してください。

## クイックスタート

すべて Docker コンテナ内で動きます (ローカルに ROS 環境は不要)。GPU を使う場合は `.env` に `USE_GPU=nvidia` (または `amd`) を書きます。
**GPU なしだとセンサーの描画がソフトウェアレンダリングになり、メッシュの大きいモデルでシミュレーション速度が大きく落ちます。**

```bash
make build svc=scenario-test                        # 初回、およびファイル (launch・スクリプト・Python モジュール・データ) を追加したとき
make scenario-validate                              # シナリオ YAML の検証 (シミュレータ不要、数秒)
make scenario-test SCENARIO=nav_basic_goal          # 1 本を起動から判定まで実行 (ヘッドレス)
make scenario-test SCENARIO=nav_basic_goal GUI=1    # Gazebo の GUI を表示 (xhost が必要)
make scenario-test-all TIER=smoke                   # 変更のたびに実行する短い確認 (約 6 分)
make scenario-test-all                              # 回帰テスト全件 (約 20 分)
make scenario-test-all REPEAT=3 TIER=smoke          # 繰り返して安定性を見る
```

| ターゲット | 内容 |
|---|---|
| `make scenario-test SCENARIO=<名前\|パス> [GUI=1] [PROFILE=]` | シミュレータ・ナビゲーションごと起動して 1 本実行。名前は `scenarios/regression/`・`examples/` 配下 (拡張子なし) |
| `make scenario-test-all [TIER=smoke] [TAGS=a,b] [REPEAT=N] [EXAMPLES=1] [KNOWN=1] [PROFILE=]` | 回帰テストを、1 本ごとにスタックを起動し直して実行。`TIER=smoke` で smoke タグのみ、`EXAMPLES=1` で見本も含める、`KNOWN=1` で既知の問題 (`known_issue` タグ) のシナリオも含める |
| `make scenario-test-attach SCENARIO=...` | 起動済みのシミュレータ・スタックに接続して実行 (`make gazebo-simulation` と `make navigation` が別途必要。開発時の反復用) |
| `make scenario-validate` | シナリオ YAML の静的検証 |
| `make test pkg=mg_scenario_test` | ユニットテスト |

**実装を変えたときの運用**: 変更のたびに `make scenario-test-all TIER=smoke` (約 6 分)、リリース前・夜間に `make scenario-test-all` (全件)。
不安定さの確認には `REPEAT=3` 以上で繰り返します。シナリオの内容と保証することは [scenarios/README.md](scenarios/README.md) を参照。

実行中は Gazebo・Nav2 を専有するため、同時に別のスタック (`make gazebo-simulation` 等) を起動しないでください。

## 結果

`${ROS2_DATA_PATH}/scenario_results/<日時>/` に保存されます。

```
<日時>/
  junit.xml                    # スイート全体の JUnit
  <シナリオ名>[_runN]/
    result.json                # チェックごとの結果、走行中のイベント (goal_reached など) と時刻
    launch.log                 # シミュレータ・Nav2・runner の全ログ
```

- 終了コード: 0=PASSED、1=FAILED、2=ERROR。**make 経由では失敗時に make の仕様で常に非 0 (2) になる**ので、
  区別が必要なら実行末尾の `Summary` か `result.json` / `junit.xml` を見てください。
- `FAILED` はロボットが期待どおりに動かなかった場合、`ERROR` はテスト基盤・セットアップの失敗です
  (起動しない、障害物が生成されない、シミュレータが止まった等。詳細は `sim_scenario_test/docs/architecture.md`)。

## シミュレーション用のデータ

`mg_simulation` に、実機の `~/ros2_data` に依存しないシミュレーション用の地図と waypoint があります。

| ファイル | 内容 |
|---|---|
| `mg_simulation/worlds/warehouse.sdf` | ワールド |
| `mg_simulation/maps/warehouse/{localization,planning}.yaml`, `map.pgm` | 測位・計画用の地図 |
| `mg_simulation/maps/warehouse/waypoints.yaml` | sequencer 用の waypoint (通過点、許容距離 1.5 m) |

## トラブルシュート

| 症状 | 原因・対処 |
|---|---|
| `ERROR ... no result was produced` | 起動に失敗している。`launch.log` の先頭側のエラーを見る。新しいファイルを追加したなら `make build svc=scenario-test` |
| `readiness timeout ... simulator world` | シナリオの `world` とシミュレータのワールドの不一致、または Gazebo が起動していない |
| `... does not exist after 5s (model download failure?)` | Fuel モデルの取得に失敗。初回は `~/.ignition/fuel` へダウンロードされる。オフラインなら `primitive` か `local` モデルを使う |
| シミュレーションが極端に遅い (RTF が 0.05 など) | GPU が使われていない (`USE_GPU`)。大きなメッシュのモデルを避ける |
| `simulation clock stalled` | シミュレータが停止・フリーズしている |
| 同じスタックで繰り返すと不安定になる | 仕様どおりシナリオごとに起動し直す (`make scenario-test` / `-all`)。`-attach` は反復開発用 |
| 障害物のシナリオが FAILED になる | `result.json` の `checks` と `launch.log` を確認。既知の問題は [scenarios/README.md](scenarios/README.md) |

## ドキュメント

| ファイル | 内容 |
|---|---|
| [docs/mg_plugins.md](docs/mg_plugins.md) | MG 固有の拡張 (driver / action / trigger / monitor / waypoint 形式) |
| [docs/writing_scenarios.md](docs/writing_scenarios.md) | MG-01 向けのシナリオの書き方・パターン・デバッグ |
| [scenarios/README.md](scenarios/README.md) | 同梱シナリオの一覧 |
