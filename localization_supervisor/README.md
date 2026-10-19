# localization_supervisor

自己位置推定が地図やほかの情報源と合っているかを検証し、ずれたときに復旧するためのフレームワーク。
特定のロボットに依存しない。ロボットごとのトピック・設定・部品は、使う側のパッケージから与える。

メッセージは [localization_supervisor_msgs](../localization_supervisor_msgs/README.md)。

## 構成

| ディレクトリ | 内容 |
| :--- | :--- |
| `localization_supervisor/core/` | ROS に依存しない中核 (標本の履歴と時刻のそろえ方、判定、総合、状態遷移、アンカー、復旧の候補と選び方、照合、安全の信号、状態と診断の中身、設定の検証、故障注入、評価の集計、ゲートの調停) |
| `localization_supervisor/ros/` | ROS に依存する部品 (入力のアダプタ、再初期化の送り先、対処、イベントの出力、オフライン評価の実行部、ノード、状態のメッセージ、診断) |
| `config/examples/` | 特定のロボットではない、最小の設定の例 |
| `scripts/` | ノードとオフライン評価を起動するラッパー |

情報源・判定・総合の組み合わせは、設定ファイル ([doc/parameters.md](./doc/parameters.md)) に書く。判定などはプラグインとして足せる ([doc/plugins.md](./doc/plugins.md))。全体の設計は [doc/design.md](./doc/design.md)。

## 実行ファイル

| 実行ファイル | 内容 |
| :--- | :--- |
| `localization_supervisor_node.py` | 設定ファイル (パラメータ `config_file`) の組み合わせで中核を周期ごとに回し、状態 (`LocalizationStatus`)・診断・イベントを出し、異常を確認したら切り離し・再初期化して復旧する ([doc/supervisor_node.md](./doc/supervisor_node.md)) |
| `gate_arbiter_node.py` | `SetBool` で開閉するゲートを、要求元ごとに調停する ([doc/gate_arbiter.md](./doc/gate_arbiter.md)) |
| `offline_runner.py` | rosbag を読み、中核を実時間を待たずに回して、故障注入の検知率・誤検知を出す ([doc/design.md](./doc/design.md#オフライン評価)) |

launch は持たない。使う側のパッケージが、自分の設定で起動する。

## 依存

[package.xml](./package.xml) を参照。ロボット固有のパッケージには依存させない。

## テスト

```bash
make test pkg=localization_supervisor
```

- `test_framework_independence.py`: `package.xml` の依存と Python の import に、ロボット固有のパッケージ (`mg_` で始まるもの) が無く、中核が ROS に依存しない
- `test_supervisor_core.py`: 時刻のそろえ方、判定、総合 (情報源の切り分けを含む)、状態遷移、アンカー、Supervisor
- `test_supervisor_scan_checks.py`: スキャンを照合する判定 (`submap_map`、`scan_drift`、`freespace`) とスキャンのバッファ
- `test_supervisor_safety.py`: 安全の信号、外からの再初期化・地図の切り替え・走行中の変更への対処
- `test_recovery_matcher.py`: 総当たりの照合 (既知のずれを求められる、似た場所の候補を離して返す)
- `test_recovery.py`: 復旧の選び方 (決めきれなければ再初期化しない)、設定の検証、Supervisor が再初期化を指示するまで (GNSS・推測航法・照合の候補)
- `test_recovery_ros.py`: 再初期化の送り先、対処、自分のメッセージの目印 (rclpy は偽物)
- `test_residuals.py`: 残差から求める分散の倍率 (受け取る側がかけたときに落ち着く、信頼できる周期だけ、イベント)、標本からの倍率の割り戻し、倍率を出す対処
- `test_isolate_only.py`: 正しいと示す判定 (`vouch`) による切り分け、切り離すだけの blame (`isolate_only`) の状態遷移と、GNSS だけがずれたときの Supervisor 全体、復旧の候補から疑わしい情報源を外すこと、対処の無効化
- `test_supervisor_status.py`: 状態と診断の中身、イベントの出力、ノードが動かせる設定かの検証
- `test_supervisor_config.py`: 設定の検証 (誤りは読み込みで落ちる)、外部のプラグインの読み込み、プロファイル、`config/examples/` が読める
- `test_supervisor_faults.py`: 故障注入と評価の集計
- `test_offline_runner.py`: 人工のメッセージの列と地図で、`config/examples/` の設定をオフライン評価に通す

ROS のモジュールは `test/conftest.py` でモックする (入力のアダプタは、メッセージの属性だけを読む)。rosbag の読み込みは、実データでのオフライン評価で確かめる。

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [doc/design.md](./doc/design.md) | 層、1 周期の流れ、時刻のそろえ方、判定、総合と情報源の切り分け、状態遷移、アンカー、復旧、安全の信号、想定外への対処、プロファイル、オフライン評価 |
| [doc/plugins.md](./doc/plugins.md) | プラグインの約束 (判定・総合・復旧の候補と評価・照合・再初期化の送り先・対処・入力のアダプタ・イベントの出力)、標準のプラグイン、追加の手順 |
| [doc/parameters.md](./doc/parameters.md) | 設定ファイルの書き方 |
| [doc/supervisor_node.md](./doc/supervisor_node.md) | 監督ノードのパラメータ・入出力・状態のメッセージ・診断とイベント |
| [doc/gate_arbiter.md](./doc/gate_arbiter.md) | ゲートの調停の規則・出力・パラメータ |
