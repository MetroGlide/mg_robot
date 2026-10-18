# localization_supervisor_node

設定ファイルに書いた情報源・判定・総合・状態遷移の組み合わせで、自己位置推定が地図やほかの情報源と合っているかを周期ごとに確かめ、状態・診断・イベントを出す。
判定と状態遷移は中核 (`core/`、[design.md](./design.md)) にあり、ノード (`ros/supervisor_node.py`) は、入力を標本にして中核に渡し、結果を出すだけにする。

今のノードは、検知と通知だけを行う。切り離し・再初期化の対処を持たないので、`state_machine.recovery_enabled` が `true` の設定 (基の設定とすべてのプロファイル) は起動時に落とす。

## パラメータ (ROS)

| 名前 | 内容 |
| :--- | :--- |
| `config_file` | 設定ファイルのパス (必須。[parameters.md](./parameters.md)) |
| `profile` | 重ねるプロファイル (空は基の設定) |
| `disabled_detectors` | 無効にする判定のインスタンスの名前 (カンマ区切り。プロファイルの上に重ねる) |
| `weight.<インスタンス>` | 総合での重み (負なら設定の値) |

`profile`・`disabled_detectors`・`weight.*` は走行中に変えられる (`ros2 param set`)。知らない名前は拒否する。プロファイルと無効化を変えると、判定・総合・状態遷移を作り直す (状態は `NORMAL` から)。重みは作り直さずに効く。

設定ファイルは起動時にすべて検証する (プロファイルも)。プロファイルで情報源 (`sources`) は変えられない (入力のアダプタは起動時に基の設定で作るため)。

## 入出力

| 種類 | 名前 | 型 | 内容 |
| :--- | :--- | :--- | :--- |
| 購読 | 設定の `sources.<名前>.input` | アダプタごと | 情報源の標本 ([plugins.md](./plugins.md#入力のアダプタ-sourceadapter)) |
| TF | アダプタが引く座標 | | センサの取り付け位置など |
| 購読 | 設定の `external_reinit.topics` | `geometry_msgs/PoseWithCovarianceStamped` | 外からの再初期化 (下記) |
| 配信 | 設定の `output.status_topic` | `localization_supervisor_msgs/LocalizationStatus` | 周期ごとの状態 |
| サービス | `~/set_profile` | `localization_supervisor_msgs/SetProfile` | プロファイルの切り替え (`profile` パラメータを変えるのと同じ) |

## 状態のメッセージ

`LocalizationStatus` の各フィールドの意味は `.msg` のコメントを参照。中身は `core/status.py` で周期の結果から作る (監督以外の監視も、同じ形で出せる)。

- `blame` / `contaminated`: 疑わしい独立の情報源と、汚れていると見なす融合の結果 ([design.md](./design.md#疑わしい情報源の切り分け-blamecontaminated))
- `checks`: 有効な判定のインスタンスごとの状態・`score`・累積・記録用の値
- `footprint_in_lethal` / `lethal_depth_m`: 安全の信号 ([design.md](./design.md#安全の信号))。設定に `safety` が無いか、地図・姿勢が無いときは、`false` / `-1`

## 想定外への対処

- **外からの再初期化**: `external_reinit.topics` に届いたら、人やほかのノードによる上書きとして扱う。状態を `NORMAL` に戻し、累積・判定の内部状態・アンカーを捨てて、`grace.after_reset_sec` の間は総合しない。イベント `<prefix>.external_reinit`。
- **地図の切り替え**: 判定が使う地図が替わったら (最初に届いたときは除く)、同じように捨てて猶予を置く。イベント `<prefix>.map_changed`。
- **時刻**: 周期の時刻はノードの時計 (`use_sim_time` に従う)。標本の時刻はメッセージの header。

## 診断とイベント

- 診断 `<ノード名>: <output.diagnostic_name>` (ハードウェア ID は `output.hardware_id`)。`NORMAL` は OK、`DEGRADED` は ERROR、それ以外は WARN。値は `blame`・`fusion_level`・`profile`・`last_event` などと、判定のインスタンスごとの `check.<名前>` (`<状態> <score>`)。
- イベントは `events.sink` のプラグインで出す (標準は logfmt の 1 行)。名前の前置きは `events.prefix`。

| イベント | レベル | 値 |
| :--- | :--- | :--- |
| `node.started` | info | `name`、`config`、`profile`、`detectors` |
| `<prefix>.state_changed` | 状態による (`NORMAL` へは info、`DEGRADED` へは error、ほかは warn) | `from`、`to`、`reason`、`blame`、`fusion_level` |
| `<prefix>.external_reinit` | warn | `topic`、`state` (そのときの状態) |
| `<prefix>.map_changed` | info | `sources` |
| `<prefix>.lethal` | 入ったら warn、出たら info | `in_lethal`、`fraction`、`depth_m` |
| `<prefix>.profile_changed` | info | `from`、`to` |
| `<prefix>.detectors_changed` | info | `disabled` |
