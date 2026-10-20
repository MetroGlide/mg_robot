# localization_supervisor_node

設定ファイルに書いた情報源・判定・総合・状態遷移・復旧の組み合わせで、自己位置推定が地図やほかの情報源と合っているかを周期ごとに確かめ、状態・診断・イベントを出す。`state_machine.recovery_enabled` が `true` なら、異常を確認したときに切り離し・再初期化して復旧する。
判定と状態遷移と復旧の計画は中核 (`core/`、[design.md](./design.md)) にあり、ノード (`ros/supervisor_node.py`) は、入力を標本にして中核に渡し、結果を出して、中核の指示を再初期化の送り先と対処に渡すだけにする。

## パラメータ (ROS)

| 名前 | 内容 |
| :--- | :--- |
| `config_file` | 設定ファイルのパス (必須。[parameters.md](./parameters.md)) |
| `profile` | 重ねるプロファイル (空は基本設定) |

`profile` は走行中に変えられる (`ros2 param set`)。知らない名前は拒否する。変えると、判定・総合・状態遷移・復旧を作り直す (状態は `NORMAL` から)。

設定ファイルは起動時にすべて検証する (プロファイルも)。入力のアダプタ・再初期化の送り先・対処は起動時に基本設定で作るので、プロファイルで `sources`・`recovery.resetters`・`recovery.actuators` は変えられない。`recovery_enabled` にした設定 (プロファイルを含む) には、`recovery.resetters` が要る。

## 入出力

| 種類 | 名前 | 型 | 内容 |
| :--- | :--- | :--- | :--- |
| 購読 | 設定の `sources.<名前>.input` | アダプタごと | 情報源の標本 ([plugins.md](./plugins.md#入力のアダプタ-sourceadapter)) |
| TF | アダプタが引く座標 | | センサの取り付け位置など |
| 購読 | 設定の `external_reinit.topics` | `geometry_msgs/PoseWithCovarianceStamped` | 外からの再初期化 (下記) |
| 配信 | 設定の `output.status_topic` | `localization_supervisor_msgs/LocalizationStatus` | 周期ごとの状態 |
| 配信 | 設定の `recovery.resetters` | 送り先ごと | 選んだ姿勢 ([plugins.md](./plugins.md#再初期化の送り先-poseresetter)) |
| 配信・サービス | 設定の `recovery.actuators` | 対処ごと | ゲートの開閉、一時停止の要求など ([plugins.md](./plugins.md#対処-actuator)) |
| サービス | `~/set_profile` | `localization_supervisor_msgs/SetProfile` | プロファイルの切り替え (`profile` パラメータを変えるのと同じ) |

## 状態のメッセージ

`LocalizationStatus` の各フィールドの意味は `.msg` のコメントを参照。中身は `core/status.py` で周期の結果から作る (監督以外の監視も、同じ形で出せる)。

- `blame` / `contaminated`: 疑わしい独立の情報源と、汚れていると見なす融合の結果 ([design.md](./design.md#疑わしい情報源の切り分け-blamecontaminated))
- `checks`: 有効な判定のインスタンスごとの状態・`score`・累積・記録用の値
- `hold_requested`: 切り離してから戻すまで、走行を止めてほしい
- `footprint_in_lethal` / `lethal_depth_m`: 安全の信号 ([design.md](./design.md#安全の信号))。設定に `safety` が無いか、地図・姿勢が無いときは、`false` / `-1`

## 復旧

異常を確認すると (`recovery_enabled`)、状態が `ISOLATED` になり、対処が切り離す (`gate` など) と同時に、`hold_requested` で走行を止めてほしいと知らせる (`pause_request` など)。`isolate_hold_sec` 待って `RECOVERING` になり、復旧の計画を `retry_sec` ごとに繰り返す。
候補が選べたら、イベント `<prefix>.reinit_requested` を出して、再初期化の送り先にすべて送る。収束 (`recovery.convergence`) を確認したら `NORMAL` に戻り、切り離しを戻す。
候補を選べない (`<prefix>.reinit_skipped` の `reason`)、または収束しない時間切れが続くと `DEGRADED` になる (走行を止めてほしいまま、一定の間隔で再び試す)。
`blame` が `state_machine.isolate_only` の情報源のときは、`ISOLATED` のまま、走行を止めず、再初期化もしない (対処 `variance_scale` が、その情報源の分散を上限にするだけ)。異常がなくなれば `NORMAL` に戻る。
対処は起動時に作るので、プロファイルでは変えられない。プロファイルを切り替えると、監督は作り直されて `NORMAL` から始まり、対処は次の周期の結果に従って働きかけを戻す (一時停止を解く、ゲートを開けるなど)。

- **監督自身の再初期化**: 送った再初期化のメッセージは、トピックとヘッダの時刻で見分けて、外からの再初期化として扱わない。
- **戻し方**: `recovery_enabled: false` にする (通知だけ)、プロファイルを替える、対処の設定を外す。

## 想定外への対処

- **外からの再初期化**: `external_reinit.topics` に、監督以外から届いたら、人やほかのノードによる上書きとして扱う。状態を `NORMAL` に戻し (復旧は中断)、累積・判定の内部状態・アンカーを捨てて、`grace.after_reset_sec` の間は総合しない。イベント `<prefix>.external_reinit`。
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
| `<prefix>.check_changed` | info | 判定のインスタンスが正常と異常の間で変わったとき(判定できない周期は飛ばす): `check`、`to` (`normal` / `anomaly`)、`score`、判定の値 `v_<名前>` |
| `<prefix>.residual_changed` | 大きくなったら warn、戻ったら info | 残差から求めた分散の倍率が `notify_scale` 以上になった・戻った: `residual`、`high`、`scale`、`mean_d2`、`count` |
| `<prefix>.recovery_candidate` | info | 計画ごと・候補ごとに `origin`、`x`、`y`、`yaw`、`total`、`vetoed_by`、評価ごとの `score_<名前>` |
| `<prefix>.reinit_requested` | warn | 選んだ候補の `origin`、`x`、`y`、`yaw`、`reason`、`margin`、`blame`、`attempt`、`candidates`、`elapsed_ms`、`generators` (候補を作るものごとの結果の理由) |
| `<prefix>.reinit_skipped` | warn | 選べなかった `reason` (`no_candidates`・`all_vetoed`・`low_score`・`ambiguous`)、`attempt`、`candidates`、`elapsed_ms`、`generators` |
