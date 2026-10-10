# 状態監視とイベントログ

ノードの状態を「どこで判定し、どう出し、どう見るか」の規約。新しい監視やログを足すときは、ここに合わせる。
実装の場所は、状態の監視が [mg_diagnostics](../mg_diagnostics/README.md)、共通のライブラリが [mg_utils](../mg_utils/README.md)、表示が [mg_ui](../mg_ui/README.md)。

## 判定の原則

1. **安全のための停止は、データの経路の中で行う。** `collision_monitor` の `stop_on_timeout` や、モータドライバの E-Stop が担う。診断やイベントは関与しない。
2. **判定は、それを最もよく知るノードが 1 回だけ行う。** 各ノードは、自分が判定したことだけを診断として出す。他のノードの判定を出し直さない。
3. **機能の状態のデータは、型付きのトピックで出す。** `LocalizationStatus`、`SequencerStatus`、`GateArbiterState`、`connected`、`emergency_stop`、`navpvt` など。制御と解析ツールは、こちらを使う。
4. **診断とイベントは観測のための出力で、ノード間の制御の入力にしない。** テストの合否判定、記録、表示に使うのはよい。回復の機能 (watchdog の一時停止と再起動、supervisor) は、診断を集約するノードが止まっても動き続ける。
5. **UI は判定しない。** 色は診断のレベルと system_manager が返す `level` で決め、数値は型付きのトピックから取る。UI が自分で判定してよいのは、通信路が新しいかどうかだけ。
6. **第三者のノードの診断は、判定し直さず、出し直さない。** Nav2、EKF、ublox などは、型付きの出力と診断を両方出している。判定の基準が合わないときは、まずそのノードのパラメータで調整する。調整できないときだけ、自作のノードが型付きの出力から判定し、元の診断は `Other` に回して、ここに記録する。

## 診断 (`/diagnostics`)

標準の `diagnostic_updater` で出し、`diagnostic_aggregator` で `/diagnostics_agg` にまとめる。自作のノードは `diagnostic_msgs/DiagnosticArray` を自前で組み立てない。

| 項目 | 規約 |
| :--- | :--- |
| 状態の名前 | `diagnostic_updater` の既定の形 `<ノード名>: <タスク名>`。名前空間はソースに書かない |
| `hardware_id` | 機器の識別子 (`top_lidar`、`motor_driver`、`gnss` など)。パラメータで与える。機器に対応しないものは `mg-robot`。ロボットのバージョン名 (`mg01` など) はソースに書かない |
| `OK` | 正常 |
| `WARN` | 機能は継続しているが、劣化している |
| `ERROR` | 機能が停止した、または対応が必要 |
| `STALE` | aggregator だけが付ける。発信元は出さない |
| `values` のキー | snake_case。単位は接尾辞で付ける (`hz`、`expected_hz`、`stale_sec`、`h_acc_m`) |
| しきい値 | 判定するノードのパラメータ YAML に置く |

第三者のノードの名前と `hardware_id` はそのまま受け入れ、aggregator の設定 (analyzer) の名前照合で吸収する。

### 集約のパス (UI との契約)

aggregator は、最上位を次の区分にまとめる。

| 区分 | 内容 |
| :--- | :--- |
| `/Drivers` | センサ・モータなどの機器とドライバ |
| `/Localization` | 自己位置推定 (AMCL、EKF、監視ノード) |
| `/Navigation` | Nav2、waypoint_sequencer、collision_monitor |
| `/Mapping` | SLAM |
| `/System` | PC の資源 |
| `/Other` | どれにも当てはまらないもの |

UI が個別に参照する項目のパスは、次に固定する。変えるときは、UI の `diagnosticPaths.ts` と一緒に変える。

| パス | 内容 |
| :--- | :--- |
| `/Drivers/GNSS` | GNSS の受信状態 |
| `/Localization/Monitor` | 自己位置推定の監視 (watchdog か supervisor) |
| `/Navigation/Nav2` | Nav2 の lifecycle (`Nav2 Health`) |
| `/System/Resources` | CPU とメモリ |

## イベントログ

状態の遷移、アクションの実行、接続の変化など、離散的に起きる出来事を **イベント** として、ノードのロガーに決まった書式で出す。周期的に出る値はイベントにしない。

### 書式 (logfmt)

メッセージの本文を、次の形にする。

```
evt=<domain>.<event> key=value key="空白を含む値" ...
```

```
[INFO] [1760154000.123] [waypoint_sequencer_node]: evt=sequencer.state_changed from=IDLE to=NAVIGATING index=3 total=10 run_id=20261011T103000
```

- 値に空白、`"`、`=` が入るときは、`"` で囲み、中の `"` と `\` を `\` でエスケープする。
- Humble では、子ロガー (`get_child`) の出力が `/rosout` に流れない。イベントかどうかは、ロガー名ではなく本文の先頭の `evt=` で見分ける。
- ログの時刻 (`{time}`) はエポック秒のまま使う。rosbag と docker の時刻と突き合わせるため。
- 1 回の実行に関係するイベントには、`run_id`、`goal_id`、`index` などの関連づけのキーを付ける。

### レベル

| レベル | 使う場面 |
| :--- | :--- |
| `INFO` | 正常なイベント |
| `WARN` | 異常だが、自動で対処している (再接続中、回復を始めた、自動で一時停止した) |
| `ERROR` | 失敗した、または対応が必要 (アクションの失敗、再起動の失敗、あきらめた) |

周期的なログを `INFO` で出さない。出すなら、変化したときだけにするか、`*_THROTTLE` を使う。例外は `ERROR` で出し、トレースバックを付ける。

### ドメインとイベント名

| ドメイン | イベント名 |
| :--- | :--- |
| `sequencer` | `state_changed`、`run_started`、`run_finished`、`paused`、`resumed`、`waypoint_reached` |
| `action` | `started`、`succeeded`、`failed` |
| `nav` | `goal_sent`、`goal_result` |
| `driver` | `connected`、`disconnected`、`reconnect_attempt`、`estop_changed` |
| `watchdog` | `stale`、`recovered`、`restart_requested`、`restart_failed`、`pause_changed` |
| `localization` | `state_changed`、`reinit_requested`、`reinit_result` |
| `gate` | `changed` |
| `health` | `level_changed` (診断のレベルの変化。中継ノードだけが出す) |
| `node` | `started` |

新しいイベントを足すときは、この表に追記する。

### 出し方

- Python は `mg_utils.event_log.EventLog`、C++ は `mg_utils/event_log.hpp` を使う。
- 汎用のパッケージ (`slam_gnss_2d`) は `mg_utils` に依存しないので、この書式を自分で守る。
- 診断のレベルの変化は、各ノードではなく中継ノードが `/diagnostics_agg` から一括でイベントにする。各ノードが出すのは「何が起きたか」の業務イベントだけにする。

## 記録と保存の役割

| 保存先 | 内容 | 使いどころ |
| :--- | :--- | :--- |
| docker ログ | 全コンテナの標準出力・標準エラー | `make logs`、Web UI のログ表示 |
| `~/ros2_data/logs/<サービス>/<日付>.log` | docker ログを常時ホストに書き出したもの | コンテナを削除したあとの調査 |
| `ROS_LOG_DIR` (`~/ros2_data/ros_log`) | ノードごとの ROS のファイルログ | ノード単位の調査 |
| rosbag | `/diagnostics`、`/diagnostics_agg`、`/rosout`、`/events`、型付きの状態トピック | 走行の記録と同期した解析 |
