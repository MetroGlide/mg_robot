# mg_diagnostics

状態監視の「集める」層。各ノードが出す診断 (`/diagnostics`) を集約して `/diagnostics_agg` に出し、自分で診断を出せないノードを外から調べ、イベントログを `/events` に中継する。
どの状態をどのノードが判定するか、診断・イベントの書式は [doc/monitoring.md](../doc/monitoring.md)。Web UI は、`/diagnostics_agg` と `/events` を表示する ([mg_ui](../mg_ui/README.md))。

スタック (`make navigation`、`make slam`、`make slam-gnss-2d`) の bringup (`mg_bringup/launch/common/bringup_common.launch.py`) が、`monitoring.launch.py` を起動する。専用のコンテナは無い。

## ノード

| 実行ファイル | ノード名 | 内容 |
| :--- | :--- | :--- |
| `aggregator_node` (`diagnostic_aggregator`) | `analyzers` | `/diagnostics` を `/Robot/<区分>/...` にまとめて `/diagnostics_agg` に出す。届かなくなった診断や、必須なのに欠けている診断を `STALE` にする |
| `scripts/topic_monitor_node.py` | `topic_monitor_node` | 自分で診断を出せないノード (Nav2 など) のトピックの受信頻度と、ノードの生存を診断として出す |
| `scripts/event_relay_node.py` | `event_relay_node` | イベントログを `/events` に中継し、診断のレベルの変化を `health.level_changed` のイベントにする |

どのノードも止めても、ロボットの動作は変わらない (回復の機能は診断に依存しない)。

## launch

`launch/monitoring.launch.py`。

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `simulation` | `$SIMULATION` (無ければ `false`) | `true` なら、実機のドライバの診断 (`drivers.yaml`) を集約しない |
| `use_aggregator` | `true` | 診断を `/diagnostics_agg` に集約する。止めても `/diagnostics` の生の値は残る |
| `analyzers` | `params/analyzers/navigation.yaml` | スタックごとの集約の設定 (共通の `common.yaml` と一緒に渡される) |
| `use_topic_monitor` | `true` | `topic_monitor_node` を起動する |
| `topic_monitor_params` | `params/topic_monitor/navigation.yaml` | `topic_monitor_node` のパラメータ (スタックごとに指定する) |
| `use_event_relay` | `true` | `event_relay_node` を起動する |

`bringup_common.launch.py` は、同じ名前の引数 (`analyzers`、`use_topic_monitor`、`topic_monitor_params`) と `use_monitoring` を受け取る。SLAM のスタックは、`analyzers` に `mapping.yaml` を渡し、`use_topic_monitor` を `false` にする。

## 集約の設定 (`params/analyzers/`)

| ファイル | 内容 | 渡す条件 |
| :--- | :--- | :--- |
| `common.yaml` | 根のパス (`Robot`)、`System` (PC の資源)、捨てる第三者の診断 | 常に |
| `drivers.yaml` | `Drivers` (LiDAR・Motor・Odometry・GNSS)。必須の診断 (`expected`) を含む | 実機のみ |
| `navigation.yaml` | `Localization` (Monitor・GnssBridge・Gate・EKF)、`Navigation` (Nav2・Sequencer・Nodes・Topics) | ナビゲーションのスタック |
| `mapping.yaml` | (今は空) | SLAM のスタック |

- 照合は、診断の名前 (`<ノード名>: <タスク名>`) の `startswith` / `regex` で行う。
- 集約のパスの規約と、UI が参照するパスは [doc/monitoring.md](../doc/monitoring.md)。
- 診断の名前や担当を変えたら、この設定も合わせて変える。名前が合わないものは `/Robot/Other` に出る (異常とはしない)。
- 常に `WARN` / `ERROR` になる第三者の診断は、`common.yaml` の `discard_third_party` で捨てる。捨てた診断も、生の `/diagnostics` と rosbag には残る。**捨てる診断に一致する `startswith` を、他の analyzer に書かない** (照合の順序に頼らず、`regex` で限定する)。
- 実機では、`drivers.yaml` の `expected` の診断が欠けると、その項目が `STALE` ("Missing") になる。シミュレーションでは `drivers.yaml` を渡さない。

## topic_monitor_node

自作のノードは、自分の状態を自分の診断として出す。ここでは、自分で診断を出せないノードだけを外から調べる。

| 種類 | 内容 | レベル |
| :--- | :--- | :--- |
| ノードの生存 (`node /<名前空間>/<名前>`) | ROS のグラフにいるか。名前空間を含めた形で `monitored_nodes` に書く。起動直後は `startup_grace_sec` のあいだ待つ | いなければ `node_fault_level` (既定 `error`) |
| トピックの受信頻度 (`topic <トピック>`) | `expected_hz` × `hz_warn_ratio` を下回るか、途絶したか | 下回れば `WARN`、途絶は `fault_level` |

トピックは `monitored_sources` に ID を並べ、ID ごとに設定を書く (書き方は `mg_utils.monitoring.topic_source_config`、[mg_utils の README](../mg_utils/README.md))。コードの変更は要らない。

| パラメータ | 既定値 | 内容 |
| :--- | :--- | :--- |
| `monitored_nodes` | なし | 生存を調べるノード (`/ns/name`) |
| `monitored_sources` | なし | 受信頻度を調べるトピックの ID |
| `hz_warn_ratio` | `0.5` | 期待する周期に対する、警告を出す割合 |
| `stale_timeout_sec` | `3.0` | 途絶とみなす時間 (ソースごとに上書きできる)。期待する周期より長くする |
| `startup_grace_sec` | `30.0` | 起動直後に、ノードやメッセージが無くても異常にしない時間 |
| `recover_hold_sec` | `2.0` | 異常から戻るまでに、正常が続く必要がある時間 |
| `node_fault_level` | `error` | ノードがいないときのレベル (`warn` / `error`) |

現在の設定は `params/topic_monitor/navigation.yaml`。LiDAR・odom・モータの途絶は `driver_watchdog_node`、シーケンサの進捗は `waypoint_sequencer_node` が自分の診断として出すので、ここでは調べない。

## event_relay_node

- `/rosout` のうち本文が `evt=` で始まるものを、そのまま `/events` (`rcl_interfaces/Log`、reliable・transient_local、履歴 `history_depth` 件) に流す。
- `/diagnostics_agg` の元の診断 (`<ノード名>: <タスク名>`) のレベルが変わったら、`evt=health.level_changed path=... from=... to=... message=...` をログに出す。これも `/rosout` を経由して `/events` に流れる。レベルが `OK` なら INFO、`WARN` なら WARN、それ以外は ERROR。
- 最初の `startup_grace_sec` (既定 30 秒) のあいだは、変化を覚えるだけでイベントにしない。猶予のあとに初めて見た診断は、`OK` 以外のときだけ `from=NONE` として出す。

| パラメータ | 既定値 | 内容 |
| :--- | :--- | :--- |
| `startup_grace_sec` | `30.0` | 起動直後に、レベルの変化をイベントにしない時間 |
| `history_depth` | `500` | `/events` に残す履歴の件数 |
| `diagnostics_topic` | `/diagnostics_agg` | レベルの変化を見る診断 |

後から `/events` を購読する側は、購読側の深さの分だけ履歴を受け取る。多く受け取るには、購読側の深さを大きくする (`foxglove_bridge` は `max_qos_depth`)。

## 依存

`diagnostic_aggregator`、`diagnostic_updater`、`diagnostic_msgs`、`rcl_interfaces`、`mg_utils` (判定エンジン・イベントログ)。監視するトピックの型 (`nav2_msgs`、`mg_msgs` など) は、設定で指定したものを実行時に読み込む。

`diagnostic_aggregator` は、イメージに入っていないので、`make build` でイメージを作り直す必要がある (`package.xml` の依存から入る)。

## テスト

このパッケージ自体のテストは無い。判定は `mg_utils` のテスト (`make test pkg=mg_utils`) で調べる。集約の設定は、`diagnostic_aggregator` を入れた環境で、サンプルの診断を流して確かめる。
