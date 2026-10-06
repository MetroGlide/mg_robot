# mg_diagnostics

ノードの生存とトピックの配信周期を監視して、`/diagnostics` (`diagnostic_msgs/DiagnosticArray`) に 1 Hz で配信する。
Web UI のトップページとシステムページが、この配信を購読して表示する ([mg_ui](../mg_ui/README.md))。

## ノード

| 実行ファイル | 内容 |
| :--- | :--- |
| `diagnostics_node.py` (ノード名 `diagnostics_node`) | 監視対象のノードとトピックを調べ、結果を `/diagnostics` に配信する |

## 監視の内容

### ノードの生存 (`node/<ノード名>`)

`monitored_nodes` に指定したノード名が、ROS のグラフにあるかを調べる。ある場合は `OK` (`alive`)、無い場合は `WARN` (`not found`)。

### トピックの周期 (`topic/<トピック名>`)

直近 10 メッセージの間隔から周期 [Hz] を計り、期待する周期と比べる。監視するトピックと期待周期は、`params/diagnostics.yaml` の `monitored_topics` で決める (「監視するトピックの設定」を参照)。現在の設定は次のとおり。

| トピック | 型 | 期待周期 [Hz] |
| :--- | :--- | :--- |
| `waypoint_sequencer_node/status` | `mg_msgs/SequencerStatus` | 10 |
| `amcl_pose` | `PoseWithCovarianceStamped` | 1 |
| `cmd_vel` | `Twist` | 10 |
| `odom` | `Odometry` | 20 |
| `scan_top_lidar` | `LaserScan` | 10 |
| `motor_driver_node/emergency_stop` | `Bool` | 1 |
| `collision_detector_state` | `nav2_msgs/CollisionDetectorState` | 5 |

診断の `values` には、周期 (`hz`)、期待周期 (`expected_hz`)、分類 (`group`) が入る。Web UI は `group` が `sensor` のものを「センサ」に、それ以外を「トピック」に出す。

判定は次のとおり。いずれも `OK` / `WARN` の 2 段階で、`ERROR` は出さない。

| 条件 | レベル | メッセージ |
| :--- | :--- | :--- |
| 周期 ≥ 期待周期 × `hz_warn_ratio` | `OK` | `<周期> Hz` |
| 0 < 周期 < 期待周期 × `hz_warn_ratio` | `WARN` | `low rate: ...` |
| メッセージが来ていない (周期 0) | `WARN` | `no data` |

トピック名は相対名なので、ノードの名前空間の下で解決される。

## launch

`launch/diagnostics.launch.py` (compose サービス `diagnostics`、`make diagnostics`)。

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `params_file` | `params/diagnostics.yaml` | パラメータのファイル |

## パラメータ

`params/diagnostics.yaml` の値 (launch はこのファイルを読む)。

| パラメータ | yaml の値 | コードの既定値 | 内容 |
| :--- | :--- | :--- | :--- |
| `monitored_nodes` | `waypoint_sequencer_node`、`amcl`、`bt_navigator`、`controller_server`、`planner_server` | 上の 5 つに `collision_monitor`、`motor_driver_node` を加えた 7 つ | 生存を調べるノード名 |
| `hz_warn_ratio` | `0.5` | `0.5` | 期待周期に対する、警告を出す割合 |
| `monitored_topics` | 上の 7 件の ID | なし (必須) | 周期を監視するトピックの ID。設定の書き方は下の節 |

- yaml の `monitored_nodes` が使われるため、`collision_monitor` と `motor_driver_node` は、yaml に足さない限り監視されない。

## 監視するトピックの設定

`monitored_topics` に ID を並べ、ID ごとに同じ名前のキーで設定を書く。コードの変更は要らない。

```yaml
monitored_topics: [scan_top_lidar]
scan_top_lidar:
  topic: scan_top_lidar              # トピック名 (相対名はノードの名前空間で解決)
  type: sensor_msgs/msg/LaserScan    # メッセージの型
  expected_hz: 10.0                  # 期待する周期 [Hz]
  qos: best_effort                   # best_effort / reliable (省略時 reliable)
  group: sensor                      # sensor / topic (省略時 topic)
```

- `topic`・`type`・`expected_hz` は必須。欠けているとノードは起動時に落ちる。
- `qos` と `group` に決められた値以外を書いても、起動時に落ちる。
- `type` は、起動した環境で解決できる型にする (`rosidl_runtime_py` で読み込む)。
- トピックを外すときは、`monitored_topics` から ID を消す。

## 依存

`diagnostic_msgs`、`rosidl_runtime_py`。監視するトピックの型 (`nav2_msgs`、`mg_msgs` など) は、yaml で指定したものを実行時に読み込む。テストはない。
