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

直近 10 メッセージの間隔から周期 [Hz] を計り、期待する周期と比べる。期待周期のトピックは、コード (`MONITORED_TOPICS`) に固定されている。

| トピック | 型 | 期待周期 [Hz] |
| :--- | :--- | :--- |
| `waypoint_sequencer_node/status` | `mg_msgs/SequencerStatus` | 10 |
| `amcl_pose` | `PoseWithCovarianceStamped` | 1 |
| `cmd_vel` | `Twist` | 10 |
| `odom` | `Odometry` | 20 |
| `scan_top_lidar` | `LaserScan` | 10 |
| `scan_front_lidar` | `LaserScan` | 10 |
| `motor_driver_node/emergency_stop` | `Bool` | 1 |
| `collision_detector_state` | `nav2_msgs/CollisionDetectorState` | 5 |

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

- yaml の `monitored_nodes` が使われるため、`collision_monitor` と `motor_driver_node` は、yaml に足さない限り監視されない。
- 監視するトピックと期待周期は、パラメータでは変えられない。変えるときは `MONITORED_TOPICS` を編集する。

## 依存

`diagnostic_msgs`、`nav2_msgs`、`mg_msgs`。テストはない。
