# nav2_pkg

Nav2 (ROS 2 Humble) のパッケージのうち、**リポジトリに取り込んで、手を加えているもの**。apt の同名のパッケージの代わりに、ワークスペースでビルドして使う。
ルートの [AGENTS.md](../AGENTS.md) では「カスタム修正済み」としている。

| パッケージ | `package.xml` のバージョン | 使うもの |
| :--- | :--- | :--- |
| `nav2_collision_monitor` | 1.2.0 | 衝突の防止 (`collision_monitor`) と検知 (`collision_detector`)。[mg_navigation](../mg_navigation/doc/nav2_config.md#衝突の防止-collision_monitor) |
| `nav2_costmap_2d` | 1.1.20 | コストマップ (レイヤー、フィルタ) |
| `nav2_msgs` | 1.1.19 | Nav2 のメッセージ・サービス・アクション |

各パッケージの `README.md` と `CHANGELOG.rst` は、上流 (Nav2) のものがそのまま入っていて、このリポジトリでの変更は書かれていない。

## 取り込みと変更の履歴

このリポジトリでの変更は、git の履歴だけに残っている (`git log -- nav2_pkg`)。上流との差分の全体は、確認していない。

| コミット | 日付 | 内容 |
| :--- | :--- | :--- |
| `fc65fa9` | 2023-10-26 | 新しい Nav2 の `nav2_collision_monitor` と、それが使う `nav2_msgs` を取り込む (`CollisionMonitorState`、`CollisionDetectorState`、`state_topic` を含む) |
| `959982b` | 2023-10-29 | `nav2_msgs` の action と msg の一部を、削除・変更する (`BackUp`、`Spin`、`FollowPath`、`NavigateToPose` など) |
| `7f53f6d` | 2025-10-17 | `nav2_msgs` に、`Route`、`RouteNode`、`RouteEdge`、`EdgeCost` の msg、`DynamicEdges`、`SetInitialPose`、`SetRouteGraph` の srv、`ComputeRoute`、`ComputeAndTrackRoute` の action を足す。バージョンを 1.1.12 から 1.1.19 に上げる |
| `9f4ad5b` | 2025-11-24 | `nav2_costmap_2d` (1.1.20) 全体をコピーして取り込む (コミットメッセージ: 「SpeedFilter をローカルにコピーしてワーニングを抑制」) |

## このリポジトリが依存しているもの

| 使う側 | 使うもの |
| :--- | :--- |
| `mg_navigation` | `collision_monitor`・`collision_detector` (`navigation_launch.py`、`nav2_params.yaml`)、`nav2_costmap_2d` のレイヤーとフィルタ、`nav2_msgs` |
| `mg_diagnostics` | `nav2_msgs/CollisionDetectorState` |
| `mg_scenario_test` | `nav2_msgs/CollisionMonitorState` (`collision_monitor_action` の monitor。[mg_plugins.md](../mg_scenario_test/doc/mg_plugins.md)) |
| `mg_waypoint_navigation`、`sim_scenario_test` | `nav2_msgs` |

## ビルド

`nav2_pkg/` の 3 つのパッケージは、他のパッケージと同じように、`colcon build` でビルドされる (Docker の `runtime` ステージ。[doc/docker.md](../doc/docker.md))。`package.xml` だけが、`docker/collect_deps.sh` で依存の解決に使われる。
ワークスペースの overlay にあるので、`source /root/ros2_ws/install/setup.bash` したあとは、apt の同名のパッケージより優先される。

## 変更・更新するとき

- 上流を更新するときは、まず上流との差分を取り、このリポジトリでの変更 (上の履歴) を確認する。
- `nav2_msgs` の型を変えると、`mg_navigation`、`mg_diagnostics`、`mg_scenario_test`、`mg_waypoint_navigation` に影響する。変えたあとは、`make test` と `make scenario-test-all TIER=smoke` で確認する。
- 変更したときは、この README の「取り込みと変更の履歴」に足す。
