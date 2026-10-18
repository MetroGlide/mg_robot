# 自己位置推定

ホイールオドメトリ・AMCL・GNSS を EKF (`robot_localization` の `ekf_global_node`、world フレーム = `map`) で融合して、`map→odom` を作る。
launch は `mg_bringup` の `bringup_navigation.launch.py` (EKF と GNSS ブリッジ) と、`mg_navigation` の `localization_launch.py` (map_server、AMCL、watchdog)。`localization_launch.py` は、調停・初期化・監督ノードを起動する `mg_localization_supervisor` の `supervisor.launch.py` を include する。

## 構成

```
wheel_odometry_node ─ /odom/raw ─► wheel_odom_corrector_node ─ /odom ─► odometry_tf_broadcaster (odom→base_footprint)
   (mg_drivers)                       (mg_drivers。スケール・バイアス補正)          │
                                                                                 ▼ 速度 (vx, vy, vyaw)
/scan_top_lidar ─► amcl ─ /amcl_pose_origin ─► amcl_publish_controller_node ─ /amcl_pose ─► ekf_global_node ─► map→odom TF
                     ▲                              ▲ SetBool で入/切                  ▲ 位置・yaw
                     │ /initialpose                 │                                  │
                     │                              │           /odom/gps ─────────────┘ 位置 (yaw は使わない)
gnss_amcl_initializer_node ◄─ /odom/gps ◄─ slam_gnss_nav_bridge ◄─ /navpvt (u-blox NavPVT)
                     ▲                                (slam_gnss_2d)
                     │ request_reinit
              amcl_watchdog_node   または   localization_supervisor_node
```

EKF の周期は `mg_drivers/params/ekf_global.yaml` の `frequency`。

| ノード | 役割 |
| :--- | :--- |
| `amcl` | LiDAR と地図による自己位置推定。出力は `/amcl_pose_origin` (TF は出さない) |
| `amcl_gate_arbiter` | AMCL の入/切を、要求元ごとに調停する ([mg_localization_supervisor](../../mg_localization_supervisor/README.md#amcl-のゲートの調停-amcl_gate_arbiter))。ゲートを操作するのは、このノードだけ |
| `amcl_publish_controller_node` | `/amcl_pose_origin` を `/amcl_pose` へ中継する (`mg_drivers` の `generic_publish_controller_node.py`)。SetBool で止めると、AMCL が EKF に入らない |
| `slam_gnss_nav_bridge` | `/navpvt` を、`gnss_transform.yaml` で map 座標に直して `/odom/gps` に出す。アンテナの位置を車体の中心に補正し、測位の質に応じた分散を付ける ([slam_gnss_2d](../../slam_gnss_2d/README.md)) |
| `gnss_amcl_initializer_node` | 起動時 (と再初期化の要求時) に、精度の良い `/odom/gps` から `/initialpose` (AMCL) と `/set_pose` (EKF) を出して初期化する ([mg_localization_supervisor](../../mg_localization_supervisor/README.md#gnss_amcl_initializer_node)) |
| `amcl_watchdog_node` | AMCL の共分散が大きい状態が続いたら、`gnss_amcl_initializer_node` に再初期化を要求する ([下記](#amcl_watchdog_node)) |
| `localization_supervisor_node` | AMCL・GNSS・EKF・スキャンと地図を、設定で組み合わせた判定で見比べ、自己位置のずれを検知して通知する。計画用地図の障害物に入っているかも出す ([localization_supervisor](../../localization_supervisor/doc/supervisor_node.md)、このロボットでの設定は [mg_localization_supervisor](../../mg_localization_supervisor/README.md#監督ノード-localization_supervisor_node)) |
| `ekf_global_node` | ホイールオドメトリの速度、AMCL の位置・yaw、GNSS の位置を融合する (`mg_drivers/params/ekf_global.yaml`) |

`gnss_amcl_initializer_node`、`amcl_watchdog_node`、`localization_supervisor_node` は、他のノードが立ち上がるのを待って遅れて起動する (`TimerAction`。初期化ノードを先に、監視ノードを後に起動する)。

## amcl_watchdog_node

AMCL の位置の共分散を監視し、大きい状態が続いたら、`gnss_amcl_initializer_node` に再初期化を要求する。既定の監視 (`localization_monitor:=watchdog`)。
`localization_supervisor:=true` で、監督ノードを並行して通知だけで動かせる (状態は監督ノードが出す)。

- `amcl_pose_origin` (`PoseWithCovarianceStamped`) を購読し、共分散から指標を計算する。
- 指標が `threshold` を超える状態が `consecutive_count` 回続くと、異常とする。
- 異常のとき、`/gnss_amcl_initializer_node/request_reinit` (`Trigger`) を呼ぶ。失敗したら少し待って、最大 `max_retries` 回まで試す。成功したら、`recovery_backoff_sec` の間は、次の再初期化を抑える。
- 実装は `mg_navigation/amcl_watchdog/` (指標の計算は `metrics.py`、状態の対応づけは `health.py`、ノードは `node.py`)。起動のラッパーは `scripts/amcl_watchdog/amcl_watchdog_node.py`。

| パラメータ | 内容 |
| :--- | :--- |
| `metric` | 指標: `trace_xy` (x・y の分散の和)、`determinant_xy` (x・y の共分散行列の行列式)、`max_eigenvalue_xy` (同、最大固有値) |
| `threshold` | 異常とする指標の値 |
| `consecutive_count` | 連続で超える回数 |
| `recovery_backoff_sec` | 再初期化に成功したあと、次を抑える時間 [s] |
| `max_retries` | サービスの呼び出しの再試行の回数 |
| `initializer.call_timeout_sec` | サービスの応答を待つ時間 [s] |
| `min_interval_between_events` | 異常の検知の最短の間隔 [s] |
| `publish_status` | 状態 `/localization/status` を出すか (監督ノードを並行して動かすときは `false`。launch が決める) |

この監視は、AMCL の**自己申告の共分散**しか見ない。そのため、AMCL が「自信を持って間違えている」状態は検知できない。

### 出力

監督ノードと同じ形で、状態を出す。UI や記録は、どちらの監視が動いていても同じに扱える。

- `/localization/status` (`localization_supervisor_msgs/LocalizationStatus`。一定の周期で配信。`publish_status` が `true` のとき): `state` は次のとおり。判定 (`checks`) は共分散の 1 つ (`amcl_covariance`) だけで、`blame` は空、安全の信号は使えない (`lethal_depth_m` が `-1`)。

| 状態 | 条件 |
| :--- | :--- |
| `NORMAL` | 異常を検知していない |
| `SUSPECT` | 指標が `threshold` を超え始めた (`consecutive_count` 回に達する前) |
| `RECOVERING` | 再初期化を要求している |
| `DEGRADED` | 再初期化が `max_retries` 回とも失敗し、指標がまだ戻っていない |

- 診断 `amcl_watchdog_node: localization` (NORMAL=OK、DEGRADED=ERROR、その他=WARN。集約で `/Robot/Localization/Monitor`)
- イベント: `localization.state_changed` (`from`、`to`、`reason`)、`localization.reinit_requested` (`attempt`、`metric`、`value`、`threshold`)、`localization.reinit_result` (`success`、`message`) ([doc/monitoring.md](../../doc/monitoring.md))

## 従来の構成に戻す

- `localization_monitor:=watchdog` にすれば、従来の監視。
- `use_odom_corrector:=false` で、ホイールオドメトリの補正なし ([mg_drivers](../../mg_drivers/doc/wheel_odom_corrector.md))。
- `localization_supervisor:=false` (既定) で、監督ノードを動かさない。
- 監督ノードの負荷は、一定の周期の判定と、間引いたスキャンの照合 (局所地図の照合は数秒おき) だけで、ごく小さい。

## 評価

パラメータの変更や復旧の仕組みは、rosbag と SLAM の出力 (地図・pose_graph) を使って、再生で評価する。実機を使わずに、同じ条件で変種を比べられる。
手順は [tools の README](../../tools/README.md) の `run_localization_variant.sh` と `eval_localization.py` を参照。
