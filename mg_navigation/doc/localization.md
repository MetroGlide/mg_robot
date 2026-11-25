# 自己位置推定

ホイールオドメトリ・AMCL・GNSS を EKF (`robot_localization` の `ekf_global_node`、world フレーム = `map`) で融合して、`map→odom` を作る。
launch は `mg_bringup` の `bringup_navigation.launch.py` (EKF と GNSS ブリッジ) と、`mg_navigation` の `localization_launch.py` (map_server、AMCL、watchdog)。`localization_launch.py` は、調停・初期化・監督ノードを起動する `mg_localization_supervisor` の `supervisor.launch.py` を include する。
目指す最終形 (置き直しを監督にまとめる、構成の切り替えなど、一部未実装) は [localization_target.md](./localization_target.md)。

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

`localization_monitor:=supervisor` と `localization_gate:=true` では、監督ノードの中で門を動かす。門は、AMCL の動きがオドメトリと合わない、または GNSS と離れたままのとき、ゲートの調停を通して AMCL を EKF から切り離し、地図との照合で決めてから `/initialpose` と `/set_pose` で置き直す ([localization_supervisor](../../localization_supervisor/doc/motion_gate.md))。置き直し・AMCL の門・一時停止・起動の直後の姿勢 (GNSS の周りの全周の照合) を監督ノードが 1 か所で決め、`gnss_amcl_initializer_node` と `manual_pose_relay` は起動しない ([mg_localization_supervisor](../../mg_localization_supervisor/README.md#監督ノード-localization_supervisor_node))。

EKF の周期は `mg_drivers/params/ekf_global.yaml` の `frequency`。

| ノード | 役割 |
| :--- | :--- |
| `amcl` | LiDAR と地図による自己位置推定。出力は `/amcl_pose_origin` (TF は出さない) |
| `amcl_gate_arbiter` | AMCL の入/切を、要求元ごとに調停する ([mg_localization_supervisor](../../mg_localization_supervisor/README.md#amcl-のゲートの調停-amcl_gate_arbiter))。ゲートを操作するのは、このノードだけ |
| `amcl_publish_controller_node` | `/amcl_pose_origin` を `/amcl_pose` へ中継する (`mg_drivers` の `generic_publish_controller_node.py`)。SetBool で止めると、AMCL が EKF に入らない |
| `slam_gnss_nav_bridge` | `/navpvt` を、`gnss_transform.yaml` で map 座標に直して `/odom/gps` に出す。アンテナの位置を車体の中心に補正し、測位の質に応じた分散を付ける ([slam_gnss_2d](../../slam_gnss_2d/README.md)) |
| `gnss_amcl_initializer_node` | 起動時 (と再初期化の要求時) に、精度の良い `/odom/gps` から `/initialpose` (AMCL) と `/set_pose` (EKF) を出して初期化する。`config/initializer.yaml` の `verify_by_scan` が `true` なら、走りながら GNSS の周りをスキャンと地図で照合し、合った姿勢で決める。外から `/initialpose` を与えられたら、それに従う ([mg_localization_supervisor](../../mg_localization_supervisor/README.md#gnss_amcl_initializer_node)) |
| `amcl_watchdog_node` | AMCL の共分散が大きい状態が続いたら、`gnss_amcl_initializer_node` に再初期化を要求する ([下記](#amcl_watchdog_node)) |
| `localization_supervisor_node` | AMCL・GNSS・EKF・スキャンと地図を、設定で組み合わせた判定で見比べ、自己位置のずれを検知して、切り離し・再初期化で復旧する(既定の監視)。計画用地図の障害物に入っているかも出す ([localization_supervisor](../../localization_supervisor/doc/supervisor_node.md)、このロボットでの設定は [mg_localization_supervisor](../../mg_localization_supervisor/README.md#監督ノード-localization_supervisor_node)) |
| `manual_pose_relay` | 監督ノードの中の門がないとき、Web UI の手動の初期姿勢 (`/localization/manual_initial_pose`) を、そのまま `/initialpose` へ送る ([手動の初期姿勢](#手動の初期姿勢)) |
| `ekf_global_node` | ホイールオドメトリの速度、AMCL の位置・yaw、GNSS の位置を融合する (`mg_drivers/params/ekf_global.yaml`) |

`gnss_amcl_initializer_node`、`amcl_watchdog_node`、`localization_supervisor_node` は、他のノードが立ち上がるのを待って遅れて起動する (`TimerAction`。初期化ノードを先に、監視ノードを後に起動する)。

## 手動の初期姿勢

- Web UI の初期姿勢の指定は `/localization/manual_initial_pose` に送る。門があれば、門が確かめてから置き直す。なければ、中継がそのまま `/initialpose` へ送る (従来と同じ)。
- RViz の 2D Pose Estimate は、従来どおり `/initialpose` に送る。門があるときは、AMCL の振る舞いとして門の判定を受ける。
- 詳細は [mg_localization_supervisor](../../mg_localization_supervisor/README.md#手動の初期姿勢)。

## amcl_watchdog_node

AMCL の位置の共分散を監視し、大きい状態が続いたら、`gnss_amcl_initializer_node` に再初期化を要求する。従来の監視 (`localization_monitor:=watchdog`)。
`localization_supervisor:=true` で、監督ノードを並行して通知だけで動かせる (状態は監督ノードが出す。プロファイル `notify_only` になるので、二重に再初期化しない)。

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

## 自己位置推定の構成

監視・監督の並走・門・プリセット・プロファイルの組を、構成の名前 1 つで選ぶ (`bringup_navigation.launch.py` の `localization_config`、環境変数 `LOCALIZATION_CONFIG`)。組は `mg_navigation/mg_navigation/localization_config.py` が正。個別の引数 (環境変数) は、空なら構成の値を使い、書けばそちらが優先する (プリセット・プロファイルを外すときは `none`)。

| 構成 | 中身 | 使い方 |
| :--- | :--- | :--- |
| `legacy` | watchdog だけ (タグ v2.1 の構成) | 戻すルート |
| `observe` | `legacy` で走り、監督ノードと門を判定と照合だけで並走させる (門は開けたまま、置き直しも停止もしない) | 実機で誤った判定の率を測る |
| `supervised` | 監督 (門を入れる)、プリセット `robust`、監督のプロファイル `robust_recovery` | 既定。最新の構成 |

- 実機では `legacy` → `observe` → `supervised` の順に有効にする。`observe` のイベントでは、門が置き直したはずの姿勢 (`localization.pose_reset` の `applied=false`) と閉じた理由を見る。
- 比べる構成 (シミュレータ・再生) は `legacy` と `supervised` ([台帳](./localization_failure_ledger.md#比べる構成))。

## 自己位置推定のプリセット

`localization_preset:=<名前>` (既定は構成の値。`none` なら何もしない) で、`params/localization_presets/<名前>.yaml` を、AMCL (`nav2_params.yaml`) と EKF (`mg_drivers/params/ekf_global.yaml`) の params の後に重ねる。書いたキーだけが変わる。
プリセットは、監督 (`localization_monitor:=supervisor`) と組で使う。ほかの監視で指定すると、launch が起動の前に落ちる。

| プリセット | 内容 |
| :--- | :--- |
| `robust` | AMCL の動きのノイズを下げ (角での向きの取り違えを減らす)、EKF が外れた AMCL を受け付けないようにする (ゲート)。EKF が AMCL を受け付けなくなったまま戻れなくならないよう、監督が初期化し直す。根拠は [localization_robustness.md](./localization_robustness.md) |

### 実機で試す

プリセット `robust` は構成 `supervised` に入っている。実機の前に、シミュレータで `make scenario-test-all TIER=smoke` と `make scenario-test-all TAGS=localization` が通ることを確かめる。
- 戻し方 (軽い順):
  - 監督の自動復旧だけ止める: `LOCALIZATION_PROFILE=notify_only` (検知と通知は残る)。
  - プリセットを外す: `LOCALIZATION_PRESET=none` (AMCL と EKF が従来の params に戻る)。
  - 監督ごと外す: [従来の構成に戻す](#従来の構成に戻す)。
  - どれも、起動し直すと効く。
- 走らせ方: 角でずれたことのある経路を、オペレータがすぐ止められる状態で走らせる。記録は `record_bag:=true` で残す。
- 見るもの:
  - 監督の状態 `/localization/status` と、止める要求で止まった回数・時間。
  - 角や GNSS の弱い場所での、オペレータから見たずれ。ずれたのに監督が `NORMAL` のままだった時刻と場所は、記録の評価で調べるので控えておく。
  - 旋回や滑りやすい場所で、AMCL の追従が遅れないか (動きのノイズを下げると、オドメトリが大きくずれたときに追いつきにくい)。
  - 負荷: 監督のプロセスの CPU 使用率 (コンテナの中で `top -b -d 5 -n 12 -p $(pgrep -f localization_supervisor_node)`)。重ければ、`mg_localization_supervisor/config/supervision.yaml` から粒子の情報源 `amcl_particles` と、それを使う `amcl_split`・候補 `hypotheses` を外す (粒子は 1 つのメッセージが大きく、受け取るだけで負荷がかかる)。
- 分かっている限界: 止まっているあいだにロボットを押して動かすと、AMCL と GNSS のブリッジは動くまで推定を出さないので、監督が気づくのは走り出してからになり、自動で戻れないことがある。動かしたら、手で初期化し直す。
- 記録 (`params/record_topic_list.txt`) には、監督の入力 (AMCL・EKF・GNSS・オドメトリ・スキャン・地図・粒子) と、監督の状態とイベントが入る。

### 実機の記録を調べる

- 何が起きたか: `make log-events BAG=<記録> DOMAIN=localization TZ=+09:00` で、監督の状態の変化・判定の変化・再初期化を時刻順に並べる ([doc/monitoring.md](../../doc/monitoring.md))。
- 設定を変えたらどうなったか: 記録を監督のオフライン評価にかけ、設定やプロファイルを変えて、判定と状態の時系列を比べる。記録には実際の再初期化の結果が入っているので、`--apply-reset` は付けない。真値がないので、ずれの大きさではなく、走行中に控えた時刻に監督が気づくか、それ以外の時刻に疑いを出さないかで比べる。

  ```bash
  ros2 run localization_supervisor offline_runner.py \
      --config /app/mg_localization_supervisor/config/supervision.yaml [--profile <名前>] \
      --bag <記録> --out <出力先>
  ```

  地図 (`/map`・`/planning_map`) は記録の最初にしか入っていないので、`--start` で途中から始めるときは、`--map map_scan=<map.yaml> --map planning_map=<map.yaml>` で地図を渡す (渡さないと、スキャンの判定が「分からない」のままになる)。
  出力の `timeline.jsonl` (周期ごとの状態と判定の値)・`events.jsonl`・`report.md` は [localization_supervisor の design.md](../../localization_supervisor/doc/design.md#オフライン評価)。
- 角でずれた区間は、SLAM の出力を真値にした再生の評価 ([tools の localization_evaluation.md](../../tools/doc/localization_evaluation.md)) のデータセットに足せる。

## 従来の構成に戻す

- `LOCALIZATION_CONFIG=legacy` (または `localization_config:=legacy`) にすれば、従来の監視 (共分散による再初期化だけ)。
- 監督ノードの中の門だけをやめるときは、`localization_gate:=false`。初期化ノードと中継が起動し、監督は門なしの復旧に戻る。Web UI の手動の初期姿勢は、中継を通して従来どおり `/initialpose` に届く。
- `localization_profile:=gnss_fallback` で、AMCL と GNSS が乖離したら GNSS で引き戻すだけの、以前の動作 (監督ノードのまま)。
- `localization_profile:=notify_only` で、監督ノードは検知と通知だけ (自動復旧しない)。
- `use_odom_corrector:=false` で、ホイールオドメトリの補正なし ([mg_drivers](../../mg_drivers/doc/wheel_odom_corrector.md))。
- 監督ノードの負荷は、一定の周期の判定と、間引いたスキャンの照合 (局所地図の照合は数秒おき) だけで、ごく小さい。復旧の計画は異常を確認したときだけで、1 回あたり数十ミリ秒。

## 評価

パラメータの変更や復旧の仕組みは、rosbag と SLAM の出力 (地図・pose_graph) を使って、再生で評価する。実機を使わずに、同じ条件で変種を比べられる。
手順は [tools の README](../../tools/README.md) の `run_localization_variant.sh` と `eval_localization.py` を参照。
角でのずれについて確かめたことと、判断の根拠は [localization_robustness.md](./localization_robustness.md)。
