# localization_supervisor_node

AMCL・GNSS・EKF・スキャン (と地図) を見比べて、AMCL のずれを検知する。復旧を有効にすると、ずれた AMCL を EKF から切り離し、姿勢を選んで AMCL と EKF を初期化し直す。
AMCL の自己申告の共分散だけを見る監視では、AMCL が「自信を持って間違えている」状態を検知できないため、AMCL とは独立な情報でずれを検知する。

判定 (`core/geometry.py`、`core/grid.py`) と状態遷移 (`core/legacy_machine.py`) は ROS に依存しない。ノードは `ros/supervisor_node.py`。
判定を固定の 4 つで持つ実装で、設定で判定を組み合わせる中核 ([design.md](./design.md)) にはまだ移っていない。

## 入出力

| 種類 | 名前 | 型 | 内容 |
| :--- | :--- | :--- | :--- |
| 購読 | `/amcl_pose_origin` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL の推定 (ゲートを通る前) |
| 購読 | `/odom/gps` | `nav_msgs/Odometry` | map 座標の GNSS の位置と分散 |
| 購読 | `scan_topic` | `sensor_msgs/LaserScan` | 地図との照合に使うスキャン |
| 購読 | `/map` | `nav_msgs/OccupancyGrid` | 照合の基準の地図 (transient_local) |
| 購読 | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 他のノードによる AMCL の初期化 (直後の飛びを異常としない) |
| TF | `map_frame`→`base_frame`、`odom_frame`→`base_frame` | | EKF の姿勢、オドメトリの動き |
| 配信 | `/localization/status` | `localization_supervisor_msgs/LocalizationStatus` | 状態、AMCL を EKF に入れているか、判定の値、直近のできごと |
| 配信 | `/initialpose`、`/set_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 復旧のときの AMCL と EKF の初期化 |
| 配信 | `speed_limit_topic` | `nav2_msgs/SpeedLimit` | `action_on_degraded: slow` のときの速度の制限 |
| クライアント | `amcl_gate_service` | `std_srvs/SetBool` | AMCL の出力を EKF に入れるゲート ([gate_arbiter_node](./gate_arbiter.md) の要求元のサービス) |

## 検知 (一定の周期)

| 判定 | 内容 |
| :--- | :--- |
| `gnss` | 標準偏差 `gnss_max_sigma_m` 以下の GNSS と AMCL の位置の食い違い (マハラノビス距離の二乗。分散は AMCL と GNSS の両方を使うので、単独測位では、数 m 以上のずれを検知する) |
| `jump` | 連続する AMCL の推定が示す `map→odom` が、オドメトリの示す動きから飛んだ (`jump_threshold_m` / `_rad`) |
| `scan` | EKF の姿勢でスキャンを地図に重ね、占有セルから `scan_tolerance_m` 以内の点の割合 (一致率) を見る。**一致率の絶対値は地図や LiDAR で決まる (正しい姿勢でも低いことがある)** ので、正常なときの一致率の中央値の `scan_ratio_factor` 倍 (下限 `scan_ratio_min`) を下回ったら異常とする。姿勢の周り (±`scan_search_range_m`) でずらしたときの一致率の増え方 (利得) も記録する。別走行の地図との位置合わせのずれで正しい姿勢でも利得が出るときは、`scan_gain_threshold` を判定に使わない値にする |
| `diff` | AMCL と EKF の位置が `diff_threshold_m` 以上離れている |

- どの判定も、使えないとき (GNSS の精度が悪すぎる、スキャンが無いなど) は判定しない (異常とも正常とも数えない)。
- 判定の値は `LocalizationStatus` に出る (`amcl_gnss_d2`、`amcl_jump_m`、`amcl_ekf_diff_m`、`scan_match_ratio`、`scan_match_gain`)。
- 起動直後 (`startup_grace_sec`) と復旧直後 (`grace_after_recovery_sec`) は、EKF・AMCL が落ち着くまで判定しない (値の記録だけ行う)。

## 動作モード (`recovery_enabled`)

| 値 | 動作 |
| :--- | :--- |
| `false` | **検知と通知だけ**。異常を確認したら `DEGRADED` になり (`/localization/status` が `DEGRADED`、診断が ERROR)、異常が消えたら `NORMAL` に戻る。AMCL は切り離さない |
| `true` | 下の状態遷移どおり、AMCL を切り離して姿勢を選び、再初期化して戻す (実験的) |

再生評価 (kidnap) で、復旧を有効にすると、選んだ姿勢が誤っていた試行で、監視なしよりも誤差が大きく広がった。AMCL は kidnap の後に自力で戻ることも多く、検知に数秒かかる間の誤差は、復旧では防げない。

## 状態遷移 (`recovery_enabled: true` のとき)

```
NORMAL ─ 異常の疑い ─► SUSPECT ─ 確認 ─► ISOLATED (AMCL を EKF から切り離す)
   ▲                     │ 異常が消える            │ isolate_hold_sec 待つ
   └─────────────────────┘                          ▼
   ▲                                          RECOVERING (姿勢を選んで /initialpose と /set_pose を送る)
   └──────────── 収束を確認 (AMCL を戻す) ◄──────────┤ recover_timeout_sec を超えて max_reinit_attempts 回続く
                                                      ▼
                                                  DEGRADED (通知。degraded_retry_sec ごとに再試行)
```

- **切り離し**: `amcl_gate_service` (SetBool) で、AMCL の出力を EKF に入れるゲートを閉じる。EKF はオドメトリと GNSS だけで動く。
- **復旧の姿勢**: 次の候補のうち、スキャンが地図に最もよく合うものを選ぶ (比べられなければ GNSS を優先)。
  1. EKF の姿勢
  2. **巻き戻し**: 異常が始まる `rollback_margin_sec` 前の `map→odom` に、今のオドメトリの動きを足した姿勢 (AMCL が飛んで EKF が引きずられた場合)
  3. 精度の良い GNSS の位置 + EKF の向き (`reinit_gnss_max_sigma_m` 以下のとき。初期化の広がりには GNSS の標準偏差を使う)
- **初期化**: 選んだ姿勢を `/initialpose` (AMCL) と `/set_pose` (EKF) に送る。初期化した直後の AMCL の飛びは異常としない。
- **DEGRADED**: `action_on_degraded: warn` は通知のみ、`slow` は `speed_limit_topic` で速度を制限する。

## 診断とイベント

- 診断 `<ノード名>: localization` (NORMAL=OK、DEGRADED=ERROR、その他=WARN。`values` に `amcl_attached`、`last_event` と、使えた判定の値)
- イベント (logfmt 書式の 1 行): `localization.state_changed` (`from`、`to`、`reason`)、`localization.reinit_requested` (`source`、`x`、`y`、`yaw`、`scan_match`)

## パラメータ

既定値はノードの実装 (`ros/supervisor_node.py` の `_declare_parameters`) にある。ロボットごとの値はそれぞれの設定ファイルで与える。各パラメータの意味は、上の各節を参照。

## 制約

- `LocalizationStatus.amcl_attached` は、監督ノード自身が要求した状態を表す。他の要求元がゲートを閉じていても `true` のままになる (実際にゲートが開いているかは、調停ノードの `~/state` と診断 `gate` で確認する)。
