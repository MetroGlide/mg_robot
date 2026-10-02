# 自己位置推定

ホイールオドメトリ・AMCL・GNSS を EKF (`robot_localization` の `ekf_global_node`、world フレーム = `map`) で融合して、`map→odom` を作る。
launch は `mg_bringup` の `bringup_navigation.launch.py` (EKF と GNSS ブリッジ) と、`mg_navigation` の `localization_launch.py` (map_server、AMCL、初期化、監視、調停)。

## 構成

```
wheel_odometry_node ─ /odom/raw ─► wheel_odom_corrector_node ─ /odom ─► odometry_tf_broadcaster (odom→base_footprint)
   (mg_drivers)                       (mg_drivers。スケール・バイアス補正)          │
                                                                                 ▼ 速度 (vx, vy, vyaw)
/scan_top_lidar ─► amcl ─ /amcl_pose_origin ─► amcl_publish_controller_node ─ /amcl_pose ─► ekf_global_node ─► map→odom TF
                     ▲                              ▲ SetBool で入/切                  ▲ 位置・yaw          (30 Hz)
                     │ /initialpose                 │                                  │
                     │                              │           /odom/gps ─────────────┘ 位置 (yaw は使わない)
gnss_amcl_initializer_node ◄─ /odom/gps ◄─ slam_gnss_nav_bridge ◄─ /navpvt (u-blox NavPVT)
                     ▲                                (slam_gnss_2d)
                     │ request_reinit
              amcl_watchdog_node   または   localization_supervisor_node
```

EKF の周期は `mg_drivers/params/ekf_global.yaml` の `frequency` (30 Hz)。

| ノード | 役割 |
| :--- | :--- |
| `amcl` | LiDAR と地図による自己位置推定。出力は `/amcl_pose_origin` (TF は出さない) |
| `amcl_gate_arbiter` | AMCL の入/切を、要求元ごとに調停する ([下記](#amcl_gate_arbiter-amcl-の入切の調停))。ゲートを操作するのは、このノードだけ |
| `amcl_publish_controller_node` | `/amcl_pose_origin` を `/amcl_pose` へ中継する (`mg_drivers` の `generic_publish_controller_node.py`)。SetBool で止めると、AMCL が EKF に入らない |
| `slam_gnss_nav_bridge` | `/navpvt` を、`gnss_transform.yaml` で map 座標に直して `/odom/gps` に出す。アンテナの位置を車体の中心に補正し、測位の質に応じた分散を付ける ([slam_gnss_2d](../../slam_gnss_2d/README.md)) |
| `gnss_amcl_initializer_node` | 起動時 (と再初期化の要求時) に、精度の良い `/odom/gps` から `/initialpose` (AMCL) と `/set_pose` (EKF) を出して初期化する ([下記](#gnss_amcl_initializer_node)) |
| `amcl_watchdog_node` | AMCL の共分散が大きい状態が続いたら、`gnss_amcl_initializer_node` に再初期化を要求する ([下記](#amcl_watchdog_node)) |
| `localization_supervisor_node` | AMCL のずれを検知して EKF から切り離し、EKF の姿勢で復旧する ([下記](#localization_supervisor_node)) |
| `ekf_global_node` | ホイールオドメトリの速度、AMCL の位置・yaw、GNSS の位置を融合する (`mg_drivers/params/ekf_global.yaml`) |

`gnss_amcl_initializer_node` は、起動の 10 秒後に、`amcl_watchdog_node` と `localization_supervisor_node` は 12 秒後に起動する (`localization_launch.py` の `TimerAction`)。

## amcl_gate_arbiter (AMCL の入/切の調停)

ウェイポイントの `amcl_on` / `amcl_off` と監督ノードが、同じゲートを直接切り替えると、互いの意図を上書きする。
そこで要求元ごとのサービスを設け、**すべての要求元が「入れる」のときだけ**ゲート (`amcl_publish_controller_node`) を開く。

| サービス (SetBool。data=true で入れる) | 要求元 |
| :--- | :--- |
| `/amcl_gate_arbiter/waypoint/change_publish_state` | ウェイポイントの `amcl_on` / `amcl_off` |
| `/amcl_gate_arbiter/supervisor/change_publish_state` | 監督ノードの切り離し・復帰 |

- ウェイポイントが切った区間では、監督ノードが復旧しても AMCL は戻らない。監督ノードが切っている間は、`amcl_on` でも戻らない。
- 反映に失敗したり、ゲートが後から起動したりしても、1 Hz で合わせ直す。ゲートのノードが落ちて戻ったときも (再起動で開いた状態に戻るため) 送り直す。ただし 1 秒より短い間の再起動は検出できない。
- `amcl_off` の後に `amcl_on` を送るのは、ウェイポイントの作り手の責任 (送り忘れると AMCL は戻らない。自動解除はしない)。
- 監視ノードの種類 (`none` / `watchdog` / `supervisor`) によらず、常に起動する。負荷はごく小さい。
- 状態は `/amcl_gate_arbiter/state` (`mg_msgs/GateArbiterState`、transient_local) に、変化したときだけ配信する。`desired` は要求元の意図 (全員が入れてよいとき true)、`applied` はゲートへの反映済みか (ゲートのノードの再起動で不明になったときは false)、`holders` は止めている要求元。Web UI の「Actions」が表示に使う。
- Web UI の「Actions」の AMCL の入/切は、`waypoint` の要求元として呼ぶ。次のウェイポイントのアクションで上書きされる。

## gnss_amcl_initializer_node

`/odom/gps` (map 座標の GNSS の位置) の精度が、続けて条件を満たしたとき、`/initialpose` (AMCL) と、`publish_set_pose` が `true` なら `/set_pose` (EKF) に、初期の姿勢を出す。
EKF の初期共分散が大きく、AMCL の初期値 (原点) に引かれるので、EKF にも送る。サービス `~/request_reinit` (`std_srvs/Trigger`) で、内部の状態をリセットして、初期化をやり直す。

主なパラメータ。

| パラメータ | 既定値 | 内容 |
| :--- | :--- | :--- |
| `required_consecutive_good` | `5` | 条件を満たす測位が続く回数 (launch で `2` に上書き) |
| `max_position_std_m` | `5.0` | 使う測位の水平の標準偏差の上限 [m] |
| `max_vertical_std_m` | `10.0` | 同、鉛直 (`ignore_z_std` が `true` のときは見ない) |
| `use_fixed_heading` | `True` | 向きが得られないとき、固定の向き (`fixed_heading`、1.57 rad) を使う (launch で `False` に上書き) |
| `override_pose_covariance` / `pose_covariance` | `False` | 初期姿勢の共分散を固定の値にする (launch で `True`、x・y は 0.25、yaw は約 0.069) |
| `max_consecutive_bad` | `20` | 条件を満たさない測位がこの回数続いたら、初期化をあきらめる (`request_reinit` でやり直せる) |
| `odom_age_timeout_sec` / `ignore_odom_age` | `2.0` / `False` | 測位の古さの確認 |

## amcl_watchdog_node

AMCL の位置の共分散を監視し、大きい状態が続いたら、`gnss_amcl_initializer_node` に再初期化を要求する。従来の監視で、`localization_monitor` の既定 (`watchdog`)。

- `amcl_pose_origin` (`PoseWithCovarianceStamped`) を購読し、共分散から指標を計算する。
- 指標が `threshold` を超える状態が `consecutive_count` 回続くと、異常とする。
- 異常のとき、`/gnss_amcl_initializer_node/request_reinit` (`Trigger`) を呼ぶ。失敗したら 1 秒おいて、最大 `max_retries` 回まで試す。成功したら、`recovery_backoff_sec` の間は、次の再初期化を抑える。
- 実装は `mg_navigation/amcl_watchdog/` (指標の計算は `metrics.py`、ノードは `node.py`)。起動のラッパーは `scripts/amcl_watchdog/amcl_watchdog_node.py`。

| パラメータ | 既定値 | 内容 |
| :--- | :--- | :--- |
| `metric` | `trace_xy` | 指標: `trace_xy` (x・y の分散の和)、`determinant_xy` (x・y の共分散行列の行列式)、`max_eigenvalue_xy` (同、最大固有値) |
| `threshold` | `2.0` | 異常とする指標の値 |
| `consecutive_count` | `3` | 連続で超える回数 |
| `recovery_backoff_sec` | `20.0` | 再初期化に成功したあと、次を抑える時間 [s] |
| `max_retries` | `3` | サービスの呼び出しの再試行の回数 |
| `initializer.call_timeout_sec` | `5.0` | サービスの応答を待つ時間 [s] |
| `min_interval_between_events` | `0.0` | 異常の検知の最短の間隔 [s] |

この監視は、AMCL の**自己申告の共分散**しか見ない。そのため、AMCL が「自信を持って間違えている」状態は検知できない。

## localization_supervisor_node

`amcl_watchdog_node` は、AMCL の自己申告の共分散しか見ないため、AMCL が「自信を持って間違えている」状態を検知できず、検知してもずれた AMCL を EKF に入れ続け、復旧も GNSS の生の値 (進行方向から作った向き) に頼っていた。
監督ノードは、AMCL とは独立な情報でずれを検知し、切り離してから復旧する。

### 検知 (1 Hz)

| 判定 | 内容 |
| :--- | :--- |
| `gnss` | 標準偏差 `gnss_max_sigma_m` (既定 2 m) 以下の GNSS と AMCL の位置の食い違い (マハラノビス距離の二乗。分散は AMCL と GNSS の両方を使うので、単独測位では数 m 以上のずれを検知する) |
| `jump` | 連続する AMCL の推定が示す `map→odom` が、オドメトリの示す動きから飛んだ (`jump_threshold_m` / `_rad`) |
| `scan` | EKF の姿勢でスキャンを地図に重ね、占有セルから `scan_tolerance_m` 以内の点の割合 (一致率) を見る。**一致率の絶対値は地図や LiDAR で決まる (正しい姿勢でも 0.3 前後のことがある)** ので、正常なときの一致率の中央値の `scan_ratio_factor` 倍 (下限 `scan_ratio_min`) を下回ったら異常とする。姿勢の周り (±`scan_search_range_m`) でずらしたときの一致率の増え方 (利得) も記録するが、別走行の地図との位置合わせのずれで正しい姿勢でも 0.4 前後になるため、`scan_gain_threshold` は既定で 1.0 (判定に使わない) |
| `diff` | AMCL と EKF の位置が `diff_threshold_m` 以上離れている |

- どの判定も、使えないとき (GNSS の精度が悪すぎる、スキャンが無いなど) は判定しない (異常とも正常とも数えない)。
- 判定の値は `LocalizationStatus` に出る (`amcl_gnss_d2`、`amcl_jump_m`、`amcl_ekf_diff_m`、`scan_match_ratio`、`scan_match_gain`)。
- 起動直後 (`startup_grace_sec`、既定 30 s) と復旧直後 (`grace_after_recovery_sec`) は、EKF・AMCL が落ち着くまで判定しない (値の記録だけ行う)。

### 動作モード (`recovery_enabled`)

| 値 | 動作 |
| :--- | :--- |
| `false` (既定) | **検知と通知だけ**。異常を確認したら `DEGRADED` になり (`/localization/status`、`/diagnostics` が ERROR)、異常が消えたら `NORMAL` に戻る。AMCL は切り離さない |
| `true` | 下の状態遷移どおり、AMCL を切り離して姿勢を選び、再初期化して戻す (実験的) |

既定を `false` にした理由: 再生評価 (kidnap) で、復旧を有効にすると、選んだ姿勢が誤っていた試行で誤差が数十 m まで広がった (監視なし p95 10.7 m に対し、最悪 143 m)。AMCL は kidnap の後に自力で戻ることも多く、検知に数秒かかる間の誤差は、復旧では防げない。

### 状態遷移 (`state_machine.py`。`recovery_enabled: true` のとき)

```
NORMAL ─ 異常の疑い ─► SUSPECT ─ 確認 ─► ISOLATED (AMCL を EKF から切り離す)
   ▲                     │ 異常が消える            │ isolate_hold_sec 待つ
   └─────────────────────┘                          ▼
   ▲                                          RECOVERING (姿勢を選んで /initialpose と /set_pose を送る)
   └──────────── 収束を確認 (AMCL を戻す) ◄──────────┤ recover_timeout_sec を超えて max_reinit_attempts 回続く
                                                      ▼
                                                  DEGRADED (通知。degraded_retry_sec ごとに再試行)
```

- **切り離し**: 調停ノードの `/amcl_gate_arbiter/supervisor/change_publish_state` (SetBool) で `/amcl_pose` を止める。EKF はオドメトリと GNSS だけで動く。
- **復旧の姿勢**: 次の候補のうち、スキャンが地図に最もよく合うものを選ぶ (比べられなければ GNSS を優先)。
  1. EKF の姿勢
  2. **巻き戻し**: 異常が始まる `rollback_margin_sec` 前の `map→odom` に、今のオドメトリの動きを足した姿勢 (AMCL が飛んで EKF が引きずられた場合)
  3. 精度の良い GNSS の位置 + EKF の向き (`reinit_gnss_max_sigma_m` 以下のとき。初期化の広がりには GNSS の標準偏差を使う)
- **初期化**: 選んだ姿勢を `/initialpose` (AMCL) と `/set_pose` (EKF) に送る。初期化した直後の AMCL の飛びは異常としない。
- **DEGRADED**: `action_on_degraded: warn` は通知のみ、`slow` は `/speed_limit` で速度を制限する。

### 出力

- `/localization/status` (`mg_msgs/LocalizationStatus`): 状態、AMCL を EKF に入れているか、判定の値、直近のできごと
- `/diagnostics`: `localization_supervisor` (NORMAL=OK、DEGRADED=ERROR、その他=WARN)

パラメータは `params/localization_supervisor.yaml` (各値の説明は、ファイルのコメント)。

### 制約

- `LocalizationStatus.amcl_attached` は、監督ノード自身が要求した状態を表す。ウェイポイントが `amcl_off` で AMCL を止めていても `true` のままになる (実際にゲートが開いているかは、調停ノードのログで確認する)。

## 従来の構成に戻す

- `localization_monitor:=watchdog` (既定) のままにすれば、従来の監視。
- `use_odom_corrector:=false` で、ホイールオドメトリの補正なし ([mg_drivers](../../mg_drivers/doc/wheel_odom_corrector.md))。
- 監督ノードの負荷は、1 Hz の判定と、1 Hz・最大 180 点のスキャンの照合だけで、ごく小さい。

## 評価

パラメータの変更や復旧の仕組みは、rosbag と SLAM の出力 (地図・pose_graph) を使って、再生で評価する。実機を使わずに、同じ条件で変種を比べられる。
手順は [tools の README](../../tools/README.md) の `run_localization_variant.sh` と `eval_localization.py` を参照。
