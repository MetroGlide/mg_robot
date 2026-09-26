# mg_navigation

Nav2 のラッパー (collision_monitor / behavior_server の設定、waypoint 関連) と、自己位置推定まわりのノードを含むパッケージ。
本 README は自己位置推定 (localization) を中心に記載する。

## 自己位置推定の構成

ホイールオドメトリ・AMCL・GNSS を EKF (`robot_localization` の `ekf_global_node`、world フレーム = `map`) で融合している。

```
wheel_odometry_node ─ /odom/raw ─► wheel_odom_corrector_node ─ /odom ─► odometry_tf_broadcaster (odom→base_footprint)
   (mg_drivers)                       (mg_drivers。スケール・バイアス補正)          │
                                                                                 ▼ 速度 (vx, vy, vyaw)
/scan_top_lidar ─► amcl ─ /amcl_pose_origin ─► amcl_publish_controller_node ─ /amcl_pose ─► ekf_global_node ─► map→odom TF
                     ▲                              ▲ SetBool で入/切                  ▲ 位置・yaw          (10 Hz)
                     │ /initialpose                 │                                  │
                     │                              │           /odom/gps ─────────────┘ 位置 (yaw は使わない)
gnss_amcl_initializer_node ◄─ /odom/gps ◄─ slam_gnss_nav_bridge ◄─ /navpvt (u-blox NavPVT)
                     ▲                                (slam_gnss_2d)
                     │ request_reinit
              amcl_watchdog_node   または   localization_supervisor_node
```

| ノード | 役割 |
| :--- | :--- |
| `amcl` | LiDAR と地図による自己位置推定。出力は `/amcl_pose_origin` (TF は出さない) |
| `amcl_gate_arbiter` | AMCL の入/切を要求元ごとに調停する (下記)。ゲートを操作するのはこのノードだけ |
| `amcl_publish_controller_node` | `/amcl_pose_origin` を `/amcl_pose` へ中継する。SetBool で止めると AMCL が EKF に入らない |
| `slam_gnss_nav_bridge` | `/navpvt` を、`gnss_transform.yaml` で map 座標に直して `/odom/gps` に出す。アンテナ位置を車体中心に補正し、測位の質に応じた分散を付ける ([slam_gnss_2d](../slam_gnss_2d/README.md)) |
| `gnss_amcl_initializer_node` | 起動時 (と再初期化の要求時) に、精度の良い `/odom/gps` から `/initialpose` (AMCL) と `/set_pose` (EKF) を出して初期化する。EKF の初期共分散が大きく、AMCL の初期値 (原点) に引かれるため、EKF にも送る (`publish_set_pose`) |
| `amcl_watchdog_node` | AMCL の共分散が大きい状態が続いたら、`gnss_amcl_initializer_node` に再初期化を要求する (従来の監視) |
| `localization_supervisor_node` | AMCL のずれを検知して EKF から切り離し、EKF の姿勢で復旧する (下記) |
| `ekf_global_node` | ホイールオドメトリの速度、AMCL の位置・yaw、GNSS の位置を融合する (`mg_drivers/params/ekf_global.yaml`) |

## amcl_gate_arbiter (AMCL の入/切の調停)

ウェイポイントの `amcl_on` / `amcl_off` と監督ノードが、同じゲートを直接切り替えると互いの意図を上書きする。
そこで要求元ごとのサービスを設け、**すべての要求元が「入れる」のときだけ**ゲート (`amcl_publish_controller_node`) を開く。

| サービス (SetBool。data=true で入れる) | 要求元 |
| :--- | :--- |
| `/amcl_gate_arbiter/waypoint/change_publish_state` | ウェイポイントの `amcl_on` / `amcl_off` |
| `/amcl_gate_arbiter/supervisor/change_publish_state` | 監督ノードの切り離し・復帰 |

- ウェイポイントが切った区間では、監督ノードが復旧しても AMCL は戻らない。監督ノードが切っている間は、`amcl_on` でも戻らない。
- 反映に失敗したり、ゲートが後から起動したりしても、1 Hz で合わせ直す。ゲートのノードが落ちて戻ったときも (再起動で開いた状態に戻るため) 送り直す。ただし 1 秒より短い間の再起動は検出できない。
- `amcl_off` の後に `amcl_on` を送るのはウェイポイントの作り手の責任 (送り忘れると AMCL は戻らない。自動解除はしない)。
- 監視ノードの種類 (`none` / `watchdog` / `supervisor`) によらず常に起動する。負荷はごく小さい。

## localization_supervisor_node

`amcl_watchdog_node` は AMCL の**自己申告の共分散**しか見ないため、AMCL が「自信を持って間違えている」状態を検知できず、
検知してもずれた AMCL を EKF に入れ続け、復旧も GNSS の生の値 (進行方向から作った向き) に頼っていた。
監督ノードは、AMCL とは独立な情報でずれを検知し、切り離してから復旧する。

### 検知 (1 Hz)

| 判定 | 内容 |
| :--- | :--- |
| `gnss` | 標準偏差 `gnss_max_sigma_m` (既定 2 m) 以下の GNSS と AMCL の位置の食い違い (マハラノビス距離の二乗。分散は AMCL と GNSS の両方を使うので、単独測位では数 m 以上のずれを検知する) |
| `jump` | 連続する AMCL の推定が示す `map→odom` が、オドメトリの示す動きから飛んだ (`jump_threshold_m` / `_rad`) |
| `scan` | EKF の姿勢でスキャンを地図に重ね、占有セルから `scan_tolerance_m` 以内の点の割合 (一致率) を見る。**一致率の絶対値は地図や LiDAR で決まる (正しい姿勢でも 0.3 前後のことがある)** ので、正常なときの一致率の中央値の `scan_ratio_factor` 倍 (下限 `scan_ratio_min`) を下回ったら異常とする。姿勢の周り (±`scan_search_range_m`) でずらしたときの一致率の増え方 (利得) も記録するが、別走行の地図との位置合わせのずれで正しい姿勢でも 0.4 前後になるため、`scan_gain_threshold` は既定で 1.0 (判定に使わない) |
| `diff` | AMCL と EKF の位置が `diff_threshold_m` 以上離れている |

- どの判定も、使えないとき (GNSS の精度が悪すぎる、スキャンが無いなど) は判定しない (異常とも正常とも数えない)。
- 判定の値は `LocalizationStatus` に出る (`amcl_gnss_d2` `amcl_jump_m` `amcl_ekf_diff_m` `scan_match_ratio` `scan_match_gain`)。

### 動作モード (`recovery_enabled`)

| 値 | 動作 |
| :--- | :--- |
| `false` (既定) | **検知と通知だけ**。異常を確認したら `DEGRADED` になり (`/localization/status`、`/diagnostics` が ERROR)、異常が消えたら `NORMAL` に戻る。AMCL は切り離さない |
| `true` | 下の状態遷移どおり、AMCL を切り離して姿勢を選び、再初期化して戻す (実験的) |

既定を `false` にした理由: 再生評価 (kidnap) で、復旧を有効にすると、選んだ姿勢が誤っていた試行で誤差が数十 m まで広がった
(監視なし p95 10.7 m に対し、最悪 143 m)。AMCL は kidnap の後に自力で戻ることも多く、検知に数秒かかる間の誤差は復旧では防げない。
復旧を成立させる課題は「今後の課題」に記載する。

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

パラメータは `params/localization_supervisor.yaml`。

### 制約

- `LocalizationStatus.amcl_attached` は、監督ノード自身が要求した状態を表す。ウェイポイントが `amcl_off` で AMCL を止めていても `true` のままになる (実際にゲートが開いているかは調停ノードのログで確認する)。

- 起動直後 (`startup_grace_sec`、既定 30 s) と復旧直後 (`grace_after_recovery_sec`) は、EKF・AMCL が落ち着くまで判定しない (値の記録だけ行う)。

## 起動引数

`mg_bringup` の `bringup_navigation.launch.py` から `mg_navigation` の launch まで同じ名前で伝わる。

| 引数 | 既定 | 内容 |
| :--- | :--- | :--- |
| `localization_monitor` | `watchdog` | 自己位置の監視ノード `none` \| `watchdog` \| `supervisor` |
| `supervisor_params_file` | `params/localization_supervisor.yaml` | 監督ノードのパラメータ |
| `use_gnss_amcl_initializer` | `true` | GNSS から AMCL の初期姿勢を与えるノードの起動 |
| `use_navigation` | `true` | `false` で map_server / AMCL / EKF だけを起動する (rosbag 再生での評価用) |
| `nav2_params_file` | `params/nav2_params.yaml` | Nav2 (AMCL を含む) のパラメータ |
| `use_odom_corrector` | 実機 `true` / シミュレータ `false` | ホイールオドメトリの補正 ([mg_drivers](../mg_drivers/README.md)) |

**従来の構成に戻す**: `localization_monitor:=watchdog` (既定) のままにすれば従来の監視、`use_odom_corrector:=false` で補正なし。
監督ノードは 1 Hz の判定と、1 Hz・最大 180 点のスキャンの照合だけで、負荷はごく小さい。

## 評価

パラメータの変更や復旧の仕組みは、rosbag と SLAM の出力 (地図・pose_graph) を使って再生で評価する。
実機を使わずに、同じ条件で変種を比べられる。手順は [tools/README.md](../tools/README.md) の
`run_localization_variant.sh` と `eval_localization.py` を参照。

## テスト

```bash
make test pkg=mg_navigation   # 監督ノードの判定・状態遷移
```
