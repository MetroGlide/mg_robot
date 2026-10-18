# ナビゲーションとの連携 (slam_gnss_nav_bridge_node)

自律走行のとき、`slam_gnss_nav_bridge_node` が、保存された `gnss_transform.yaml` を読む。走行中に受信する生の GNSS (`/navpvt` または `/gps/fix`) を、地図の座標に直して、`/odom/gps` に配信する。
`mg_bringup` の `bringup_navigation.launch.py` が、ノード名 `slam_gnss_nav_bridge` で起動する (引数 `use_slam_gnss_bridge`、`bridge_params_file`、`gnss_transform_file`)。

```
[生の GNSS]  /navpvt (ublox_msgs/NavPVT) または /gps/fix (sensor_msgs/NavSatFix)
      │
      ▼
[slam_gnss_nav_bridge_node]
  1. GeographicLib で UTM に投影
  2. アンカーからの相対 → 地図の座標 ([gnss_transform.md](./gnss_transform.md))
  3. 変位ベクトルからヘディングを推定 (Circular EMA フィルタ)
  4. 解の種類 (Fix / Float / 単独) と水平精度 (hAcc) から、位置の分散を決める
  5. アンテナの位置を、車体の中心 (base_footprint) の位置に直す (向きは EKF の map→base の TF)
      │
      ▼
[/odom/gps] (nav_msgs/Odometry) ──► ekf_global_node (位置)、gnss_amcl_initializer_node (初期化)
```

自己位置推定の全体 (AMCL・初期化・監視) は [mg_navigation](../../mg_navigation/doc/localization.md)。

## 出力

| トピック / サービス | 型 | 内容 |
| :--- | :--- | :--- |
| `/odom/gps` | `nav_msgs/Odometry` | 地図の座標の GNSS の位置 |
| `/slam_gnss_2d/anchor` | `sensor_msgs/NavSatFix` (transient_local) | アンカーの緯度経度。外部のノードと Web UI が、地図の絶対位置を知るため |
| `~/change_publish_state` | `std_srvs/SetBool` | `/odom/gps` の配信を止める・再開する (ウェイポイントの `gps_on` / `gps_off` が使う)。ノード名 `slam_gnss_nav_bridge` なら `/slam_gnss_nav_bridge/change_publish_state` |
| `~/publish_state` | `std_msgs/Bool` (transient_local) | 配信の状態。起動時と、切り替えたときだけ配信する |

### 診断とイベント

GNSS を地図に使えているか (採用しているか) を、診断 `slam_gnss_nav_bridge: adoption` (`diagnostic_updater`、集約で `/Robot/Localization/GnssBridge`) に出す。ublox の受信そのものの品質は `gnss_diagnostics_node` が出す。

| 状態 | レベル | メッセージ |
| :--- | :--- | :--- |
| 変換ファイルを読めなかった | `ERROR` | `GNSS transform not loaded` |
| `~/change_publish_state` で配信を止めている | `OK` | `publishing disabled` (意図した状態) |
| `adoption_timeout_sec` 以内に、使える測位があった | `OK` | `adopting GNSS` |
| 使える測位が無い | `WARN` (`not_adopting_level: ok` なら `OK`) | `not adopting GNSS: <理由>` |

理由 (`last_reject_reason`): `no fix` (Fix なし)、`low quality (not RTK or accuracy too low)` (解の種類・精度が条件を満たさない)、`heading unavailable` (ヘディングを求められない)、`no GNSS data yet` (まだ受信していない)。`min_publish_distance` で出さなかった測位は、使えているので採用とする。

採用の有無が変わったときだけ、イベント `gnss.adoption_changed` (`adopting`、`reason`) を出す ([doc/monitoring.md](../../doc/monitoring.md)。このパッケージは `mg_*` に依存しないので、書式を自分で守っている)。

## `/odom/gps` の仕様

`robot_localization` が、そのまま融合できるように作る。

- `header.frame_id`: `map`
- `header.stamp`: 受信した時刻 (`now()`) から `time_offset_sec` を引いた時刻
- `child_frame_id`: アンテナの位置の補正ができたときは `base_footprint`、できなかったときは `gps_link`
- `pose.pose.position`: 地図の位置 (z は 0)。**アンテナの位置の補正が有効なとき (`lever_arm_compensation`) は、車体の中心の位置**
- `pose.pose.orientation`: 推定したヘディングのクォータニオン (EKF は使わない。AMCL の初期化に使う)
- **共分散** (`pose.covariance`、36 要素):
  - `cov[0]` (x)、`cov[7]` (y): `max(hAcc, 下限)² × 係数`。アンテナの位置を補正できなかったときは、レバーアームの長さの二乗 (レバーアームの長さの二乗) を足す
  - `cov[35]` (yaw): ヘディングの分散 
  - `cov[14]` (z)、`cov[21]` (roll)、`cov[28]` (pitch): 未観測として 99999.0 (EKF に無視させる)

### 分散の決め方

解の種類ごとに、`max(hAcc, 下限)² × 係数` とする。

| 解の種類 | 下限 | 係数 |
| :--- | :--- | :--- |
| RTK Fix | `fix_floor_m` | `fix_scale` |
| RTK Float | `float_floor_m` | `float_scale` |
| 単独測位 | `single_floor_m` | `single_scale` |

- コードの既定の係数は 1 (従来と同じ `hAcc²`)。
- **`params/nav_bridge.yaml` の係数は、SLAM の `gnss.navpvt_hacc_scale` (標準偏差を hAcc の何倍にするか) の二乗に合わせる**。
- NavSatFix の入力 (`gnss_input: navsatfix`) では、搬送波位相の解の種類が分からないので、RTK (`STATUS_GBAS_FIX` 以上) だけを Fix、それ以外を単独測位とし、`position_covariance[0]` の平方根を hAcc とする。
- `accept_single: false` にすると、単独測位は使わない。
- 位置の分散の 2 倍が `max_covariance_threshold` を超える測位は、使わない。

### 外からの分散の倍率

`variance_scale_topic` を設定すると、`std_msgs/Float64` の倍率を購読し、上の分散にかける (倍率は 1 以上 `variance_scale_max` 以下に収める。1 未満は 1 として扱う)。
GNSS と他の情報源の残差から、GNSS の自己申告より実際の誤差が大きいと分かったとき (地図と GNSS のずれは場所ごとに違うので、一定の補正はせずに信頼度だけを下げる)、自己位置の監督が倍率を出す ([localization_supervisor](../../localization_supervisor/doc/plugins.md#対処-actuator) の `variance_scale`)。

- 最後に受け取ってから `variance_scale_timeout_sec` を過ぎたら、倍率を 1 に戻す (監督が止まったときに、GNSS を弱いままにしない)
- 診断 `adoption` の値 `variance_scale` に、今の倍率を出す
- トピックが空 (既定) なら購読しない。止めたいときは、このパラメータを空にする

### アンテナの位置の補正

`robot_localization` は、Odometry の姿勢について `child_frame_id` を使わない。アンテナの位置のまま渡すと、アンテナのずれ (レバーアーム) が車体の位置として扱われ、車体の向きによって、レバーアームの長さまでの誤差になる。
そこで、EKF が出す `map→base_footprint` の TF から車体の向き ψ を取り、`車体の中心 = アンテナの位置 − R(ψ)·(lx, ly)` にして出す。
TF が無い、または `yaw_max_age_sec` より古いときは、アンテナの位置のまま、レバーアームの分だけ分散を増やして出す。

- 負荷は、測位ごとの座標変換だけで、ごくわずか。
- `lever_arm_compensation: false` で、従来どおりアンテナの位置を出す。

## パラメータ

`params/nav_bridge.yaml` (ノード名 `slam_gnss_nav_bridge`)。シミュレータ用は `mg_simulation/config/nav_bridge_sim.yaml` ([mg_simulation](../../mg_simulation/README.md))。値はこれらの yaml が正 (yaml に無い項目は、コードの既定を使う)。

| パラメータ | 内容 |
| :--- | :--- |
| `gnss_transform_file` | 読み込む `gnss_transform.yaml` (必須) |
| `gnss_input` | 入力: `navpvt` / `navsatfix` |
| `gnss_topic` | 受信するトピック |
| `map_frame_id` | 地図のフレーム |
| `base_frame_id` | 車体のフレーム (アンテナの位置の補正で、向きを取る TF の子) |
| `gps_frame_id` | アンテナのフレーム |
| `heading_source` | ヘディングの算出元: `computed` (変位ベクトル) / `navpvt` (内蔵の方位) |
| `heading_min_distance` | 変位からヘディングを求める最小の移動量 [m] |
| `heading_smoothing_alpha` | Circular EMA の平滑化係数 |
| `min_publish_distance` | 前の地点からこの距離以上動くまで、次の測位を出さない [m] |
| `max_covariance_threshold` | 配信を許す最大の共分散 (2σ²) [m²] |
| `adoption_timeout_sec` | 使える測位がこの時間なければ、診断を `not_adopting_level` にする [s] |
| `not_adopting_level` | 使える測位が無いときの診断のレベル (`warn` / `ok`)。停止中は方位を求められないなど、無いのが正常な環境で `ok` にする |
| `lever_arm_compensation` | アンテナの位置を、車体の中心の位置に直す |
| `lever_arm_x` / `lever_arm_y` | アンテナの車体座標での位置 [m] (URDF の `base_footprint`→`gps_link` と合わせる) |
| `yaw_max_age_sec` | 向きに使う TF の古さの上限 [s] |
| `time_offset_sec` | `/odom/gps` のスタンプを、`now()` からこの秒数だけ過去にする (測位が届くまでの遅れの補正) |
| `fix_floor_m` / `float_floor_m` / `single_floor_m` | 解の種類ごとの hAcc の下限 [m] |
| `fix_scale` / `float_scale` / `single_scale` | 解の種類ごとの、分散にかける係数 |
| `accept_single` | `false` で単独測位を使わない |
| `variance_scale_topic` | 位置の分散にかける倍率 (`std_msgs/Float64`) を購読するトピック。空なら購読しない |
| `variance_scale_timeout_sec` | 倍率を最後に受け取ってから、これを過ぎたら 1 に戻す [s] |
| `variance_scale_max` | 倍率の上限 |
