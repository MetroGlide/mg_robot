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

## `/odom/gps` の仕様

`robot_localization` が、そのまま融合できるように作る。

- `header.frame_id`: `map`
- `header.stamp`: 受信した時刻 (`now()`) から `time_offset_sec` を引いた時刻
- `child_frame_id`: アンテナの位置の補正ができたときは `base_footprint`、できなかったときは `gps_link`
- `pose.pose.position`: 地図の位置 (z は 0)。**アンテナの位置の補正が有効なとき (`lever_arm_compensation`) は、車体の中心の位置**
- `pose.pose.orientation`: 推定したヘディングのクォータニオン (EKF は使わない。AMCL の初期化に使う)
- **共分散** (`pose.covariance`、36 要素):
  - `cov[0]` (x)、`cov[7]` (y): `max(hAcc, 下限)² × 係数`。アンテナの位置を補正できなかったときは、レバーアームの長さの二乗 (約 0.085 m²) を足す
  - `cov[35]` (yaw): ヘディングの分散 (既定 0.1 rad²)
  - `cov[14]` (z)、`cov[21]` (roll)、`cov[28]` (pitch): 未観測として 99999.0 (EKF に無視させる)

### 分散の決め方

解の種類ごとに、`max(hAcc, 下限)² × 係数` とする。

| 解の種類 | 下限 | 係数 |
| :--- | :--- | :--- |
| RTK Fix | `fix_floor_m` | `fix_scale` |
| RTK Float | `float_floor_m` | `float_scale` |
| 単独測位 | `single_floor_m` | `single_scale` |

- コードの既定は、下限 0.02 m、係数 1.0 (従来と同じ `hAcc²`)。
- **`params/nav_bridge.yaml` では、係数が 1.44 (= 1.2²)**。SLAM の `gnss.navpvt_hacc_scale` (標準偏差を hAcc の 1.2 倍) に合わせた値。
- NavSatFix の入力 (`gnss_input: navsatfix`) では、搬送波位相の解の種類が分からないので、RTK (`STATUS_GBAS_FIX` 以上) だけを Fix、それ以外を単独測位とし、`position_covariance[0]` の平方根を hAcc とする。
- `accept_single: false` にすると、単独測位は使わない。
- 位置の分散の 2 倍が `max_covariance_threshold` (49 m²、標準偏差で約 4.95 m) を超える測位は、使わない。

### アンテナの位置の補正

`robot_localization` は、Odometry の姿勢について `child_frame_id` を使わない。アンテナの位置のまま渡すと、アンテナのずれ (0.26, -0.13) m が車体の位置として扱われ、車体の向きによって、最大約 0.29 m の誤差になる。
そこで、EKF が出す `map→base_footprint` の TF から車体の向き ψ を取り、`車体の中心 = アンテナの位置 − R(ψ)·(lx, ly)` にして出す。
TF が無い、または `yaw_max_age_sec` より古いときは、アンテナの位置のまま、レバーアームの分だけ分散を増やして出す。

- 負荷は、1 Hz の測位ごとの座標変換だけで、ごくわずか。
- `lever_arm_compensation: false` で、従来どおりアンテナの位置を出す。

## パラメータ

`params/nav_bridge.yaml` (ノード名 `slam_gnss_nav_bridge`)。シミュレータ用は `mg_simulation/config/nav_bridge_sim.yaml` ([mg_simulation](../../mg_simulation/README.md))。

| パラメータ | コードの既定値 | `nav_bridge.yaml` | 内容 |
| :--- | :--- | :--- | :--- |
| `gnss_transform_file` | `""` | launch から渡される | 読み込む `gnss_transform.yaml` (必須) |
| `gnss_input` | `navpvt` | `navpvt` | 入力: `navpvt` / `navsatfix` |
| `gnss_topic` | `/navpvt` | `/navpvt` | 受信するトピック |
| `map_frame_id` | `map` | - | 地図のフレーム |
| `base_frame_id` | `base_footprint` | - | 車体のフレーム (アンテナの位置の補正で、向きを取る TF の子) |
| `gps_frame_id` | `gps_link` | - | アンテナのフレーム |
| `heading_source` | `computed` | `computed` | ヘディングの算出元: `computed` (変位ベクトル) / `navpvt` (内蔵の方位) |
| `heading_min_distance` | `0.6` | - | 変位からヘディングを求める最小の移動量 [m] |
| `heading_smoothing_alpha` | `0.6` | - | Circular EMA の平滑化係数 |
| `min_publish_distance` | `1.0` | `1.0` | 前の地点からこの距離以上動くまで、次の測位を出さない [m] |
| `max_covariance_threshold` | `49.0` | `49.0` | 配信を許す最大の共分散 (2σ²) [m²] |
| `lever_arm_compensation` | `true` | `true` | アンテナの位置を、車体の中心の位置に直す |
| `lever_arm_x` / `lever_arm_y` | `0.26` / `-0.13` | `0.26` / `-0.13` | アンテナの車体座標での位置 [m] (URDF の `base_footprint`→`gps_link` と合わせる) |
| `yaw_max_age_sec` | `1.0` | - | 向きに使う TF の古さの上限 [s] |
| `time_offset_sec` | `0.0` | `0.0` | `/odom/gps` のスタンプを、`now()` からこの秒数だけ過去にする (測位が届くまでの遅れの補正) |
| `fix_floor_m` / `float_floor_m` / `single_floor_m` | `0.02` | `0.02` | 解の種類ごとの hAcc の下限 [m] |
| `fix_scale` / `float_scale` / `single_scale` | `1.0` | `1.44` | 解の種類ごとの、分散にかける係数 |
| `accept_single` | `true` | `true` | `false` で単独測位を使わない |
