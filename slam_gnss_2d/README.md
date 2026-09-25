# slam_gnss_2d

2D LiDAR + オドメトリ + GNSS (RTK) を統合した、C++ 実装の高精度・リアルタイム 2D 占有格子マップ生成パッケージ。

スキャンマッチング（Point-to-Line ICP, 2D-NDT, CSM）、サブマップ検証付きループクロージャ、GTSAM / iSAM2 によるファクターグラフ最適化、WGS84 ↔ UTM 測地座標変換、および Navigation2 / EKF 連携ブリッジノードを提供します。

---

## 目次

1. [パッケージ構成](#パッケージ構成)
2. [システムアーキテクチャ](#システムアーキテクチャ)
3. [提供ノード・実行可能ファイル一覧](#提供ノード実行可能ファイル一覧)
4. [アルゴリズム詳細](#アルゴリズム詳細)
   - [スキャンマッチング](#スキャンマッチング)
   - [ループクロージャ](#ループクロージャ)
   - [GNSS 拘束と逐次最適化](#gnss-拘束と逐次最適化)
5. [実行手順](#実行手順)
   - [オンライン SLAM 起動](#オンライン-slam-起動)
   - [オフライン再処理 (rosbag2)](#オフライン再処理-rosbag2)
   - [ポーズグラフ再最適化 (reoptimize_node)](#ポーズグラフ再最適化-reoptimize_node)
6. [マップ・パラメータの保存](#マップパラメータの保存)
   - [保存ラッパー CLI (save_slam_map_cli) の利用 (推奨)](#保存ラッパー-cli-save_slam_map_cli-の利用-推奨)
   - [ROS 2 サービスコールによる直接保存 (SaveSlamMap)](#ros-2-サービスコールによる直接保存-saveslammap)
   - [出力ファイル群](#出力ファイル群)
7. [gnss_transform.yaml の詳細仕様と座標変換の数学モデル](#gnss_transformyaml-の詳細仕様と座標変換の数学モデル)
   - [フォーマット仕様](#フォーマット仕様)
   - [SLAM 生成時のアライメントと初期方位回転角 (rotation_rad)](#slam-生成時のアライメントと初期方位回転角-rotation_rad)
   - [ナビゲーション時における座標変換ロジック](#ナビゲーション時における座標変換ロジック)
8. [自律移動 (Navigation2 / EKF) との連携 (slam_gnss_nav_bridge_node)](#自律移動-navigation2--ekf-との連携-slam_gnss_nav_bridge_node)
   - [ノードの役割](#ノードの役割)
   - [/odom/gps トピックの仕様と共分散行列設計](#odomgps-トピックの仕様と共分散行列設計)
   - [/slam_gnss_2d/anchor の配信](#slam_gnss_2danchor-の配信)
9. [パラメータ一覧](#パラメータ一覧)

---

## パッケージ構成

本ディレクトリ配下は以下の 2 パッケージで構成されています。

```
slam_gnss_2d/
├── slam_gnss_2d_msgs/           # カスタムメッセージおよびサービス定義
│   ├── msg/
│   │   └── PoseGraphDiff.msg    # ポーズグラフ差分メッセージ
│   └── srv/
│       ├── GetPoseGraph.srv     # フルポーズグラフ取得サービス
│       └── SaveSlamMap.srv      # マップ・パラメータ保存サービス
│
└── slam_gnss_2d/                # C++ SLAM コアエンジンおよびノード群
    ├── include/slam_gnss_2d/    # ヘッダーファイル群
    ├── src/
    │   ├── core/                # 幾何演算、設定ロード、同期、データ保存
    │   ├── gnss/                # UTM 測地変換 (GeographicLib)、アンカー管理
    │   ├── input/               # ROS 2 トピック / rosbag2 入力アダプター
    │   ├── map_manager/         # 占有格子マップ描画 (Counting / Overwrite)
    │   ├── optimizer/           # GTSAM LM / iSAM2 最適化バックエンド
    │   ├── pose_graph/          # オドメトリ / スキャンマッチング / ループ閉合ビルダー
    │   ├── ros/                 # 可視化・TF・サービスサーバー
    │   ├── scan_matching/       # ICP, NDT, CSM マッチャー
    │   ├── tools/               # 再最適化ツール、保存 CLI ツール
    │   └── nodes/               # 各種 ROS 2 ノード
    ├── launch/                  # 起動スクリプト (Python Launch)
    ├── params/                  # パラメータ設定 YAML
    └── rviz/                    # RViz 設定
```

---

## システムアーキテクチャ

```
┌─────────────────────────────────────────────────────────┐
│ Input Layer (ROS 2 / rosbag2)                           │
│  - ROS2ScanSource / BagScanSource (/scan)               │
│  - ROS2OdomSource / BagOdomSource (/odom)               │
│  - ROS2NavpvtSource / BagNavPVTSource (/navpvt)         │
│  - ROS2GnssUtmSource / BagGnssSource (/gps/fix)         │
└────────────────────────────┬────────────────────────────┘
                             │ SensorFrame (ScanData, OdomData, GnssData)
                             ▼
┌─────────────────────────────────────────────────────────┐
│ Core Layer (ROS 非依存ロジック)                         │
│  - GraphOrchestrator: 全体制御・キーフレーム選定        │
│  - ScanMatchingBuilder: ICP / NDT / CSM スキャンマッチング│
│  - LoopClosureBuilder: 近傍検索 + サブマップ検証        │
│  - UtmTransformer: GeographicLib による高精度 WGS84↔UTM │
│  - ISAM2Optimizer / GTSAMOptimizer: ファクターグラフ最適化│
│  - CountingRenderer: ヒット/ミス比率による占有格子生成  │
└────────────────────────────┬────────────────────────────┘
                             │ ポーズグラフ / 占有格子 / TF
                             ▼
┌─────────────────────────────────────────────────────────┐
│ Output & Node Layer                                     │
│  - slam_node / slam_offline_node                        │
│  - save_slam_map_cli: マップ・パラメータ一括保存 CLI    │
│  - slam_gnss_nav_bridge_node: ナビゲーション座標変換     │
│  - MapSaveService: SaveSlamMap サービスサーバー         │
│  - SlamVisualizer: マーカー / 差分 / パス配信           │
│  - SlamTfBroadcaster: map -> odom TF 配信               │
└─────────────────────────────────────────────────────────┘
```

---

## 提供ノード・実行可能ファイル一覧

| 実行可能ファイル名 | 種別 | 概要 |
| :--- | :--- | :--- |
| `slam_node` | Node | リアルタイム オンライン SLAM ノード |
| `slam_offline_node` | Node | rosbag2 を 1 スキャンずつ高速ステップ処理するオフライン SLAM ノード |
| `reoptimize_node` | Node | 保存された `pose_graph.json` を読み込み、再最適化と再描画を行うツール |
| `pose_graph_preview_node` | Node | 保存済みポーズグラフとマップをロードして RViz 等でプレビュー配信するノード |
| `slam_gnss_nav_bridge_node` | Node | 自律移動（Navigation2）時に `gnss_transform.yaml` を読み込み、生の GNSS 観測をマップ座標系に変換して `/odom/gps` を配信するブリッジ |
| `anchor_publisher_node` | Node | `gnss_transform.yaml` からアンカー位置を読み込み `/slam_gnss_2d/anchor` に配信する常駐ノード |
| `save_slam_map_cli` | CLI Tool | マップ画像 (`map.pgm`/`yaml`)、アンカー (`gnss_transform.yaml`)、ポーズグラフ (`pose_graph.json`) を一括出力する CLI 保存ツール |

---

## アルゴリズム詳細

### スキャンマッチング
1. **Point-to-Line ICP**:
   - 参照点群（直前のスキャンまたは直近 N キーフレームの合成サブマップ）の法線方向残差を最小化。
   - Huber / Cauchy 頑健カーネルを適用し、動的障害物や外れ値の影響を抑制。
2. **2D-NDT (Normal Distributions Transform)**:
   - 空間をグリッドセルに分割し、セル内の点群分布（平均・共分散）に対する尤度を直接最大化。
3. **CSM (Correlative Scan Matcher)**:
   - 多重解像度グリッドによる網羅的探索。初期値誤差が大きい場合でも高い引き込み性能を発揮。

#### 現在の既定構成 (`params/slam_gnss_2d.yaml`)
- **マッチャー**: `multi_res_csm` (2段階の相関探索 + 放物線ピーク補間)。参照は直近 30 キーフレームのローカルマップ。
- **デスキュー** (`deskew`): LiDAR は 1 走査に約 0.1 秒かかるため、各ビームの計測時刻のオドメトリ姿勢を補間し、`header.stamp` 時点の姿勢へ点群を補正します。rplidar ではビーム番号が増えるほど先に計測されます (`direction: -1`)。
- **オドメトリ融合** (`scan_matching.odom_fusion`): スキャンマッチング結果とオドメトリの並進をガウス積で融合します。廊下などマッチングの拘束が弱い方向ではオドメトリが効き、拘束が強い方向ではマッチングが効きます。
- **ロバスト化** (`optimization.between_robust_kernel`): スキャンマッチングのエッジ (逐次・near-link・ループ) に Huber カーネルを適用し、誤マッチの影響を抑えます。
- **キーフレーム**: 0.5 m / 0.5 rad ごと (slam_toolbox と同じ)。
- **スレッド数** (`scan_matching.num_threads`): 1 回のマッチングに使う OpenMP スレッド数。多すぎると並列効率が落ちます。near-link の候補は同時にマッチングします。

### ループクロージャ
- 現在位置から `search_radius` 以内かつノード間隔が `min_node_gap` 以上離れた過去のノードを候補として検出。
- 候補ノード周辺のキーフレーム点群を合成したサブマップに対してスキャンマッチングを実行。
- マッチングスコア、回転変化量、交差角（crossing angle）の判定を通過したエッジのみをループ拘束としてポーズグラフへ挿入。

### GNSS 拘束と逐次最適化
- 最初の有効な GNSS 測位から UTM ゾーンを自動決定して基準アンカーを設定。
- ロボットが `init_distance_m` 移動した時点で、SLAM 軌跡と GNSS 軌跡の変位ベクトルから初期方位角 $\theta_0$ を算出し、グラフ全体を UTM 座標系に初期回転整合。
- 走行中は水平精度が `max_sigma_m` 以下の良好な GNSS 測位に対し、位置のみを拘束するファクターをファクターグラフにインクリメンタル追加。
- **レバーアーム補正** (`gnss.lever_arm`): GNSS アンテナは `base_link` から (0.26, -0.13) m ずれているため、ファクターの予測値を `位置 + R(yaw) × レバーアーム` として、アンテナ位置に対する観測として扱います。旋回時には方位にも拘束がかかります。
- GTSAM の iSAM2 エンジンによりリアルタイムにグラフを逐次最適化。

---

## 実行手順

### オンライン SLAM 起動

```bash
# RViz2 可視化ありで起動
ros2 launch slam_gnss_2d bringup_slam_gnss_2d.launch.py rviz:=true

# シミュレーション環境 (use_sim_time:=true)
ros2 launch slam_gnss_2d bringup_slam_gnss_2d.launch.py simulation:=true rviz:=true
```

### オフライン再処理 (rosbag2)

リアルタイム再生を待たずに、bag 内のセンサデータをステップ駆動で高速処理します。

```bash
# 全件処理
ros2 launch slam_gnss_2d offline_slam_gnss_2d.launch.py \
    bag_path:=/root/ros2_data/rosbag/sample_bag \
    rviz:=true

# 区間指定処理（例: bag開始10秒後から60秒目まで処理）
ros2 launch slam_gnss_2d offline_slam_gnss_2d.launch.py \
    bag_path:=/root/ros2_data/rosbag/sample_bag \
    start_time:=10.0 \
    end_time:=60.0 \
    rviz:=true
```

- `start_time` (double, デフォルト: `0.0`): rosbag 開始からの経過秒 [s]（`0.0` で最初から）
- `end_time` (double, デフォルト: `0.0`): rosbag 開始からの経過秒 [s]（`0.0` で末尾まで）
- `skip_intermediate_rendering` (bool, デフォルト: `true`): 中間の地図描画・可視化配信を止めて終了時に一括描画します（高速化）。RViz で処理中の地図を見たいときは `false` にします。

終了時に `[timing]` として、処理ステージごとの所要時間がログ出力されます。
出力の評価は `tools/scripts/eval_slam.py` (`make bag-eval-slam`) で、パラメータ変種の比較は `tools/scripts/run_slam_variant.sh` で行えます (`tools/README.md` を参照)。


### ポーズグラフ再最適化 (reoptimize_node)

一度保存されたポーズグラフを読み込み、ループクロージャの追加探索や一括最適化（Levenberg-Marquardt）を再実行します。

```bash
ros2 launch slam_gnss_2d reoptimize.launch.py \
    input_dir:=/root/ros2_data/slam_maps/latest \
    bag_path:=/root/ros2_data/rosbag/sample_bag \
    rviz:=true
```

---

## マップ・パラメータの保存

SLAM の実行中、またはオフライン処理完了後に、マップと各種パラメータを保存します。

### 保存ラッパー CLI (save_slam_map_cli) の利用 (推奨)

`nav2_map_server map_saver_cli` と同様に、`ros2 run` で呼び出せるコマンドラインツールです。
**実行した端末のカレントディレクトリ（CWD）を自動認識し、全 3 種（4 ファイル）をワンコマンドで一括保存します。**

```bash
# 1. カレントディレクトリに保存（一番シンプル）
ros2 run slam_gnss_2d save_slam_map_cli

# 2. 相対パスまたは指定ディレクトリに保存
ros2 run slam_gnss_2d save_slam_map_cli -d ./my_maps
ros2 run slam_gnss_2d save_slam_map_cli -d /root/ros2_data/slam_maps/experiment_01

# オプション一覧
ros2 run slam_gnss_2d save_slam_map_cli --help
#  -d, --dir <PATH>        保存先ディレクトリ (省略時はカレントディレクトリ)
#  -m, --map-topic <TOPIC> マップトピック名 (デフォルト: /map)
#  -s, --service <NAME>    保存サービス名 (デフォルト: /slam_gnss_2d/save_slam_map)
#  --no-pgm                マップ画像 (map.pgm/yaml) の出力をスキップ
#  -t, --timeout <SEC>     タイムアウト秒数 (デフォルト: 5.0)
```

### ROS 2 サービスコールによる直接保存 (SaveSlamMap)

スクリプトやプログラムからサービス経由で直接保存をトリガーすることも可能です。

```bash
# 任意ディレクトリを指定して保存
ros2 service call /slam_gnss_2d/save_slam_map slam_gnss_2d_msgs/srv/SaveSlamMap "{map_dir: '/root/ros2_data/slam_maps/latest'}"

# カレントディレクトリを指定して保存 ($PWD 展開)
ros2 service call /slam_gnss_2d/save_slam_map slam_gnss_2d_msgs/srv/SaveSlamMap "{map_dir: '$PWD'}"

# '.' (相対パス) での指定も自動で絶対パス解決されます
ros2 service call /slam_gnss_2d/save_slam_map slam_gnss_2d_msgs/srv/SaveSlamMap "{map_dir: '.'}"
```

※ サービスコール単体で保存した場合、`gnss_transform.yaml` と `pose_graph.json` が生成されます（`map.pgm` / `map.yaml` を含む一括保存には上記の `save_slam_map_cli` をご利用ください）。

### 出力ファイル群

保存を実行すると、指定ディレクトリに以下のファイル群が出力されます。

| ファイル名 | 役割 | 主な利用先 |
| :--- | :--- | :--- |
| `map.pgm` | 2D 占有格子マップ画像（白: 空白、黒: 障害物、灰: 未探索） | Nav2 `map_server` |
| `map.yaml` | マップの解像度、原点座標、閾値を定義するメタデータ | Nav2 `map_server` |
| `gnss_transform.yaml` | マップ原点と地球座標系（WGS84 / UTM）の対応パラメータ | `slam_gnss_nav_bridge_node` |
| `pose_graph.json` | 全キーフレーム位置、姿勢、観測エッジ情報行列の完全履歴 | `reoptimize_node` / Web UI |

---

## gnss_transform.yaml の詳細仕様と座標変換の数学モデル

### フォーマット仕様

`gnss_transform.yaml` は、SLAM マップ座標系と地球測地系（WGS84 / UTM）の間の剛体変換を保持します。

```yaml
anchor:
  latitude: 35.6812362
  longitude: 139.7671248
anchor_utm:
  easting: 388123.456
  northing: 3949821.789
  zone: 54
  hemisphere: north
rotation_rad: 0.284592
metadata:
  created_at: 2026-09-21T01:20:00
  slam_backend: gtsam
```

### SLAM 生成時のアライメントと初期方位回転角 (rotation_rad)

- **アンカー設定**: 走行開始直後に受信した高精度 Fix（RTK-Fixed 等）の経度から UTM ゾーンが自動選定され、その座標が $(E_{anchor}, N_{anchor})$ としてアンカー登録されます。
- **初期方位角 $\theta_0$ の推定**:
  起動直後、ロボットのローカルな進行方向と UTM 座標系（真北基準）の間の回転オフセットは未知です。ロボットが `init_distance_m` (約 2.0m) 移動した時点で、SLAM の移動変位ベクトル $(\Delta x, \Delta y)$ と GNSS の UTM 移動変位ベクトル $(\Delta E, \Delta N)$ を比較し、初期方位角 $\theta_0$ を算出します。
- **ファクターグラフの回転整定**:
  算出した $\theta_0$ を用いて最初のノード姿勢を UTM 向きに回転させて確定させます。この結果、**SLAM マップ全体の X 軸は UTM Easting（東向き）、Y 軸は UTM Northing（北向き）に完全に一致するように構築されます**。
- YAML 内に記録される `rotation_rad` は、SLAM 構築時に推定されたこの初期回転角 $\theta_0$ を表す記録情報です。

### ナビゲーション時における座標変換ロジック

自律移動時、ロボットが受信した生の GNSS 観測（緯度 $\phi$, 経度 $\lambda$）からマップ座標 $(x_{map}, y_{map})$ への変換は以下の式で行われます:

$$\begin{pmatrix} E_{gnss} \\ N_{gnss} \end{pmatrix} = \mathrm{UTM\_Forward}(\phi, \lambda)$$

$$\begin{pmatrix} x_{map} \\ y_{map} \end{pmatrix} = R(\theta_{applied}) \begin{pmatrix} E_{gnss} - E_{anchor} \\ N_{gnss} - N_{anchor} \end{pmatrix}$$

ここで、$R(\theta)$ は 2D 回転行列です。

> [!IMPORTANT]
> **ナビゲーション時の適用回転角が 0.0 (`rotation_rad_ = 0.0`) となる理由**:
> SLAM マップは上記の通り、初期方位整定によって**すでにマップの座標軸が UTM 座標軸（X=東, Y=北）と平行になるようアライメントされて生成されています**。
> したがって、自律移動時にアンカー座標からの差分 $(E_{gnss} - E_{anchor}, N_{gnss} - N_{anchor})$ を計算した時点で、そのベクトルはすでにマップ座標系の X/Y 軸と向きが一致しています。
> 追加の回転（2重回転）を行わないため、`slam_gnss_nav_bridge_node` では $\theta_{applied} = 0.0$ として純粋な平行移動差分をマップ座標値として採用します。

---

## 自律移動 (Navigation2 / EKF) との連携 (slam_gnss_nav_bridge_node)

### ノードの役割

自律移動フェーズでは `slam_gnss_nav_bridge_node` を起動します。
本ノードは保存された `gnss_transform.yaml` を読み込み、ロボット走行中にリアルタイム受信する生 GNSS メッセージ（`/navpvt` または `/gps/fix`）をマップ座標系に投影し、`/odom/gps` トピックとしてパブリッシュします。

```
[生の GNSS 受信]
  - /navpvt (ublox_msgs/NavPVT) または /gps/fix (sensor_msgs/NavSatFix)
        │
        ▼
[slam_gnss_nav_bridge_node]
  1. GeographicLib による UTM 投影 (WGS84 -> UTM)
  2. アンカー相対マップ座標変換 (x_map, y_map)
  3. 変位ベクトルによるヘディング角推定 (Circular EMA フィルタ)
  4. 搬送波位相の解 (Fix / Float / 単独) と水平位置精度 (hAcc) から位置の分散を算出
  5. アンテナ位置を車体中心 (base_footprint) の位置に補正 (車体の向きは EKF の map->base の TF)
        │
        ▼
[/odom/gps] (nav_msgs/Odometry)
        │
        ▼
[robot_localization (EKF: ekf_global_node)]
  - ホイールオドメトリ (/odom: 補正済みの速度)
  - AMCL (/amcl_pose: 位置・yaw)
  - GNSS (/odom/gps: 位置)
        │ (センサフュージョン)
        ▼
[map→odom TF, /ekf_global_odom] ──► Navigation2 (BT Navigator, Controller)
```

自己位置推定全体の構成 (AMCL・初期化・監視) は [mg_navigation](../mg_navigation/README.md) を参照してください。

### /odom/gps トピックの仕様と共分散行列設計

パブリッシュされる `nav_msgs/msg/Odometry` は、カルマンフィルタ（`robot_localization`）がそのままフュージョンできるように設計されています。

- `header.frame_id`: `map` (グローバル座標フレーム)
- `header.stamp`: 受信時刻 (`now()`) から `time_offset_sec` を引いた時刻
- `child_frame_id`: アンテナ位置の補正ができたときは `base_footprint`、できなかったときは `gps_link`
- `pose.pose.position`: 変換されたマップ位置 $(x_{map}, y_{map}, 0.0)$。**アンテナ位置の補正が有効なとき (`lever_arm_compensation`) は、車体中心の位置**
- `pose.pose.orientation`: 推定ヘディング角から生成されたクォータニオン (EKF は使わない。AMCL の初期化用)
- **共分散行列 (`pose.covariance` [36])**:
  - `cov[0]` ($x$), `cov[7]` ($y$): 解の種類ごとに $\max(h_{acc}, \text{floor})^2 \times \text{scale}$ ($\sigma_{xy}^2$)。アンテナ位置を補正できなかったときは、レバーアームの長さの二乗 (約 $0.085\ \mathrm{m}^2$) を足す
  - `cov[35]` ($yaw$): 推定ヘディング角の分散 (デフォルト `0.1` $\mathrm{rad}^2$)
  - `cov[14]` ($z$), `cov[21]` ($roll$), `cov[28]` ($pitch$): 未観測成分として極大値 `99999.0` を設定（EKF 側で無視させる設定）

**アンテナ位置の補正**: `robot_localization` は Odometry の姿勢について `child_frame_id` を使わないため、アンテナ位置のまま渡すと、
アンテナのずれ $(0.26, -0.13)\ \mathrm{m}$ が車体の位置として扱われ、車体の向きによって最大約 $0.29\ \mathrm{m}$ の誤差になります。
そのため、EKF が出す `map→base_footprint` の TF から車体の向き $\psi$ を取り、車体中心 = アンテナ位置 − $R(\psi)\,(l_x, l_y)^\top$ にして出します。
TF が無い・`yaw_max_age_sec` より古いときは、アンテナ位置のまま、レバーアームの分だけ分散を増やして出します。

**負荷とロールバック**: 1 Hz の測位ごとの座標変換だけで、負荷はごくわずかです。`lever_arm_compensation: false` で従来どおりアンテナ位置を出します。

**配信の切り替え**: `~/change_publish_state` (`std_srvs/srv/SetBool`、ノード名 `slam_gnss_nav_bridge` なら `/slam_gnss_nav_bridge/change_publish_state`) で `/odom/gps` の配信を止める・再開できます (ウェイポイントの `gps_on` / `gps_off` が使う)。

### /slam_gnss_2d/anchor の配信

ナビゲーション起動時、`slam_gnss_nav_bridge_node` は `gnss_transform.yaml` に記録された基準アンカーの緯度経度を `/slam_gnss_2d/anchor`（`sensor_msgs/msg/NavSatFix`, QoS: Transient Local）として配信します。これにより、外部ノードや Web UI がマップの絶対位置をいつでも把握できます。

---

## パラメータ一覧

主なパラメータ設定（`params/slam_gnss_2d.yaml`）:

### 共通パラメータ
| パラメータ | デフォルト | 説明 |
| :--- | :--- | :--- |
| `topics.scan` | `/scan_top_lidar` | LiDAR スキャントピック名 |
| `topics.odom` | `/odom` | オドメトリトピック名 |
| `map.resolution` | `0.05` | 占有格子解像度 [m/px] |
| `map.publish_hz` | `1.0` | マップ配信レート [Hz] |
| `keyframe.min_translation` | `0.5` | キーフレーム追加 最小移動距離 [m] |
| `keyframe.min_rotation` | `0.5` | キーフレーム追加 最小回転角 [rad] |
| `deskew.enabled` | `true` | スキャンのモーションディストーション補正 |
| `deskew.direction` | `-1` | ビーム番号と計測時刻の対応 (`+1`: 番号が増えるほど後、`-1`: 先) |
| `deskew.start_offset_s` | `-0.03` | 最初に計測されるビームの `header.stamp` に対する時刻オフセット [s] |

### スキャンマッチング / ループ閉合 / 最適化
| パラメータ | デフォルト | 説明 |
| :--- | :--- | :--- |
| `scan_matching.enabled` | `true` | スキャンマッチングの有効化 |
| `scan_matching.type` | `multi_res_csm` | マッチャー種別 (`multi_res_csm` / `icp` / `ndt` / `csm` ほか) |
| `scan_matching.num_threads` | `12` | 1 回のマッチングが使う OpenMP スレッド数 (`0`: OpenMP 既定値) |
| `scan_matching.odom_fusion.information_x` | `1500.0` | 進行方向のオドメトリ融合の情報量 (`0` で無効) |
| `optimization.between_robust_kernel` | `huber` | スキャンマッチングエッジのロバスト化 (`none` / `huber` / `cauchy`) |
| `loop_closure.enabled` | `true` | ループクロージャ検出の有効化 |
| `loop_closure.search_radius`| `2.0` | ループ候補検索半径 [m] |
| `loop_closure.min_node_gap` | `50` | ループ候補の最小ノード間隔 |
| `optimization.backend` | `gtsam` | 最適化バックエンド (`gtsam` / `isam2`) |

### GNSS パラメータ
| パラメータ | デフォルト | 説明 |
| :--- | :--- | :--- |
| `gnss.enabled` | `true` | GNSS 拘束の有効化 |
| `gnss.source` | `navpvt` | データソース (`navpvt` / `navsat_fix`) |
| `gnss.topics.navpvt` | `/navpvt` | NavPVT トピック名 |
| `gnss.topics.fix` | `/gps/fix` | NavSatFix トピック名 |
| `gnss.anchor.init_distance_m` | `2.0` | 初期方位角推定に必要な移動距離 [m] |
| `gnss.lever_arm.x` / `.y` | `0.26` / `-0.13` | GNSS アンテナの `base_link` 座標系での位置 [m] (`0`/`0` で無効) |
| `gnss.validation.max_sigma_m`| `2.0` | 拘束採用を許容する最大位置標準偏差 $\sigma$ [m] |

### ナビゲーションブリッジ パラメータ (`slam_gnss_nav_bridge_node`)
| パラメータ | デフォルト | 説明 |
| :--- | :--- | :--- |
| `gnss_transform_file` | `""` | 読み込む `gnss_transform.yaml` のファイルパス (必須) |
| `gnss_input` | `navpvt` | 入力メッセージ種別 (`navpvt` / `navsatfix`) |
| `gnss_topic` | `/navpvt` | 受信する GNSS トピック名 |
| `map_frame_id` | `map` | マップ座標系のフレーム ID |
| `base_frame_id` | `base_footprint` | 車体座標系のフレーム ID (アンテナ位置の補正で、向きを取る TF の子フレーム) |
| `gps_frame_id` | `gps_link` | アンテナ座標系のフレーム ID |
| `heading_source` | `computed` | ヘディング算出元 (`computed`: 変位ベクトル / `navpvt`: 内蔵方位) |
| `heading_min_distance` | `0.6` | 変位ヘディング算出を行う最小移動量 [m] |
| `heading_smoothing_alpha` | `0.6` | Circular EMA フィルタの平滑化係数 |
| `min_publish_distance` | `1.0` | 前地点からこの距離以上移動した場合にパブリッシュ [m] |
| `max_covariance_threshold` | `49.0` | 配信を許可する最大共分散閾値 ($2 \sigma^2$) [$\mathrm{m}^2$] |
| `lever_arm_compensation` | `true` | アンテナ位置を車体中心の位置に直して配信する |
| `lever_arm_x` / `lever_arm_y` | `0.26` / `-0.13` | アンテナの車体座標系での位置 [m] (URDF の `base_footprint`→`gps_link` と合わせる) |
| `yaw_max_age_sec` | `1.0` | 車体の向きに使う TF がこれより古いときは、向きが分からないものとして扱う [s] |
| `time_offset_sec` | `0.0` | `/odom/gps` のスタンプを `now()` からこの秒数だけ過去にする (測位が届くまでの遅れの補正) [s] |
| `fix_floor_m` / `float_floor_m` / `single_floor_m` | `0.02` | 解の種類ごとの hAcc の下限 [m] |
| `fix_scale` / `float_scale` / `single_scale` | `1.0` | 解の種類ごとに分散にかける係数 |
| `accept_single` | `true` | `false` にすると、搬送波位相の解が無い (単独測位) 測位を使わない |
| `use_file_rotation` | `false` | `gnss_transform.yaml` の `rotation_rad` で map 座標を回転する。SLAM で作った地図は UTM に整合済みなので `false`、東・北に揃った地図 (シミュレータ) では `true` |

分散は `max(hAcc, 下限)² × 係数`。既定では従来と同じ `hAcc²` です。NavSatFix 入力 (`gnss_input: navsatfix`) では、搬送波位相の解が無いため、
RTK (`STATUS_GBAS_FIX` 以上) だけを Fix、それ以外を単独測位として扱い、`position_covariance[0]` の平方根を hAcc とします。
パラメータは `params/nav_bridge.yaml` にあります。
