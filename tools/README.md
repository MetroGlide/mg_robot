# 開発・デバッグ・データ解析ツール群 (tools/)

本ディレクトリには、自律移動台車ロボット MG-01 の開発、センサデータの検証、rosbag 解析、SLAM・ナビゲーションのデバッグ用共通ライブラリおよび CLI ツール群が配置されています。

---

## ディレクトリ構成

```text
tools/
├── __init__.py
├── README.md               # 本ドキュメント
├── common/                 # 共通ライブラリパッケージ
│   ├── __init__.py
│   ├── bag.py             # rosbag 操作・デシリアライズ・メタデータ
│   ├── geo.py             # WGS84/UTM 座標変換・クォータニオン/Yaw変換・大圏距離
│   ├── map.py             # 地図 YAML/画像読込・Procrustes 剛体変換推定・座標変換
│   ├── cli.py             # CLI 出力パス解決 (--output, --output-to-bag-dir, --output-dir)
│   ├── pose_graph.py      # slam_gnss_2d 出力 (pose_graph.json / gnss_transform.yaml) の読込・補間・アンカー間の座標移動
│   ├── loc_metrics.py     # 自己位置推定の評価指標 (誤差・飛び・NEES・遅れ・復旧) ※numpy のみ
│   └── faults.py          # 故障注入定義 (YAML) の読込
├── datasets/localization/ # 自己位置推定の再生評価のデータセット定義 (地図・評価走行・真値の組)
├── test/                  # tools のテスト (make test pkg=tools)
├── data/                  # 一時解析データ・可視化画像出力先 (Git追跡除外)
└── scripts/               # 実行可能 CLI スクリプト群
    ├── rosbag_summary.py            # rosbag 健全性・統計サマリー出力 (make bag-summary)
    ├── plot_gnss_trajectory.py      # GNSS 軌跡・Fix状態・精度円・地図比較 (make bag-plot-gnss)
    ├── plot_lidar_scans.py          # LiDAR スキャン点群の 2D 画像化 (make bag-plot-scans)
    ├── generate_static_transforms.py # 対応点からの UTM -> map 剛体変換 (x,y,yaw) 算出
    ├── rosbag_modify_base.py        # rosbag 内特定トピック修正 (共分散付与等)
    ├── bag_to_json.py               # rosbag メッセージの JSON ダンプ
    ├── diff_bag_list.py             # 記録対象トピックと現在アクティブなトピックの比較
    ├── eval_slam.py                 # SLAM 出力の RTK(GNSS) 比較評価 (make bag-eval-slam)
    ├── eval_localization.py         # 自己位置推定 (EKF 融合) の評価 (make bag-eval-localization)
    ├── run_localization_variant.sh  # 自己位置推定のパラメータ変種を rosbag 再生で評価 (ホスト側の入口)
    ├── localization_replay.sh       # 上記の本体 (コンテナ内で スタック起動・再生・記録・評価)
    ├── loc_init_pose.py             # 再生評価で AMCL と EKF の初期姿勢を真値から与える
    ├── loc_recorder.py              # 再生評価の出力をシミュレーション時刻で rosbag2 に記録する
    ├── loc_dataset.py               # データセット定義 (datasets/localization/) の読込
    ├── compare_localization.py      # 変種ごとの評価結果の比較表
    ├── run_slam_variant.sh          # パラメータ変種のオフラインSLAM実行〜評価までを一括実行
    ├── patch_params.py              # コメント付きパラメータ YAML の値を書き換え
    ├── record.sh                    # rosbag 記録 (MCAP)
    └── bag_play.sh                  # rosbag 再生
```

---

## 共通ライブラリ (`tools/common/`)

解析ツール間で重複しがちな定型処理（rosbag のオープン、座標変換、地図読込、出力先解決など）をモジュール化し、保守性と再利用性を高めています。
新規の解析スクリプトを作成する際も、これらのモジュールをインポートして活用してください。

### 1. `tools.common.bag` (rosbag 操作)
- **`detect_storage_id(bag_path)`**:
  - rosbag パスから MCAP (`.mcap`) または SQLite3 (`.db3`) を自動判別。
- **`load_metadata(bag_path)`**:
  - `metadata.yaml` が存在すれば安全に読み込み、辞書として返却。
- **`open_reader(bag_path, storage_id=None, topics=None)`**:
  - `rosbag2_py.SequentialReader` を初期化・オープン（トピックフィルタリング対応）。
- **`open_writer(bag_path, storage_id='mcap')`**:
  - `rosbag2_py.SequentialWriter` を初期化・オープン。
- **`MessageDeserializer`**:
  - トピック名または型名から ROS2 メッセージクラスを動的解決・キャッシュし、デシリアライズを実行。
- **`message_to_dict(msg)`**:
  - ROS2 メッセージを辞書に再帰変換（NumPy 配列やタプルも JSON シリアライズ可能化）。

### 2. `tools.common.geo` (幾何・地理座標変換)
- **`lat_lon_to_utm(lat, lon, zone=54)`**:
  - WGS84 緯度経度から UTM 座標 (Easting, Northing) [m] へ変換（`pyproj.Proj` インスタンスキャッシュ付き）。
- **`quaternion_to_yaw(qx, qy, qz, qw)`**:
  - クォータニオンから 2D 平面上の Yaw 角 [rad] を算出。
- **`haversine_distance(lat1, lon1, lat2, lon2)`**:
  - 2地点の緯度経度から球面三角法による大圏距離 [m] を算出。

### 3. `tools.common.map` (地図・座標系変換)
- **`load_map_info(map_yaml_path)`**:
  - Nav2 `map.yaml` を読み込み、画像配列 (NumPy)、origin `[x, y, yaw]`、resolution `[m/px]`、画像パスを返却。
- **`estimate_rigid_transform(utm_xy, map_xy)`**:
  - 対応点群から Procrustes 解析 (SVD) により剛体変換 `(x, y, yaw)` [UTM -> map] を推定。
- **`load_transform_from_yaml(yaml_path, label=None)`**:
  - `static_transforms.yaml` から指定 label の `(x, y, yaw)` を取得。
- **`pixel_to_map_coordinates(u, v, resolution, origin_x, origin_y, image_height)`**:
  - 画像ピクセル座標 (u, v) [左上原点] を地図実世界座標 (x, y) [m, 左下原点] に変換。

### 4. `tools.common.cli` (CLI 出力パス解決)
- **`add_output_args(parser, default_filename)`**:
  - `-o/--output`, `--output-to-bag-dir`, `--output-dir` を argparse パーサーに一括追加。
- **`resolve_output_path(bag_path, default_filename, output=None, output_to_bag_dir=False, output_dir=None)`**:
  - 優先度順に出力ファイルの絶対パスを決定し、親ディレクトリを自動作成。

### 5. `tools.common.pose_graph` (slam_gnss_2d 出力の読込)
rclpy に依存しないため、テストや rosbag を読まない解析からも使えます。
- **`load_slam_output(slam_dir)`**: `pose_graph.json` / `gnss_transform.yaml` を読み込み、ノード列 `[t, x, y, yaw]` を返却。
- **`interpolate_nodes(nodes, times)`**: ノード列を時刻で補間 (隣接ノードの間隔が大きい所は無効)。
- **`shift_nodes_to_anchor(nodes, from_transform, to_transform)`**: 2 つの SLAM 結果のアンカー (UTM) の差だけ座標を移す。別走行を、地図を作った走行の座標系で評価するときに使う。
- **`antenna_positions` / `fit_rigid` / `apply_rigid`**: GNSS アンテナ位置の算出と SE(2) 剛体整合。

### 6. `tools.common.loc_metrics` (自己位置推定の評価指標)
numpy のみに依存する指標の計算 (`compose`, `pose_errors`, `correction_jumps`, `nees`, `best_lag`, `recovery_metrics`, `count_episodes` ほか)。

### 7. `tools.common.faults` (故障注入の定義)
評価ツールと故障注入ノードが共有する故障定義 (`kidnap` / `gnss_bias` / `gnss_drop` / `odom_scale` / `scan_drop`) の読込。時刻は再生開始からの経過秒。

---

## 出力先ディレクトリと保存先切り替え

本ツール群および `make bag-*` コマンドは、出力成果物（Markdown、PNG画像等）の保存先を柔軟に制御できます。

- **デフォルト (未指定)**:
  - 対象 rosbag と同じディレクトリに保存されます（例: `summary.md`, `gnss_trajectory.png`, `lidar_scans_plot.png`）。
  - rosbag ディレクトリ内に結果を永続的に残しておきたい場合に最適です。
- **`tools/data/` への保存 (`TO_TOOLS=1` または `OUT_DIR=tools`)**:
  - `tools/data/` 配下に保存されます。
  - `tools/data/` は `.gitignore` されているため、Git作業ツリーを汚さずに一時解析データや画像を保存・確認できます。
- **任意ディレクトリへの保存 (`OUT_DIR=<dir>`)**:
  - 指定したディレクトリに出力されます（ディレクトリが存在しない場合は自動作成）。
- **特定ファイルへの直接保存 (`OUT=<file>` / `-o <file>`)**:
  - ファイル名やパスを完全に指定して出力します。

---

## 各ツールの詳細と実行例

### 1. `rosbag_summary.py` (rosbag 統計サマリー)

rosbag をストリーミング走査し、通信ヘルス（レート・ドロップ警告）および GNSS（Fixステータス分布・精度統計・ヒストグラム）、オドメトリ移動距離などを瞬時に解析・サマリーします。

```bash
# 基本実行 (端末カラー表示)
python3 tools/scripts/rosbag_summary.py /path/to/rosbag

# 全センサ詳細表示 (Odom, LiDAR, TFツリー, ログ診断エラーまで出力)
python3 tools/scripts/rosbag_summary.py /path/to/rosbag --all

# メタ情報のみ高速確認 (メッセージを全件走査せず 0.1 秒で表示)
python3 tools/scripts/rosbag_summary.py /path/to/rosbag --info-only

# Markdown / JSON 形式で保存
python3 tools/scripts/rosbag_summary.py /path/to/rosbag --format markdown -o summary.md
python3 tools/scripts/rosbag_summary.py /path/to/rosbag --format json -o summary.json

# make コマンド経由 (.env の ROSBAG_FILE を解析)
make bag-summary                   # bag ディレクトリに summary.md を保存
make bag-summary TO_TOOLS=1        # tools/data/ に summary.md を保存
make bag-summary OUT_DIR=/path/to  # 指定ディレクトリに保存
make bag-summary BAG=/path/to/bag  # 指定 rosbag を解析
```

---

### 2. `plot_gnss_trajectory.py` (GNSS軌跡・精度可視化 & 地図/オドメトリ比較)

GNSS（`/navpvt` または `/gps/fix`）の軌跡を UTM 座標系でプロットし、Fix 状態（RTK Fix: 緑、RTK Float: 橙、Single/3D: 灰、No Fix: 赤）や精度誤差円を描画します。
オドメトリ軌跡との比較や、SLAM 等で生成された占有格子地図画像との重ね合わせ描画が可能です。

```bash
# 1. GNSS 単体プロット (UTM 座標系、Fix 色分け + 精度円)
python3 tools/scripts/plot_gnss_trajectory.py /path/to/rosbag -o gnss_trajectory.png

# Fix/Float/Single の区間推移サマリーを端末に表示
python3 tools/scripts/plot_gnss_trajectory.py /path/to/rosbag --periods

# 2. オドメトリ比較 (並列形状比較)
python3 tools/scripts/plot_gnss_trajectory.py /path/to/rosbag --compare-odom -o compare_shapes.png

# 3. オドメトリ重ね合わせ (初期方位角で回転させて UTM 上に重ねてプロット)
python3 tools/scripts/plot_gnss_trajectory.py /path/to/rosbag --compare-odom --overlay --initial-yaw 141.6 -o odom_overlay.png

# 4. 地図画像オーバーレイ (OccupancyGrid 地図画像の上に GNSS 軌跡を重ねてプロット)
# static_transforms YAML がある場合:
python3 tools/scripts/plot_gnss_trajectory.py /path/to/rosbag --mode map --map /root/ros2_data/map/map.yaml --static-transforms /root/ros2_data/map/gnss_to_map_static_transforms.yaml --label label_1 -o map_overlay.png

# static_transforms YAML がない場合 (自動フィットまたは初期方位角で歪んだ地図でも重ね合わせ可能):
python3 tools/scripts/plot_gnss_trajectory.py /path/to/rosbag --mode map --map /root/ros2_data/map/map.yaml --initial-yaw 141.6 -o map_overlay.png

# 5. GNSS 精度円 (NavPVT hAcc) の描画
python3 tools/scripts/plot_gnss_trajectory.py /path/to/rosbag --accuracy-circles -o gnss_circles.png
# 精度円を定数倍（例: 10倍）に拡大描画、サンプリング間隔や最大半径 [m] の調整
python3 tools/scripts/plot_gnss_trajectory.py /path/to/rosbag --circle-scale 10.0 --circle-step 10 -o gnss_circles_x10.png

# 6. make コマンド経由
make bag-plot-gnss                            # bag ディレクトリに gnss_trajectory.png を保存
make bag-plot-gnss TO_TOOLS=1                 # tools/data/ に gnss_trajectory.png を保存
make bag-plot-gnss CIRCLES=1                  # 精度円 (hAcc) を等倍で描画
make bag-plot-gnss SCALE=10 TO_TOOLS=1        # 精度円を10倍に拡大して tools/data/ に保存
make bag-plot-gnss BAG=/path/to/bag OPTS="--compare-odom"
```

---

### 3. `plot_lidar_scans.py` (LiDARスキャン点群・SLAMノード2D画像化)

rosbag 内のオドメトリ移動量（並進 0.2m / 回転 0.1rad 等）ごとに SLAM ノードを抽出し、指定したノード区間の LiDAR スキャン点群をオドメトリ座標系で 2D 画像（PNG）としてプロットします。
局所的な障害物の形状確認や、スキャンマッチングのズレ・歪みの原因調査に役立ちます。

```bash
# 先頭 30 ノードのスキャン点群を画像化
python3 tools/scripts/plot_lidar_scans.py /path/to/rosbag -o scans_preview.png

# 特定のノード番号範囲 (例: ノード 705 から 715) を指定して画像化
python3 tools/scripts/plot_lidar_scans.py /path/to/rosbag --nodes 705:715 -o nodes_705_715.png

# 解像度や最大プロット距離を変更
python3 tools/scripts/plot_lidar_scans.py /path/to/rosbag --nodes 100:130 --resolution 0.03 --max-range 15.0 -o detail_scans.png

# make コマンド経由
make bag-plot-scans                          # bag ディレクトリに lidar_scans_plot.png を保存
make bag-plot-scans TO_TOOLS=1 NODES=1:20    # tools/data/ に先頭20ノードを描画保存
make bag-plot-scans BAG=/path/to/bag NODES=705:715
```

---

### 4. `generate_static_transforms.py` (地図とGNSSの剛体変換算出)

地図画像上のピクセル座標と、対応する現地での GNSS 緯度経度から、UTM 座標系から地図座標系への剛体変換（回転 + 並進 `[x, y, yaw]`）を Procrustes 解析（SVD）により推定します。

```bash
python3 tools/scripts/generate_static_transforms.py \
  --map-dir /root/ros2_data/map \
  --base-yaml-name gnss_to_map_base_data.yaml \
  --out-yaml-name gnss_to_map_static_transforms.yaml \
  --utm-zone 54
```

---

### 5. `rosbag_modify_base.py` (rosbag トピック内容修正)

rosbag 内の特定トピックのメッセージ内容（例: `/odom` への共分散行列の付与、`/scan_front_lidar` の `frame_id` の置換など）を書き換えて、新しい rosbag を生成します。

```bash
python3 tools/scripts/rosbag_modify_base.py -i /path/to/input_bag.mcap -o /path/to/output_bag.mcap
```

---

### 6. `bag_to_json.py` (rosbag の JSON ダンプ)

rosbag 内の特定トピックまたは全トピックのメッセージを JSON 形式にシリアライズしてダンプします。

```bash
# 特定トピックを抽出して JSON 出力
python3 tools/scripts/bag_to_json.py --bag_path /path/to/rosbag --topics /odom /tf_static --output_dir tools/data/
```

---

### 7. `diff_bag_list.py` (トピック記録リスト比較)

記録対象トピック一覧テキストファイルと、現在実行中の ROS2 システム上のトピック一覧（`ros2 topic list`）を比較し、差分をターミナルに色付き表示します。

```bash
python3 tools/scripts/diff_bag_list.py /path/to/record_topics.txt
```

---

### 8. `record.sh` & `bag_play.sh` (rosbag 記録・再生)

```bash
# SLAM・TF・GNSS の軽量版記録 (約0.28GB/30分)
./tools/scripts/record.sh -m slam

# カメラ画像・デプス・IMU 含む全センサ版記録 (約39GB/30分)
./tools/scripts/record.sh -m all

# rosbag 再生
./tools/scripts/bag_play.sh /path/to/rosbag
```

### 9. `eval_slam.py` (SLAM 出力の RTK(GNSS) 比較評価)

slam_gnss_2d のオフライン出力 (`pose_graph.json` / `gnss_transform.yaml` / `map.yaml`) を、rosbag の `/navpvt` と比較して評価します。
GNSS アンテナ位置 (`base_link` + `R(yaw)` × レバーアーム) と NavPVT 位置の残差を、RTK 状態 (Fix / Float / None) ごとに集計します。

```bash
make bag-eval-slam SLAM_DIR=/root/ros2_data/rosbag/<bag>/eval/<name>
make bag-eval-slam SLAM_DIR=<dir> TO_TOOLS=1 OPTS="--time-offset 0.2"
```

| 区分 | 内容 |
| :--- | :--- |
| `raw` | `gnss_transform.yaml` のアンカーをそのまま使った残差 (UTM 整合性) |
| `aligned` | Fix 全体へ SE(2) 剛体整合した後の残差 (地図の形状精度) |
| Fix 区間ごと | 連続した Fix 区間ごとに個別整合した残差 (局所形状精度) |
| スキャンマッチング品質 | 逐次エッジのスコア (旋回中を別集計) |
| 地図 | 占有セル数、平均壁厚 (小さいほど壁が鮮鋭) |

- `--lever-arm X Y`: `base_link` から `gps_link` へのオフセット [m] (既定 `0.26 -0.13`)
- `--time-offset`: GNSS 時刻に加えるオフセット [s]

### 10. `run_slam_variant.sh` & `patch_params.py` (パラメータ変種の比較)

`params/slam_gnss_2d.yaml` を複製して値を書き換え、`.env` の `ROSBAG_FILE` でオフラインSLAMを実行し、地図と `pose_graph.json` を
`<bag>/eval/<名前>/` に保存して `eval_slam.py` の評価結果と処理時間を表示します。

```bash
tools/scripts/run_slam_variant.sh <名前> [key.path=value ...]
# 例: デスキューを無効にして比較
tools/scripts/run_slam_variant.sh ds_off deskew.enabled=false
# 区間指定 (launch 引数の追加)
EXTRA_OPTS="start_time:=750.0 end_time:=1000.0" tools/scripts/run_slam_variant.sh short keyframe.min_translation=0.3
```

- 書き換えた YAML は `tools/data/variants/<名前>.yaml` (Git 追跡除外) に保存されます。
- 元の値が小数のパラメータに整数を指定した場合は `.0` を補います (ROS2 のパラメータ型は厳密なため)。
- ノードが異常終了した場合や 20 分以内に完了しない場合はエラー終了します。

### 11. `eval_localization.py` (自己位置推定の評価)

オドメトリ + AMCL + GNSS を EKF で融合した自己位置推定の出力を rosbag から評価します。
実機のナビ走行の bag と、再生評価の出力 bag のどちらにも使えます。

```bash
# 真値なし (滑らかさ・EKF と AMCL の挙動)
make bag-eval-localization BAG=/root/ros2_data/rosbag/<bag> TO_TOOLS=1
# 真値あり (評価 bag の SLAM 出力)
make bag-eval-localization BAG=<bag> GT_DIR=<評価 bag の SLAM 出力>
# 別走行の評価: 地図を作った走行の SLAM 出力も渡し、真値をその座標系へ移す
make bag-eval-localization BAG=<bag> GT_DIR=<評価 bag の SLAM 出力> MAP_GT_DIR=<地図の SLAM 出力>
```

推定姿勢は `/tf` の `map→odom` と `odom→base_footprint` を合成して求めます (Nav2 が使う姿勢と同じ)。

| 指標 | 内容 |
| :--- | :--- |
| 滑らかさ | `map→odom` の更新でロボットの姿勢が飛ぶ大きさ (AMCL / GNSS の補正が Nav2 に与える影響) |
| 精度 | 真値 (`pose_graph.json`) との位置・yaw の誤差。全体と時間窓ごと |
| NEES | EKF の共分散が誤差に見合うか (自由度 3 の一貫した推定なら平均 3、95% が 7.8 以下) |
| EKF と odom | EKF の速度とホイールオドメトリ速度の差・遅れ、EKF の σ |
| AMCL | σ、遅延 (bag の記録時刻と header が同じ時計のときのみ)、届いた時点の推定姿勢との差 |
| 故障注入 (`FAULTS=`) | 故障ごとの最大誤差・復旧までの時間・検知までの時間、故障のない区間での誤検知の回数 |

- `--start` / `--end`: 評価する区間 (先頭からの経過秒)
- `--ok-threshold` / `--hold-sec`: 復旧とみなす位置誤差と、その誤差以下でいる時間
- `--map-frame` / `--odom-frame` / `--base-frame` / `--*-topic`: フレーム名とトピック名の変更

### 12. `run_localization_variant.sh` (自己位置推定の再生評価)

自己位置推定のスタック (`map_server` / AMCL / GNSS ブリッジ / EKF) だけを起動し、rosbag のセンサデータ
(`/odom` `/scan_top_lidar` `/navpvt` `/gps/fix`) を再生して、推定結果を `eval_localization.py` で評価します。
パラメータを変えた「変種」を同じ条件で試して比べるためのものです。実機は使いません。

```bash
# 現状のパラメータでベースラインを測る (既定は短縮版データセット、3 回)
tools/scripts/run_localization_variant.sh baseline
# パラメータを書き換えた変種 (プレフィックス: ekf. / amcl. / bridge.)
tools/scripts/run_localization_variant.sh lag_on ekf.smooth_lagged_data=true ekf.history_length=1.0
tools/scripts/run_localization_variant.sh light_amcl amcl.max_particles=2000 amcl.max_beams=240 --cpus 4
# 本番のデータセット (10 分)
tools/scripts/run_localization_variant.sh baseline --dataset map043837_eval051635
# 変種同士を比べる
docker run --rm -v $PWD:/app -v ~/ros2_data:/root/ros2_data mg_develop:latest \
  python3 /app/tools/scripts/compare_localization.py <出力ディレクトリ1> <出力ディレクトリ2>
```

| オプション | 内容 |
| :--- | :--- |
| `--dataset <名前\|パス>` | `tools/datasets/localization/` の定義。既定 `map043837_eval051635_short` |
| `--runs N` | 繰り返す回数 (既定 3)。AMCL は乱数を使うため、分布で比べる |
| `--init gt\|gnss` | 初期姿勢。`gt` は真値を与える (既定)、`gnss` は GNSS による初期化に任せて初期化も評価する |
| `--start S` / `--duration D` | 評価する区間 (データセットの値を上書き) |
| `--rate R` | 再生速度 (既定 1.0)。処理が間に合わなくなるため実時間が基本 |
| `--cpus N` | コンテナが使える CPU 数。開発 PC は実機より速いため、実機相当に絞って AMCL の遅延を見る |

- 出力: `<評価 bag>/eval_loc/<データセット>/<名前>/{params/, run_N/, summary.md}`。`run_N/eval_localization.md` に試行ごとの評価、`summary.md` に試行をまとめた表が入ります。
- 起動前に、関連パッケージ (`slam_gnss_2d` `mg_msgs` `mg_bringup` `mg_navigation` `mg_drivers`) をコンテナ内で増分ビルドします。ソースの変更はイメージの再ビルドなしで反映されます。
- 書き換えられるのは 1 行の値 (スカラー) だけです。キーが元のファイルに無い場合はエラーになります。
- ROS の通信は `ROS_LOCALHOST_ONLY=1` とランダムな `ROS_DOMAIN_ID` で隔離しており、実機や他のコンテナと混ざりません。

#### データセット

`tools/datasets/localization/*.yaml` に、地図 (`localization_yaml` / `gnss_transform` / `slam_dir`)、評価する走行 (`bag` / `slam_dir`)、区間 (`start_offset` / `duration`) を定義します。

| データセット | 内容 |
| :--- | :--- |
| `same_run_043837` | 地図を作った走行をそのまま評価。手早いチェック用。地図がその走行に合わせ込まれているため楽観的な結果になる |
| `map043837_eval051635` | 別走行 (地図は 043837、評価は 051635 の 470〜1070 s)。本番用 |
| `map043837_eval051635_short` | 上の 470〜650 s だけ。動作確認と粗い比較用 |

別走行では、評価する走行の SLAM 出力 (`pose_graph.json`) を、UTM 経由で地図の座標系へ移して真値にします。次の点に注意してください。
- 真値と地図の位置合わせの精度は、2 つの SLAM を作ったときの GNSS の精度で決まります。RTK Fix が少ない bag (051635 は Fixed 0%) では約 1 m ずれるため、絶対値ではなく**変種同士の相対比較**に使ってください。評価レポートには、走行全体を SE(2) 整合した後の誤差 (地図に対する整合性) を併記します。
- 評価する走行が、地図の範囲外を通る区間では AMCL が使えず破綻します。区間は地図がカバーする範囲に絞ってください (051635 は地図 043837 に対して 0〜138 s / 462〜1586 s / 1636〜1856 s が範囲内)。
- `--init gt` では、初期姿勢を与えた直後の 10 秒 (`EVAL_SKIP`) を評価から除きます。

### テスト

`tools/test/` に共通モジュールと指標の単体テストがあります。

```bash
make test pkg=tools
```
