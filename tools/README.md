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
│   ├── odom_calib.py      # ホイールオドメトリ校正の計算 (窓ごとの相対移動の当てはめ) ※numpy のみ
│   ├── faults.py          # 故障注入定義 (YAML) の読込
│   └── fault_apply.py     # 故障注入の計算 (numpy のみ。fault_injector.py が使う)
├── datasets/localization/ # 自己位置推定の再生評価のデータセット定義 (地図・評価走行・真値の組)
├── doc/                   # 詳細ドキュメント (localization_evaluation.md)
├── test/                  # tools のテスト (make test pkg=tools。[テスト](#テスト))
├── data/                  # 一時解析データ・可視化画像出力先 (Git追跡除外)
└── scripts/               # 実行可能 CLI スクリプト群
    ├── rosbag_summary.py            # rosbag 健全性・統計サマリー出力 (make bag-summary)
    ├── bag_summary.sh               # 上のラッパー (引数なしで使い方を表示)
    ├── plot_gnss_trajectory.py      # GNSS 軌跡・Fix状態・精度円・地図比較 (make bag-plot-gnss)
    ├── plot_lidar_scans.py          # LiDAR スキャン点群の 2D 画像化 (make bag-plot-scans)
    ├── generate_static_transforms.py # 対応点からの UTM -> map 剛体変換 (x,y,yaw) 算出
    ├── make_sim_gnss_transform.py   # シミュレータのワールド SDF から、GNSS ブリッジ用の gnss_transform.yaml を作る
    ├── rosbag_modify_base.py        # rosbag 内特定トピック修正 (共分散付与等)
    ├── bag_to_json.py               # rosbag メッセージの JSON ダンプ
    ├── diff_bag_list.py             # 記録対象トピックと現在アクティブなトピックの比較
    ├── eval_slam.py                 # SLAM 出力の RTK(GNSS) 比較評価 (make bag-eval-slam)
    ├── eval_localization.py         # 自己位置推定 (EKF 融合) の評価 (make bag-eval-localization)
    ├── run_localization_variant.sh  # 自己位置推定のパラメータ変種を rosbag 再生で評価 (ホスト側の入口)
    ├── localization_replay.sh       # 上記の本体 (コンテナ内で スタック起動・再生・記録・評価)
    ├── loc_init_pose.py             # 再生評価で AMCL と EKF の初期姿勢を真値から与える
    ├── loc_recorder.py              # 再生評価の出力をシミュレーション時刻で rosbag2 に記録する
    ├── fault_injector.py            # 再生評価で、センサ入力に故障を注入する (--faults。[doc](./doc/localization_evaluation.md#センサ故障の注入))
    ├── loc_dataset.py               # データセット定義 (datasets/localization/) の読込
    ├── compare_localization.py      # 変種ごとの評価結果の比較表
    ├── calib_wheel_odom.py          # ホイールオドメトリのスケール・バイアス・遅れを走行ログから推定
    ├── run_slam_variant.sh          # パラメータ変種のオフラインSLAM実行〜評価までを一括実行
    ├── patch_params.py              # コメント付きパラメータ YAML の値を書き換え
    ├── record.sh                    # rosbag 記録 (MCAP)
    ├── bag_play.sh                  # rosbag 再生
    └── record_topics_{slam,all,sim}.txt, record_topics.txt  # 記録するトピックの一覧 (下記)
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

### 8. `tools.common.fault_apply` (故障注入の計算)
故障の作用の判定と、スキャンの欠測・GNSS のバイアスなどの計算。numpy のみに依存し、`fault_injector.py` が使う。

---

## 出力先ディレクトリと保存先切り替え

本ツール群および `make bag-*` コマンドは、出力成果物（Markdown、PNG画像等）の保存先を柔軟に制御できます。

- **デフォルト (未指定)**:
  - 対象 rosbag と同じディレクトリに保存されます（例: `summary.md`, `gnss_trajectory.png`, `lidar_scans_plot.png`）。
  - rosbag ディレクトリ内に結果を永続的に残しておきたい場合に最適です。
- **`tools/data/` への保存 (`TO_TOOLS=1`、`TOOLS=1`、または `OUT_DIR=tools`)**:
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

# 5.5. 地図群 + gnss_transform.yaml (ナビゲーションの slam_gnss_nav_bridge と同じ変換) での重ね描き
# --map は複数指定でき、--map-list で map_list.txt の地図をまとめて指定できる。
# --gnss-transform 指定時は UTM ゾーンもそのファイルの値を使い、/odom は描かない。
python3 tools/scripts/plot_gnss_trajectory.py /path/to/rosbag --map-list $MAP_PATH/map_list.txt --gnss-transform $MAP_PATH/gnss_transform.yaml --accuracy-circles -o gnss_on_map.png

# 6. make コマンド経由
make bag-plot-gnss-map                        # MAP_PATH の地図群 + gnss_transform.yaml で gnss_on_map.png を bag ディレクトリに保存
make bag-plot-gnss-map TO_TOOLS=1 CIRCLES=1   # tools/data/ に精度円つきで保存 (MAP_LIST= / GNSS_TRANSFORM= で別ファイルを指定可)
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

### 4-2. `make_sim_gnss_transform.py` (シミュレータの gnss_transform.yaml)

シミュレータのワールド SDF の `spherical_coordinates` (ワールド原点の緯度経度) から、GNSS ブリッジ (`slam_gnss_nav_bridge_node`) が使う `gnss_transform.yaml` を作ります。
シミュレータの地図は、東・北に揃っていて、UTM のグリッドとは子午線収束角の分だけずれます。そのずれを `map_rotation_rad` に入れます ([slam_gnss_2d/doc/gnss_transform.md](../slam_gnss_2d/doc/gnss_transform.md))。

```bash
python3 tools/scripts/make_sim_gnss_transform.py mg_simulation/worlds/warehouse.sdf -o mg_simulation/maps/warehouse/gnss_transform.yaml
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

記録するトピックは、モードごとの一覧ファイル (`scripts/` 配下) で決まります。`-f <file>` で別の一覧も指定できます。

| ファイル | 使われ方 |
| :--- | :--- |
| `record_topics_slam.txt` | `-m slam` (軽量版) |
| `record_topics_all.txt` | `-m all` (全センサ版) |
| `record_topics.txt` | モードの一覧が見つからないときのフォールバック。`diff_bag_list.py` の例にも使う |
| `record_topics_sim.txt` | シミュレータ用の一覧。`record.sh` は自動では読まない (`-f` で指定する) |

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

オドメトリ + AMCL + GNSS を EKF で融合した自己位置推定の出力を、rosbag から評価します (滑らかさ、真値との誤差、NEES、AMCL の遅延、故障からの復旧)。
実機のナビ走行の bag と、再生評価の出力 bag のどちらにも使えます。

```bash
make bag-eval-localization BAG=/root/ros2_data/rosbag/<bag> TO_TOOLS=1                       # 真値なし
make bag-eval-localization BAG=<bag> GT_DIR=<評価 bag の SLAM 出力>                           # 真値あり
make bag-eval-localization BAG=<bag> GT_DIR=<SLAM 出力> MAP_GT_DIR=<地図の SLAM 出力>        # 別走行の評価
```

指標・オプションの詳細は [doc/localization_evaluation.md](./doc/localization_evaluation.md#eval_localizationpy-自己位置推定の評価)。

### 12. `run_localization_variant.sh` (自己位置推定の再生評価)

自己位置推定のスタックだけを起動し、rosbag のセンサデータを再生して、パラメータを変えた「変種」を同じ条件で比べます (センサ故障の注入にも対応)。実機は使いません。

```bash
tools/scripts/run_localization_variant.sh baseline                                          # ベースライン (短縮版データセット、3 回)
tools/scripts/run_localization_variant.sh lag_on ekf.smooth_lagged_data=true ekf.history_length=1.0   # 変種 (プレフィックス: ekf. / amcl. / bridge. / odom. / sup.)
```

オプション、評価の進め方、センサ故障の注入、データセットは [doc/localization_evaluation.md](./doc/localization_evaluation.md#run_localization_variantsh-自己位置推定の再生評価) を参照してください。

### 13. `calib_wheel_odom.py` (ホイールオドメトリの校正)

走行ログの `/odom` (**補正前の生の姿勢**) と、slam_gnss_2d の SLAM 出力 (`pose_graph.json`) を比べて、
ホイールオドメトリの誤差を推定します。全体を一度に積算すると誤差が累積して崩れるため、真値の経路で
5 m ごとに区切った窓について、窓の始点から終点への相対移動を比べ、Huber 損失で外れ値の窓の影響を抑えます。

```bash
docker run --rm -v $PWD:/app -v ~/ros2_data:/root/ros2_data mg_develop:latest bash -c \
  "source /opt/ros/humble/setup.bash; python3 /app/tools/scripts/calib_wheel_odom.py <bag> --slam-dir <SLAM 出力> -o wheel_odom_corrector.yaml"
```

| 推定するもの | 意味 |
| :--- | :--- |
| `k_v` | 並進のスケール (車輪半径) |
| `k_w` | 旋回のスケール (実効トレッド幅) |
| `yaw_bias_per_meter` | 走行距離あたりに曲がる量 [rad/m] (左右の車輪半径差) |
| `time_offset` | オドメトリの遅れ [s]。損失が最小になる遅れを探索する |
| `covariance_vx` / `covariance_vyaw` | EKF に入れる速度の共分散の目安。補正後の窓の平均速度の誤差 σ を `--inflate` 倍 (既定 3) して二乗 |

- 出力の YAML は `mg_drivers/params/wheel_odom_corrector.yaml` の形式で、そのまま補正ノードのパラメータになります。
- 推定値は前半・後半でも別々に出るので、安定しているか確認してください。路面や荷重が違う走行では、走行ごとに推定して差を見ます。
- 補正ノードを通した bag (補正済みの `/odom`) には使えません。

### テスト

`tools/test/` に、共通モジュールと指標の単体テストがあります。

```bash
make test pkg=tools
```

| テスト | 対象 |
| :--- | :--- |
| `test_loc_metrics.py` | 自己位置推定の評価指標 (`common/loc_metrics.py`) |
| `test_odom_calib.py` | ホイールオドメトリの校正の計算 (`common/odom_calib.py`) |
| `test_faults.py`、`test_fault_apply.py` | 故障注入の定義の読込と計算 |
| `test_pose_graph.py` | `pose_graph.json` の読込・補間 |
| `test_map_helpers.py` | 地図の読込と `map_list.txt` |
| `test_compare_localization.py` | 変種の比較表 |
| `test_patch_params.py` | パラメータ YAML の書き換え |
| `test_sim_gnss_transform.py` | シミュレータの `gnss_transform.yaml` の生成 |
