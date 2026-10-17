# rosbag の確認・可視化・記録

rosbag の統計、GNSS 軌跡、LiDAR スキャンの画像化、JSON ダンプ、イベントログ、記録と再生のツール。一覧は [tools の README](../README.md)。出力先の切り替えは README の「出力先ディレクトリと保存先切り替え」。

## `rosbag_summary.py` (rosbag 統計サマリー)

rosbag をストリーミング走査し、通信ヘルス（レート・ドロップ警告）および GNSS（Fixステータス分布・精度統計・ヒストグラム）、オドメトリ移動距離などを瞬時に解析・サマリーします。

```bash
# 基本実行 (端末カラー表示)
python3 tools/scripts/rosbag_summary.py /path/to/rosbag

# 全センサ詳細表示 (Odom, LiDAR, TFツリー, ログ診断エラーまで出力)
python3 tools/scripts/rosbag_summary.py /path/to/rosbag --all

# メタ情報のみ高速確認 (メッセージを全件走査せず すぐに表示)
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

**状態とイベント** (`/diagnostics_agg` または `/diagnostics`、`/rosout`、`/events` が入っているとき)。内容があれば、端末と Markdown の末尾に「Status Timeline & Events」の節を出します。

| 内容 | 元 |
| :--- | :--- |
| OK 以外になった診断の一覧 (最後と最悪のレベル、WARN・ERROR・STALE だった秒数) | `/diagnostics_agg` があればそれ、無ければ `/diagnostics` |
| レベルの変化 (時刻・診断・変化・メッセージ。古い順で 30 件まで) | 同上 |
| イベント (`evt=...`) の数と、WARN 以上のイベントの一覧 | `/rosout` と `/events` (同じイベントは 1 回) |

集計は `tools/common/status_timeline.py` (ROS 不要)。イベントの書式は [doc/monitoring.md](../../doc/monitoring.md)。詳しく見るときは `make log-events BAG=<bag>` で時系列にできます。

## `plot_gnss_trajectory.py` (GNSS軌跡・精度可視化 & 地図/オドメトリ比較)

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

## `plot_lidar_scans.py` (LiDARスキャン点群・SLAMノード2D画像化)

rosbag 内のオドメトリ移動量ごとに SLAM ノードを抽出し、指定したノード区間の LiDAR スキャン点群をオドメトリ座標系で 2D 画像（PNG）としてプロットします。
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

## `rosbag_modify_base.py` (rosbag トピック内容修正)

rosbag 内の特定トピックのメッセージ内容（例: `/odom` への共分散行列の付与、`/scan_front_lidar` の `frame_id` の置換など）を書き換えて、新しい rosbag を生成します。

```bash
python3 tools/scripts/rosbag_modify_base.py -i /path/to/input_bag.mcap -o /path/to/output_bag.mcap
```

## `bag_to_json.py` (rosbag の JSON ダンプ)

rosbag 内の特定トピックまたは全トピックのメッセージを JSON 形式にシリアライズしてダンプします。

```bash
# 特定トピックを抽出して JSON 出力
python3 tools/scripts/bag_to_json.py --bag_path /path/to/rosbag --topics /odom /tf_static --output_dir tools/data/
```

## `event_timeline.py` (イベントログの時系列)

イベントログ (`evt=<domain>.<event> key=value ...`。書式は [doc/monitoring.md](../../doc/monitoring.md)) を、コンテナのログと rosbag から集めて、時刻順の表 (Markdown または CSV) にします。

| 入力 | 内容 |
| :--- | :--- |
| `--logs <dir...>` | `make log-collector` が書き出したログ (`<サービス>/<日付>.log`) と、`make logs-export` のログ (`<日時>/<サービス>.log`) |
| `--bag <rosbag>` | rosbag の `/events` と `/rosout` (`rcl_interfaces/Log`) |

並べる時刻は、ログ本文の epoch (ノードが記録した時刻) です。同じイベントが複数の入力にあっても、1 回だけ出ます。

```bash
make log-events SINCE=2026-10-11T00:00:00+09:00 LEVEL=WARN TZ=+09:00
make log-events DOMAIN="driver watchdog" OUT=/app/tools/data/events.md
make log-events BAG=/root/ros2_data/rosbag/xxx LOGS=/root/ros2_data/logs FORMAT=csv OUT=/app/tools/data/events.csv
```

絞り込みは `--since` / `--until` / `--level` / `--domain` / `--service`。表示のタイムゾーンは `--tz` (`UTC`、`+09:00`、`Asia/Tokyo`)。

## `diff_bag_list.py` (トピック記録リスト比較)

記録対象トピック一覧テキストファイルと、現在実行中の ROS2 システム上のトピック一覧（`ros2 topic list`）を比較し、差分をターミナルに色付き表示します。

```bash
python3 tools/scripts/diff_bag_list.py /path/to/record_topics.txt
```

## `record.sh` & `bag_play.sh` (rosbag 記録・再生)

```bash
# SLAM・TF・GNSS の軽量版記録 (軽量版)
./tools/scripts/record.sh -m slam

# カメラ画像・デプス・IMU 含む全センサ版記録 (全センサ版。容量は大きい)
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
| `mg_utils/params/record_topics_monitoring.txt` | **状態監視とイベントのトピック**。どのモードにも自動で加わる (`-n` で外す)。`/diagnostics`、`/diagnostics_agg`、`/rosout`、`/events`、`/localization/status` など。数十 KB/s 以下。`mg_utils` の `record_bag.launch.py` も同じファイルを使う |

監視用のトピックを残すと、後から次のことができます。生の `/diagnostics` を `diagnostic_aggregator` にかけ直して集約を作り直す。`/rosout` と `/events` のイベントを `make log-events BAG=<bag>` で時系列にする。重複するトピックは 1 回だけ記録します。
