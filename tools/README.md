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
│   ├── status_timeline.py # 診断のレベルの遷移とイベントの集計・整形 ※ROS 不要 (rosbag_summary.py が使う)
│   ├── event_timeline.py  # イベントログ (evt=...) の解析・結合・絞り込み・整形 ※ROS 不要。書式の読み取りは mg_utils.event_log
│   ├── faults.py          # 故障注入定義 (YAML) の読込
│   ├── fault_apply.py     # 故障注入の計算 (numpy のみ。fault_injector.py が使う)
│   └── offline_faults.py  # 故障注入定義を、自己位置の監督のオフライン評価の定義に直す
├── datasets/localization/ # 自己位置推定の再生評価のデータセット定義 (地図・評価走行・真値の組)
├── doc/                   # ツールごとの詳細 (下の「ドキュメント」)
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
    ├── event_timeline.py            # イベントログをコンテナのログと rosbag から集めて時刻順の表にする (make log-events)
    ├── diff_bag_list.py             # 記録対象トピックと現在アクティブなトピックの比較
    ├── eval_slam.py                 # SLAM 出力の RTK(GNSS) 比較評価 (make bag-eval-slam)
    ├── eval_localization.py         # 自己位置推定 (EKF 融合) の評価 (make bag-eval-localization)
    ├── run_localization_variant.sh  # 自己位置推定のパラメータ変種を rosbag 再生で評価 (ホスト側の入口)
    ├── localization_replay.sh       # 上記の本体 (コンテナ内で スタック起動・再生・記録・評価)
    ├── loc_init_pose.py             # 再生評価で AMCL と EKF の初期姿勢を真値から与える
    ├── loc_recorder.py              # 再生評価の出力をシミュレーション時刻で rosbag2 に記録する
    ├── fault_injector.py            # 再生評価で、センサ入力に故障を注入する (--faults。[doc](./doc/localization_evaluation.md#センサ故障の注入))
    ├── eval_localization_offline.py # 自己位置の監督を、再生評価の記録でオフライン評価する ([doc](./doc/localization_evaluation.md#監督のオフライン評価))
    ├── loc_dataset.py               # データセット定義 (datasets/localization/) の読込
    ├── compare_localization.py      # 変種ごとの評価結果の比較表
    ├── calib_wheel_odom.py          # ホイールオドメトリのスケール・バイアス・遅れを走行ログから推定
    ├── run_slam_variant.sh          # パラメータ変種のオフラインSLAM実行〜評価までを一括実行
    ├── patch_params.py              # コメント付きパラメータ YAML の値を書き換え
    ├── record.sh                    # rosbag 記録 (MCAP。状態監視のトピックも加える。-n で外す)
    ├── bag_play.sh                  # rosbag 再生
    └── record_topics_{slam,all,sim}.txt, record_topics.txt  # 記録するトピックの一覧 ([bag_tools.md](./doc/bag_tools.md))
```

## ドキュメント

使い方と実行例は `doc/` にツールの種類ごとに分けている。

| ファイル | ツール |
| :--- | :--- |
| [doc/bag_tools.md](./doc/bag_tools.md) | `rosbag_summary.py`、`plot_gnss_trajectory.py`、`plot_lidar_scans.py`、`rosbag_modify_base.py`、`bag_to_json.py`、`event_timeline.py`、`diff_bag_list.py`、`record.sh`、`bag_play.sh` |
| [doc/gnss_transform_tools.md](./doc/gnss_transform_tools.md) | `generate_static_transforms.py`、`make_sim_gnss_transform.py` |
| [doc/slam_evaluation.md](./doc/slam_evaluation.md) | `eval_slam.py`、`run_slam_variant.sh`、`patch_params.py` |
| [doc/localization_evaluation.md](./doc/localization_evaluation.md) | `eval_localization.py`、`run_localization_variant.sh`、故障注入、`eval_localization_offline.py` |
| [doc/wheel_odom_calibration.md](./doc/wheel_odom_calibration.md) | `calib_wheel_odom.py` |
| [doc/common_library.md](./doc/common_library.md) | `tools/common/` の共通モジュール |

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

## テスト

`tools/test/` に、共通モジュールと指標の単体テストがあります。

```bash
make test pkg=tools
```

| テスト | 対象 |
| :--- | :--- |
| `test_loc_metrics.py` | 自己位置推定の評価指標 (`common/loc_metrics.py`) |
| `test_odom_calib.py` | ホイールオドメトリの校正の計算 (`common/odom_calib.py`) |
| `test_event_timeline.py` | イベントログの解析・結合・絞り込み・整形 (`common/event_timeline.py`) |
| `test_status_timeline.py` | 診断のレベルの遷移とイベントの集計・整形 (`common/status_timeline.py`) |
| `test_faults.py`、`test_fault_apply.py` | 故障注入の定義の読込と計算 |
| `test_offline_faults.py` | 故障注入の定義の、監督のオフライン評価への変換 |
| `test_pose_graph.py` | `pose_graph.json` の読込・補間 |
| `test_map_helpers.py` | 地図の読込と `map_list.txt` |
| `test_compare_localization.py` | 変種の比較表 |
| `test_patch_params.py` | パラメータ YAML の書き換え |
| `test_sim_gnss_transform.py` | シミュレータの `gnss_transform.yaml` の生成 |
