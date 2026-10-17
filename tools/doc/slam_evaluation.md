# SLAM の評価と変種の比較

SLAM の出力を RTK (GNSS) と比べる評価と、パラメータの変種の比較。一覧は [tools の README](../README.md)。

## `eval_slam.py` (SLAM 出力の RTK(GNSS) 比較評価)

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

- `--lever-arm X Y`: `base_link` から `gps_link` へのオフセット [m]
- `--time-offset`: GNSS 時刻に加えるオフセット [s]

## `run_slam_variant.sh` & `patch_params.py` (パラメータ変種の比較)

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
