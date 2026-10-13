# LiDAR の前処理 (laser_filters)

上 LiDAR の生のスキャン `/scan_top_lidar` から、使ってはいけない点を落として `/scan_filtered/scan_top_lidar` に出す。ノード名は `top_laser_filter`。設定は `params/top_laser_filter.yaml` (値はここが正)、起動は `launch/laser_filters.launch.py`。

## 何を落とすか

| フィルタ | 落とす点 | 理由 |
| :--- | :--- | :--- |
| `LaserScanRangeFilter` | 近すぎる点と、遠すぎる点 | 近距離のノイズが障害物として記録されるのを防ぐ |
| `LaserScanBoxFilter` | `base_footprint` 基準の箱の中の点 | 自車の支柱やケーブルが、障害物として見えるのを防ぐ |

落とした点は、範囲のフィルタでは 0.0、箱のフィルタでは NaN になる。コストマップ (`top_obstacle_layer`) と `collision_monitor` は、どちらも無効な点として読み捨てる。

## 誰が使うか

| 使う | 入力 |
| :--- | :--- |
| コストマップ (`local_costmap`、`global_costmap`)、`collision_monitor` | 前処理後の `/scan_filtered/scan_top_lidar` |
| AMCL、SLAM | 生の `/scan_top_lidar` (地図との照合に、自車の点は影響しないので、加工しない) |

rosbag の再生では、`mg_navigation` の `bag_play.launch.py` が、bag の生のスキャンからこの前処理を動かす。

## 他の設定との関係

箱の大きさと位置は、次と揃える。関係は `mg_navigation/test/test_nav2_params_consistency.py` が検査する。

- 箱は、`nav2_params.yaml` の footprint を含む (余白を足した大きさ)。
- `collision_monitor` の `PolygonApproach` の形は、この箱と同じ。
- `collision_monitor` の `PolygonStop` は、この箱の外に置く (箱の中の点は消えているため、中に置いても検知できない)。

箱を大きくすると、自車の近くの障害物も落とすので、footprint から離れすぎないようにする。

## 運用

- 落ちたら自動で起動し直す (`respawn`)。起動直後に TF が無くて失敗した場合も、起動し直して直る。
- 前処理が止まると、`collision_monitor` は `stop_on_timeout` で速度 0 にする ([衝突の防止](../../mg_navigation/doc/costmap_and_collision.md#衝突の防止-collision_monitor))。止まったことは、`mg_diagnostics` のトピック監視 (`/top_laser_filter`) と、ウェイポイントの `~/activity` で分かる。
- 実機で、静止しているときに `collision_monitor` が発動しないこと (自車の点が消えていること) を確認する。RViz で、生のスキャンと前処理後のスキャンを重ねると分かる。
