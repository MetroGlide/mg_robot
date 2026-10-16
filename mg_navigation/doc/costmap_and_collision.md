# コストマップ、プランナ、衝突の防止

Nav2 の他の部分は [architecture.md](./architecture.md)。値は `params/nav2_params.yaml` と `params/planner_*.yaml` が正で、この文書には意味と関係を書く。

## コストマップ

`local_costmap` と `global_costmap` は、フットプリントと `footprint_padding` を共有する (アンカー `&robot_footprint`、`&footprint_padding`)。フットプリントの原点は車体の後端寄り。

| | local_costmap | global_costmap |
| :--- | :--- | :--- |
| 座標系 | `odom` (ローリングウィンドウ) | `map` |
| 更新 | 高め (障害物への追従) | 低め |
| 地図 | なし | `/planning_map` (`static_layer`) |
| レイヤー | `top_obstacle_layer`、`obstacle_stvl_layer`、`inflation_layer` | `static_layer`、`top_obstacle_layer`、`obstacle_stvl_layer`、`inflation_layer` |
| 膨張 | `inflation_radius` は global より小さい | `inflation_radius` は local より大きい |

- `top_obstacle_layer`: 上 LiDAR の障害物の記録と消去。入力は、前処理済みの `/scan_filtered/scan_top_lidar` (近距離のノイズと自車の点を落としたもの。[mg_drivers](../../mg_drivers/doc/laser_filters.md) の `laser_filters.launch.py`)。`obstacle_max_range` と `raytrace_max_range` は、local は短く、global は長くする。近距離の除去は前処理に一本化し、`*_min_range` は 0 にしている。
- `obstacle_stvl_layer` (`spatio_temporal_voxel_layer`): 3D 障害物検出の出力 (`/obstacle_detection_3d_node/points_obstacle`、[mg_drivers](../../mg_drivers/doc/obstacle_detection_3d.md)) を記録する。`voxel_decay` は、障害物を覚えている時間で、local は短く、global は長い。
- 深度カメラの `depth_voxel_layer`、`denoise_layer` は、定義だけが残り、`plugins` からは外れている。
- `speed_filter` (`filters`) は、定義だけがあり、`filters` はコメントアウトされている。
- **`footprint_padding` の理由**: 膨張レイヤーの内接半径は、フットプリントの原点から最も近い辺までの距離で決まる。このフットプリントは原点が後端寄りで、そのままでは半幅より小さくなる。車体の幅より狭い隙間 (例: 間隔 0.8 m のコーン) が致死扱いにならず、プランナが通れると誤認して経路を引き、RPP が詰まって失敗する。余白で内接半径を半幅にして、通れない隙間を塞ぐ。
- Smac から NavFn に戻すときは、`inflation_radius` と `cost_scaling_factor` と、この余白も確認する (`planner_navfn.yaml` のコメント)。
- 膨張の `cost_scaling_factor` は、RPP のコストによる減速と同じ値にする ([rpp_tuning.md](./rpp_tuning.md))。

## グローバルプランナ

`global_planner` 引数で、`params/planner_<名前>.yaml` を選ぶ (launch が既定を決める)。

| 名前 | ファイル | 特徴 |
| :--- | :--- | :--- |
| `smac_lattice` | `planner_smac_lattice.yaml` | フットプリント全体で衝突を判定し、差動二輪の動き (前進・その場旋回) の経路を作る。Gazebo での計測では、CPU は NavFn とほぼ同じ (計画が重くなるのは、経路がない・狭い場所のとき) |
| `navfn` | `planner_navfn.yaml` | 点ロボットとして計画する (従来)。計算負荷が小さい |

実機で計画の負荷が高いときは、`global_planner:=navfn` で戻せる。`navfn` を使うときは、`nav2_params.yaml` の `footprint_padding` と `inflation_layer` の値を、上の理由で確認する。

## 衝突の防止 (`collision_monitor`)

**RPP の後ろ盾になる、最後の安全層。** 障害物を主に避けるのは RPP とコストマップ (経路に基づく衝突の確認、コストによる減速) で、`collision_monitor` は生の scan だけで判定する。コストマップの更新の遅れに依存しないが、通常の走行では発動しないように作る (障害物のない走行と、静止した障害物を避ける走行で、`STOP` と `APPROACH` が 0 回であることを、シナリオテストで確認している)。

入力は、前処理済みの上 LiDAR (`/scan_filtered/scan_top_lidar`。コストマップと同じ。近距離のノイズと、自車の箱 (footprint に余白を足したもの) の中の点を落としたもの。[mg_drivers](../../mg_drivers/doc/laser_filters.md))。次の 3 つを、この順に評価する (`stop` が決まったら、以降を飛ばす)。すべて `min_points` で孤立した点を無視する。

| ポリゴン | 動作 | 形の考え方 |
| :--- | :--- | :--- |
| `PolygonStop` | 停止 (速度 0) | 自車の箱の前端の外側に置く。奥行きは、巡航速度からの制動距離 (`max_decel` から求める) に、scan と制御の遅れぶんを足した以上にする |
| `PolygonSlowdown` | 減速 (`slowdown_ratio` 倍) | 前方と両脇に広く、後方に短く取る。側方から進路に入る対象 (横切る歩行者) に早く反応するため、両脇を広くしている |
| `PolygonApproach` | 接近 (`time_before_collision`) | 自車の箱と同じ大きさの形を、現在の速度 (旋回を含む) で前方にシミュレートし、その時間以内に接触するなら減速する。旋回中の側面の接触も対象 |

- `PolygonApproach` の時間は、RPP の衝突の確認 (`max_allowed_time_to_collision_up_to_carrot`) より短くする。RPP が先に減速・停止する。
- `PolygonApproach` は `footprint_topic` (`local_costmap/published_footprint`) を使わない。`footprint_padding` を含んで自車の箱より大きくなり、箱の外の自車の点や、狭い隙間 (間隔 0.8 m のコーンなど) で止まる。
- 3 つの形は、自車の箱の外に置く (箱の中の点は前処理で消えている)。値の関係は `test/test_nav2_params_consistency.py` が検査する。
- 上流の `collision_detector` (指令には介入せず、状態だけを出す) は、使っていないので起動しない。
- 状態は `collision_monitor_state` に出る (観測用。`waypoint_sequencer_node` の `~/activity` の入力にもなる。`slowdown` と `approach` は `obstacle_slowdown`)。
- 各ポリゴンは、`enabled` を動的パラメータ (`ros2 param set /collision_monitor PolygonStop.enabled false` など) で切り替えられる。走行が不安定なときの切り分けに使える。
- **データの途絶での停止 (`scan_top.stop_on_timeout: True`)**: scan のデータが `source_timeout` 以上途絶えた、またはまだ届いていないときに、ポリゴンの `enabled` によらず速度 0 にする。`source_timeout` は、LiDAR の周期と実測の最大の間隔より十分長くする。`collision_monitor_state` の `polygon_name` は `source_timeout:scan_top`。LiDAR や前処理 (`top_laser_filter`) が止まったまま走り続けるのを防ぐ。シーケンサの走行は、`driver_watchdog_node` が先に一時停止する ([mg_drivers](../../mg_drivers/doc/driver_watchdog.md))。`ros2 param set /collision_monitor scan_top.stop_on_timeout false` で切り替えられる。
- **保護の外**: Web UI のジョイスティックなどの手動操作は `/cmd_vel` に直接出すので、`collision_monitor` と `velocity_smoother` を通らない。`behavior_server` (後退・旋回) も通らない。後退・旋回の安全は、`behavior_server` がローカルコストマップとフットプリントで先読みする衝突の確かめ (致死・未知のセルで中止) だけに頼る。ごく近い物 (前処理で落とす自車の箱の中) と LiDAR の面より低い物は見えないので、実機では、上 LiDAR から後方が台車の陰なく見えていることを確かめる。
- **水平な 2D LiDAR の限界**: 低い障害物 (LiDAR の面より低いもの) は見えない。上り坂の地面を、障害物として見ることがある。実機で、段差や坂の手前で意図せず減速・停止しないか確認する。
