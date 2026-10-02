# パラメータ

SLAM のパラメータは `slam_gnss_2d/params/slam_gnss_2d.yaml` (ノード名にかかわらず `/**`)。各値の意味は、yaml のコメントにもある。
ここに書く値は、**yaml の値** (launch が読み込む値)。コードの既定値 (`core/config_loader.cpp` の `declare_params`) とは、違うものがある。例: `map.publish_hz` (コード 1.0、yaml 0.1)、`map.renderer` (`overwrite`、`counting`)、`scan_matching.type` (`ndt`、`multi_res_csm`)。
ナビゲーションの GNSS ブリッジのパラメータは [nav_bridge.md](./nav_bridge.md)。

## トピック

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `topics.scan` | `/scan_top_lidar` | LiDAR のスキャン (`LaserScan`) |
| `topics.odom` | `/odom` | ホイールオドメトリ (`Odometry`)。SLAM は補正前の値を使う ([mg_drivers](../../mg_drivers/doc/wheel_odom_corrector.md)) |

## 地図 (`map.*`)

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `resolution` | `0.05` | 占有格子の解像度 [m/セル] |
| `expansion_margin` | `20.0` | 地図の境界を広げるときの余白 [m] |
| `publish_hz` | `0.1` | 地図の配信周期 [Hz] |
| `renderer` | `counting` | `counting` (hit と miss の数) / `overwrite` (上書き) |
| `hit_threshold` | `0.3` | (counting) 占有とする、`weighted_hit / (weighted_hit + weighted_miss)` の閾値 |
| `min_hits` | `2` | (counting) 占有とするのに必要な最小の hit の回数 |
| `hit_weight` / `miss_weight` | `1.0` / `0.5` | (counting) hit と miss の重み。miss を軽くして、壁をかすめた通過で消しすぎないようにする |
| `miss_clearance_margin` | `0.0` | (counting) 終端の手前で miss を止める余白 [m] (0 で無効) |
| `max_miss_ratio` | `0.0` | (counting) 確かな壁に対する miss の上限の比 |
| `skip_intermediate_rendering` | `false` | 途中の描画を省き、終了時に描く (オフラインは launch 引数が優先) |

## キーフレームとデスキュー

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `keyframe.min_translation` | `0.5` | 新しいノードに必要な最小の移動 [m] (slam_toolbox と同じ) |
| `keyframe.min_rotation` | `0.5` | 同、最小の回転 [rad] |
| `deskew.enabled` | `true` | スキャン内の運動によるゆがみの補正 |
| `deskew.direction` | `-1` | ビーム番号と計測時刻の対応 (`+1`: 番号が増えるほど後、`-1`: 先)。rplidar は `-1` |
| `deskew.start_offset_s` | `-0.03` | 最初に計測されるビームの、`header.stamp` に対する時刻のずれ [s] |
| `deskew.duration_s` | `0.0` | 走査時間 [s] (0 でメッセージの `scan_time`) |

## スキャンマッチング (`scan_matching.*`)

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `enabled` | `true` | スキャンマッチングを使うか |
| `type` | `multi_res_csm` | `multi_res_csm` / `multi_start_coarse_to_fine` / `coarse_to_fine` / `icp` / `ndt` / `csm` |
| `reference` | `scan_to_local_map` | 参照の点群: `scan_to_scan` / `scan_to_local_map` |
| `max_failure_streak` | `5` | 連続の失敗の許容回数。超えたらオドメトリにフォールバックする |
| `max_translation_drift` | `0.08` | 進行方向の縮退のすべりの保護の閾値 [m] (オドメトリの進み量との差の上限) |
| `yaw_information_multiplier` | `3.0` | マッチングのエッジの、向きの情報量の倍率 |
| `odom_fusion.information_x` | `1500.0` | オドメトリを融合する、進行方向の情報量 (0 で無効) |
| `odom_fusion.information_y` / `information_yaw` | `0.0` / `0.0` | 同、横方向 / 回転 |
| `num_threads` | `12` | (multi_res_csm) 1 回のマッチングが使う OpenMP のスレッド数 (0 で OpenMP の既定。多すぎると並列の効率が落ちる) |

### multi_res_csm

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `multi_res_csm.linear_search_window` | `0.4` | 並進の探索窓 [m] (±) |
| `multi_res_csm.angular_search_window_deg` | `15.0` | 回転の探索窓 [deg] (±) |
| `multi_res_csm.linear_step` | `0.03` | 並進の刻み [m] |
| `multi_res_csm.angular_step_deg` | `1.0` | 回転の刻み [deg] (細かい段は 0.2 度と放物線補間) |
| `multi_res_csm.grid_resolution` | `0.03` | 尤度場の解像度 [m] |
| `multi_res_csm.score_threshold` | `0.05` | 相関スコアの有効の閾値 |
| `multi_res_csm.enable_variance_penalty` | `false` | オドメトリの予測からの乖離のペナルティ (slam_toolbox 相当) |
| `multi_res_csm.distance_variance_penalty` / `angle_variance_penalty` | `0.5` / `1.0` | ペナルティの係数 |
| `multi_res_csm.minimum_distance_penalty` / `minimum_angle_penalty` | `0.5` / `0.9` | ペナルティの下限 |

### near_links (直近のキーフレームの多重リンク)

片道の走行でも、局所のメッシュ構造にして、回転のゆがみを打ち消す (slam_toolbox 相当)。

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `near_links.enabled` | `true` | 多重リンク |
| `near_links.buffer_size` | `10` | 直近のキーフレームのバッファ |
| `near_links.max_distance` | `2.0` | リンクを張る最大の距離 [m] |
| `near_links.min_index_diff` | `2` | 直前 (差 1) 以外の、最小のインデックスの差 |
| `near_links.max_links_per_node` | `3` | 1 ノードあたりの最大のリンク数 |
| `near_links.max_translation_drift` / `max_rotation_drift_deg` | `0.4` / `15.0` | リンクを採用する、並進 [m] / 回転 [deg] の乖離の上限 |
| `near_links.min_eigenvalue` | `10.0` | 縮退を防ぐ、情報行列の最小の固有値 |

### その他のマッチャー

| グループ | 内容 |
| :--- | :--- |
| `multi_start.*` | `multi_start_*` のとき。回転グリッド: `angular_search_window_deg` 20、`angular_step_deg` 2.5、直進 0 度と前回の角速度の仮説 |
| `icp.*` | `icp`、`coarse_to_fine` の細かい段。`max_iterations` 100、`max_correspondence_dist` 1.0、`robust_kernel` `huber` (`robust_kernel_scale` 0.15)、`motion_prior_weight_x/y/yaw` 10 / 500 / 0 (差動二輪の横すべりを防ぐ) |
| `ndt.*` | `ndt`、`coarse_to_fine` の粗い段。`cell_size` 1.0、`cell_sizes` [1.0, 0.5]、`use_bilinear` |
| `csm.*` | 単体の `csm`。`linear_search_window` 1.0、`angular_search_window` 0.5、`linear_step` 0.05、`angular_step` 0.02 |
| `local_map.*` | `scan_to_local_map` のとき。`window` 30 (直近のノード数)、`radius` 30.0 m |

## ループクロージャ (`loop_closure.*`)

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `enabled` | `false` | ループクロージャ (`false` のとき、以下はすべて無効) |
| `search_radius` | `4.0` | 候補を探す半径 [m] |
| `min_node_gap` | `50` | 候補にする、最小のノード番号の差 |
| `max_failure_streak` | `3` | 連続の失敗の閾値 |
| `matcher_type` | `icp` | `icp` / `ndt` / `csm` / `coarse_to_fine` |
| `yaw_information_multiplier` | `100.0` | ループのエッジの、向きの情報量の倍率 |
| `max_dyaw_deg` | `145.0` | 許容する最大の相対回転 [deg] |
| `crossing_reject_deg` | `45.0` | 直交して交差するものを除く角度 [deg] |
| `submap_radius` | `5.0` | 相手側のサブマップの半径 [m] |
| `max_score` | `0.05` | ループのエッジを採用する、最大のマッチングのスコア |
| `icp.*` | | `max_iterations` 100、`max_correspondence_dist` 1.0、`robust_kernel` `cauchy` (`robust_kernel_scale` 0.1) |
| `ndt.*` | | `cell_sizes` [2.0, 1.0, 0.5] |
| `csm.*` | | `matcher_type: csm` のとき |

## GNSS

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `gnss.enabled` | `true` | GNSS の絶対位置の拘束 (`false` のとき、以下はすべて無効) |
| `gnss.source` | `navpvt` | `navsat_fix` / `navpvt` |
| `gnss.topics.fix` / `navpvt` | `/gps/fix` / `/navpvt` | トピック |
| `gnss.navpvt_hacc_scale` | `1.2` | (navpvt) 水平精度 (`h_acc`) にかける、標準偏差の係数 |
| `gnss.min_interval_m` | `4.0` | prior を足す最小の移動の間隔 [m] |
| `gnss.max_innovation_m` | `0.0` | SLAM の推定位置との乖離の上限 [m] (0 以下で無効。デッドロックの防止) |
| `gnss.robust_kernel` / `robust_kernel_scale` | `huber` / `1.5` | prior のロバストカーネル |
| `gnss.prior.min_fix_status` | `0` | prior に採用する最小の測位の種類 (0 すべて、1 RTK Float 以上、2 RTK Fix のみ) |
| `gnss.validation.max_sigma_m` | `2.0` | 採用する標準偏差の上限 [m] |
| `gnss.lever_arm.x` / `y` | `0.26` / `-0.13` | アンテナの `base_link` での位置 [m]。URDF の `base_link`→`gps_link` と合わせる (0 / 0 で無効) |
| `gnss.sigma.fix_m` / `float_m` | `0.05` / `0.3` | 共分散が分からないときの標準偏差 [m] |
| `gnss.sigma.factor_yaw_variance` | `1e8` | prior の向きの分散 [rad²] |

### アンカー (`gnss.anchor.*`)

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `min_fix_status` | `0` | アンカーを決める最小の測位の種類 |
| `init_distance_m` | `30.0` | 初期方位の推定に必要な、最小の直線の移動 [m] |
| `sigma_m` | `2.0` | 最初のノードの位置の標準偏差 [m] |
| `init_yaw_sigma_rad` | `1.0` | 最初のノードの向きの標準偏差 [rad] |

### 動的な再アンカー (`gnss.dynamic_reanchor.*`)

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `enabled` | `true` | 高精度な測位が出たとき、アンカーの位置を計算し直す |
| `min_fix_status` | `2` | RTK Fix のみ |
| `min_samples` | `10` | 剛体変換に必要な最小のサンプル数 |
| `min_distance_m` | `10.0` | 同、最小の広がり [m] |
| `max_residual_rms_m` | `1.0` | 採用する、残差の RMS の上限 [m] |

## 最適化 (`optimization.*`)

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `backend` | `isam2` | 最適化のバックエンド (`isam2` のみ) |
| `between_robust_kernel` | `huber` | マッチングのエッジ (逐次・near-link・ループ) のロバスト化: `none` / `huber` / `cauchy` |
| `between_robust_kernel_scale` | `1.345` | ロバスト化の閾値 (白色化した残差。標準偏差の倍数) |
| `rerender_threshold_m` | `0.1` | 最適化後のノードの動きがこれを超えたら、地図を全体で描き直す [m] |
| `batch_on_finalize` | `true` | オフライン処理の終了時に、全グラフを一括で最適化する |
| `batch_max_iterations` | `100` | 一括最適化の最大の反復 |
| `isam2.relinearize_threshold` | `0.1` | iSAM2 の再線形化の閾値 |

## 軌跡によるノイズ除去 (`trajectory_noise_filter.*`)

| パラメータ | 値 | 内容 |
| :--- | :--- | :--- |
| `enabled` | `false` | ノイズ除去 |
| `type` | `attenuate` | `clear` (消す) / `attenuate` (薄める) |
| `radius_m` | `1.0` | 軌跡の周りの半径 [m] |

## ノードごとの上書き

| ノード | パラメータ | 値 | 内容 |
| :--- | :--- | :--- | :--- |
| `slam_gnss_2d_offline_node` | `offline_step_hz` | `0.0` | オフラインの処理速度 [Hz] (0 以下で最大の速度) |
| `reoptimize_node` | `enable_re_scan_matching` | `false` | 逐次エッジの再マッチング |
| | `enable_new_loop_search` | `false` | 新しいループの探索 |
| | `scan_matching.type` ほか | `ndt` ほか | オフライン専用の高精度の設定 (NDT の `cell_sizes` [0.5, 0.2]、ICP の `max_iterations` 500 など) |
| | `loop_closure.search_radius` | `5.0` | オンラインより広い探索 |
| | `trajectory_noise_filter.enabled` | `true` | |

## 変更するとき

パラメータを追加するときは、3 か所を同時に更新する ([development.md](./development.md#パラメータを追加するとき))。launch の引数は [README](../README.md#launch)。
