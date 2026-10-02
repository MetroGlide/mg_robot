# 設計

GNSS をポーズグラフの拘束として使い、地球座標系と矛盾の少ない 2D 占有格子地図を、**オンラインでもオフラインでも、同じ処理で逐次**作る。
実装は C++ (`slam_gnss_2d/src`、`slam_gnss_2d/include/slam_gnss_2d`)。

## 全体の構成

```
┌──────────────────────────────────────────────────────────────────┐
│ Input 層 (ROS・rosbag2 に触れる)  input/ros2/                    │
│   ScanSourceBase   ← ROS2ScanSource  / BagScanSource  (LaserScan)│
│   OdomSourceBase   ← ROS2OdomSource  / BagOdomSource  (Odometry) │
│   GnssSourceBase   ← ROS2GnssUtmSource (NavSatFix)               │
│                    ← ROS2NavpvtSource  / BagNavPVTSource (NavPVT)│
└──────────────────────────────┬───────────────────────────────────┘
                               │ SensorFrame (scan + odom + gnss を同期したもの)
                               ▼
┌──────────────────────────────────────────────────────────────────┐
│ Core 層 (ROS のメッセージ型・通信を使わない)                      │
│   GraphOrchestrator   ポーズグラフの更新と最適化の起動            │
│     ├ GnssAnchorManager  WGS84 ↔ UTM、アンカー (地図の原点)       │
│     ├ ISAM2Optimizer     iSAM2 による逐次最適化                   │
│     └ PoseGraphBuilderBase  ノードとエッジの構築                   │
│          ├ OdomOnlyBuilder                                        │
│          ├ ScanMatchingBuilder  (逐次エッジ、near-link)            │
│          └ LoopClosureBuilder                                     │
│   MapRendererBase     CountingRenderer / OverwriteRenderer        │
└──────────────────────────────┬───────────────────────────────────┘
                               │ ポーズグラフ / 占有格子 / TF
                               ▼
┌──────────────────────────────────────────────────────────────────┐
│ Output・ノード層                                                  │
│   slam_node / slam_offline_node / reoptimize_node                 │
│   SlamVisualizer (map・path・pose_graph・anchor)                  │
│   SlamTfBroadcaster (map→odom)                                    │
│   MapSaveService (save_slam_map)、PoseGraphService (get_pose_graph)│
└──────────────────────────────────────────────────────────────────┘
```

- Core 層の `pose_graph/`、`scan_matching/`、`gnss/`、`optimizer/`、`map_manager/` は、ROS のメッセージ型を使わない。ログに `RCLCPP_*` だけを使う。
- ROS に触れる入力は `input/ros2/` に集め、`SlamNodeBase` (`core/slam_node_base.cpp`) が、センサを同期して `SensorFrame` を作る。`GraphOrchestrator` は、同期済みのデータだけを受け取る。
- スキャンの角度は、入力の層が `base_link` の座標系に直す。センサの取り付けの回転は、`/tf_static` (URDF から配信される) を使い、YAML には書かない (URDF が唯一の正)。

## データ型 (`core/data_types.hpp`)

| 型 | 内容 |
| :--- | :--- |
| `ScanData` | 1 回のスキャン (各ビームの距離と角度、タイムスタンプ、LiDAR の取り付け位置、走査時間) |
| `OdomData` | オドメトリの姿勢 (x, y, yaw) |
| `GnssData` | GNSS の測位 (地図の平面の位置、共分散、`fix_status`、緯度経度) |
| `SensorFrame` | 同期した `ScanData`・`OdomData`・`GnssData` (`GraphOrchestrator` への入力の単位) |
| `PoseNode` | ポーズグラフのノード (キーフレームの位置・向き・スキャン・法線) |
| `PoseEdge` | ノード間のエッジ (相対姿勢、情報行列、スコア、オドメトリへのフォールバックか) |
| `GnssPrior` | GNSS による絶対位置の拘束 (ノード、位置、情報行列) |
| `MatchResult` | スキャンマッチングの結果 (相対姿勢、情報行列、スコア、収束したか) |
| `ScanProcessResult` | 1 フレームを処理した結果 (追加したノード、ループが成立したか、再描画が必要か、追加したエッジと prior) |
| `SubmapPatch` | ノードごとの、地図の部分的な更新 |

## 処理の流れ

`GraphOrchestrator::process_frame` が、1 フレームごとに次を行う。

1. **キーフレームの判定とスキャンマッチング** (`PoseGraphBuilderBase::add_scan`)。移動が `keyframe.min_translation` または回転が `keyframe.min_rotation` を超えたときに、新しいノードを作る。直前のノードとの逐次エッジと、`near_links` の多重リンクの候補を作る。新しいノードができなければ、ここで終わる。
2. **GNSS の初期化** (状態が `INITIALIZING` のとき)。アンカーを決め、`init_distance_m` 走ったところで初期方位を決めて、グラフ全体を UTM の向きに回す ([gnss_algorithm.md](./gnss_algorithm.md))。完了するまでは、最適化を行わない。完了すると、状態は `RUNNING` になる。
3. **エッジの追加**。最新の逐次エッジと、新しいループのエッジ (`loop_closure.enabled` のとき。サブマップに対してマッチングを行い、基準を満たしたものだけ) を、オプティマイザに足す。
4. **動的な再アンカー** (`gnss.dynamic_reanchor.enabled`)。RTK Fix の測位が十分に集まったら、アンカーの位置を計算し直す。
5. **GNSS の prior の追加**。条件を満たす測位を、ノードの位置の拘束として追加する。
6. **iSAM2 の更新**と、結果のポーズグラフへの反映。ノードの動きが `optimization.rerender_threshold_m` を超えたとき (ループの成立と再アンカーのときも)、地図を全体で描き直す。

GNSS を使わない (`gnss.enabled: false`) ときは、状態が最初から `RUNNING` で、2、4、5 を行わない。

オフライン (`slam_offline_node`) は、同じ処理を、rosbag2 から読んだデータで、リアルタイムを待たずに進める。終了時 (`finalize`) に、`optimization.batch_on_finalize` が `true` なら、全グラフの一括最適化 (Levenberg-Marquardt、最大 `batch_max_iterations` 回) を行う。

## スキャンマッチング

`scan_matching.type` で選ぶ (`scan_matching/`)。

| 種別 | 内容 |
| :--- | :--- |
| `multi_res_csm` (既定) | 2 段階の相関探索 (粗い → 細かい) と、放物線によるピークの補間。尤度場のグリッドを使う |
| `icp` | Point-to-Line ICP。法線方向の残差を最小にする。Huber・Cauchy のロバストカーネル |
| `ndt` | 2D の NDT (セル内の分布の尤度を最大にする) |
| `csm` | 単体の相関スキャンマッチャー (網羅的な探索) |
| `coarse_to_fine` | NDT (粗い) → ICP (細かい) |
| `multi_start_coarse_to_fine`、`multi_start_icp` | 回転のグリッドで複数の初期値から探す (`scan_matching.multi_start`) |

- 参照は `scan_matching.reference` で、`scan_to_local_map` (直近 `local_map.window` 個のキーフレームの合成) または `scan_to_scan`。
- **デスキュー** (`deskew`): LiDAR は 1 走査に約 0.1 秒かかるので、各ビームの計測時刻のオドメトリの姿勢を補間し、`header.stamp` 時点の姿勢に点群を直す。rplidar は、ビーム番号が増えるほど先に計測される (`direction: -1`)。
- **オドメトリの融合** (`scan_matching.odom_fusion`): スキャンマッチングの結果とオドメトリの並進を、ガウス積で融合する。マッチングの拘束が弱い方向 (廊下など) では、オドメトリが効く。
- **連続失敗の対策**: マッチングが `max_failure_streak` 回続けて失敗したら、オドメトリにフォールバックして、低い信頼度のエッジとして受け入れる (失敗が連鎖するのを避ける)。`max_translation_drift` は、進行方向の縮退のすべりの保護。
- **ロバスト化** (`optimization.between_robust_kernel`): マッチングのエッジ (逐次・near-link・ループ) に、Huber などのカーネルをかけて、誤マッチの影響を抑える。

## ループクロージャ (`loop_closure.*`)

- 現在位置から `search_radius` 以内で、ノード番号の差が `min_node_gap` 以上の過去のノードを、候補にする。
- 候補の周辺のキーフレームの点群を合成したサブマップ (`submap_radius`) に対して、`matcher_type` のマッチャーで合わせる。
- マッチングのスコア (`max_score`)、相対回転 (`max_dyaw_deg`)、交差の角度 (`crossing_reject_deg`) の判定を通ったものだけを、ループのエッジとしてグラフに加える。
- 既定では**無効** (`loop_closure.enabled: false`)。

## 地図の描画 (`map_manager/`)

| レンダラー | 内容 |
| :--- | :--- |
| `counting` (既定) | セルごとに hit と miss の重み付きの数を数え、比が `hit_threshold` 以上で、`min_hits` 回以上のセルを占有とする。壁をかすめた通過による、消しすぎを抑える |
| `overwrite` | 新しいスキャンで上書きする |

`trajectory_noise_filter` で、軌跡の周りのノイズを消す (または薄める)。既定では無効。

## 最適化 (`optimizer/`)

- 既定の `isam2` (`ISAM2Optimizer`) が、ノード・エッジ・GNSS の prior を足すたびに、逐次に最適化する。`relinearize_threshold` は 0.1。
- `GTSAMOptimizer` (Levenberg-Marquardt) は、`reoptimize_node` の一括最適化が使う。
- オフラインの終了時の一括最適化は、`optimization.batch_on_finalize` と `batch_max_iterations` で決まる (`GraphOrchestrator::finalize`)。

## 設計の決定

- **入出力とアルゴリズムの分離**: ROS の入出力と、SLAM のロジックを分ける。ロジックは rosbag2 でも ROS のトピックでも動く。
- **逐次の単一のパイプライン**: オンラインとオフラインで別のロジックにしない。GNSS は、使えるときに、すぐ prior として加える。
- **GNSS を失ったときの特別な状態を持たない**: 得られたデータだけを、そのままグラフに反映する。
- **反復して収束する処理には、連続失敗の対策を置く**: 自分の出力を次の入力の初期値にする処理 (ICP の初期値、ループの候補) は、失敗が連鎖する。`failure_streak` の上限で、フォールバックする。

## ノードの役割

| 実行ファイル | 内容 |
| :--- | :--- |
| `slam_node` | オンラインの SLAM |
| `slam_offline_node` | rosbag2 のオフライン処理 |
| `reoptimize_node` | 保存したポーズグラフの再最適化 |
| `pose_graph_preview_node` | 保存済みのポーズグラフと地図のプレビュー |
| `slam_gnss_nav_bridge_node` | ナビゲーション用に、GNSS を地図の座標に変換する ([nav_bridge.md](./nav_bridge.md)) |
| `anchor_publisher_node` | `gnss_transform.yaml` のアンカーを配信する |
| `save_slam_map_cli` | 地図と関連ファイルの一括保存 |

## 関連

- 全パラメータ: [parameters.md](./parameters.md)
- GNSS の拘束: [gnss_algorithm.md](./gnss_algorithm.md)
- 開発のルール: [development.md](./development.md)
