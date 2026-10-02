# SLAM-GNSS-2D ページの表示

SLAM-GNSS-2D ページ (`/slam-gnss-2d`、`pages/SlamGnss2DPage.tsx`) で表示する各要素の意味と、色の対応。
表示するのは、現役の SLAM ([slam_gnss_2d](../../../slam_gnss_2d/README.md)、C++) の出力。

ポーズグラフ (ノードとエッジ) は、`slam_gnss_2d` が配信する `MarkerArray` ではなく、フロントエンドが `PoseGraphDiff` から**自分で描く** (`components/ros-viewer/layers/PoseGraphLayer.tsx`)。軌跡と地図は、トピックをそのまま描く。

## 描画する要素

| 要素 | 色 | 意味 | データ |
| :--- | :--- | :--- | :--- |
| 地図 | (グレースケール) | 占有格子の地図 | `/map` (`MapLayer`) |
| 軌跡 (最終) | シアン `#00ffff` | 全ノードの推定位置を、時系列に結んだ線。ループの成立と GNSS による最適化のあとは、グラフ全体を作り直した**最終の軌跡** | `slam_gnss_2d/path` (`PathLine`) |
| 軌跡 (最適化前) | グレー `#666666` | 最適化の直前の軌跡のスナップショット。最適化の前後の変化を比べる (`layers.pathBefore`) | `slam_gnss_2d/path_before_optimize` |
| ノード | シアン。最新のノードだけ赤 | スキャンを採用したキーフレームの推定位置。移動が `keyframe.min_translation`、または回転が `keyframe.min_rotation` を超えたときに追加される。クリックで詳細 (`PoseGraphDetailPanel`) | `PoseGraphDiff`、`get_pose_graph` |
| 逐次エッジ | 緑 (半透明) | 時系列で隣り合うノードの、スキャンマッチングの拘束 | 同上 |
| ループエッジ | マゼンタ (太線) | ループクロージャで足した、離れたノード間の拘束。既定ではループクロージャが無効なので、出ない | 同上 |
| GNSS prior | オレンジ (ワイヤーフレームの四角) | GNSS の拘束が付いたノード | 同上 (`prior_node_indices`) |

レイヤーごとの表示は、ページの「layers」で切り替える (`poseGraphNodes`、`poseGraphSeqEdges`、`poseGraphLoopEdges`、`poseGraphGnssPrior`、`pathBefore`)。

## データの取得 (`hooks/usePoseGraph.ts`)

1. 起動時に、サービス `/slam_gnss_2d/get_pose_graph` (`GetPoseGraph`) で、ポーズグラフ全体を取る。
2. 以降は、`slam_gnss_2d/pose_graph_diff` (`PoseGraphDiff`) の差分で、ノード・エッジ・prior・統計を更新する。
3. 差分の `full_refresh_needed` が `true` のときは、保持しているデータを捨てて、1 をやり直す。

`PoseGraphDiff` と `GetPoseGraph` は、`slam_gnss_2d_msgs` の型。差分には、ノード (位置と向き)、逐次エッジ (スコアと種別: ICP かオドメトリへのフォールバックか)、GNSS prior (標準偏差と測位の種類)、ループエッジ、`loop_closed` が入る。

## 使うトピックとサービス

| 名前 | 型 | 配信元 |
| :--- | :--- | :--- |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_node` |
| `slam_gnss_2d/path` | `nav_msgs/Path` | `slam_node` |
| `slam_gnss_2d/path_before_optimize` | `nav_msgs/Path` | `slam_node` |
| `slam_gnss_2d/pose_graph_diff` | `slam_gnss_2d_msgs/PoseGraphDiff` | `slam_node` |
| `/slam_gnss_2d/get_pose_graph` (サービス) | `slam_gnss_2d_msgs/GetPoseGraph` | `slam_node`、`pose_graph_preview_node` |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GNSS (衛星の写真への重ね描きと、状態の表示に使う) |

`ros/topics.ts` には、`slam_gnss_2d/gnss_raw_markers` と `slam_gnss_2d/gnss_prior_markers` の定義もある。現役の SLAM は、この 2 つを配信しない (旧 Python 版の `slam_offline_node` が配信していたもの)。

## 地図の操作 (system_manager の API)

ページの「saved-map」から、次を呼ぶ ([system_manager.md](../../mg_system_manager/doc/system_manager.md))。

| 操作 | API |
| :--- | :--- |
| 地図の保存 | `POST /slam_gnss_2d/map/save` |
| 保存済みの一覧 | `GET /slam_gnss_2d/maps` |
| プレビューの開始・停止 | `POST /slam_gnss_2d/preview/start`・`stop` (compose の `map-preview`) |
| 再最適化の開始・停止 | `POST /slam_gnss_2d/reoptimize/start`・`stop` (compose の `reoptimize-slam`) |
