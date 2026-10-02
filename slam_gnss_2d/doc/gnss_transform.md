# gnss_transform.yaml と座標変換

`gnss_transform.yaml` は、SLAM の地図の座標系と、地球の座標系 (WGS84 / UTM) の対応を持つ。SLAM の保存 (`save_slam_map_cli`、`SaveSlamMap`) が作り、ナビゲーションの GNSS ブリッジ (`slam_gnss_nav_bridge_node`) が読む。

## フォーマット

```yaml
anchor:
  latitude: 35.6812362
  longitude: 139.7671248
anchor_utm:
  easting: 388123.456
  northing: 3949821.789
  zone: 54
  hemisphere: north
rotation_rad: 0.284592
map_rotation_rad: 0.012255      # 任意。SLAM の出力には含まれない
metadata:
  created_at: 2026-09-21T01:20:00
  slam_backend: gtsam
```

| キー | 内容 | ブリッジが使うか |
| :--- | :--- | :--- |
| `anchor.latitude` / `longitude` | アンカー (地図の原点) の緯度経度。`/slam_gnss_2d/anchor` に配信される | 使う |
| `anchor_utm.easting` / `northing` / `zone` / `hemisphere` | アンカーの UTM の座標 | 使う |
| `rotation_rad` | SLAM の起動時に推定した、初期方位の回転 (記録用) | **使わない** |
| `map_rotation_rad` | 地図の座標軸の、UTM のグリッド (東・北) に対する回転 | 使う (無ければ 0) |
| `metadata` | 作成日時、最適化のバックエンド | 使わない |

## SLAM が作るもの

- **アンカー**: 最初の有効な測位の位置が、アンカーになる。
- **初期方位の回転 `rotation_rad`**: 起動直後の、ロボットの向きと真北の差。ロボットが `init_distance_m` 走ったときに、GNSS の軌跡と SLAM の軌跡から推定する ([gnss_algorithm.md](./gnss_algorithm.md#2-初期方位の推定-initialize_with_gnss_if_ready))。
- 推定した回転で、最初のノードの向きを決めて、グラフ全体を回す。その結果、**SLAM が作る地図の X 軸は UTM の東、Y 軸は北に揃う**。`rotation_rad` は、その記録になる。

## ブリッジの変換

生の GNSS の緯度 φ・経度 λ から、地図の座標 (x, y) への変換は、次のとおり。

```
(E, N) = UTM_Forward(φ, λ)                     # ゾーンは anchor_utm.zone
(x, y) = R(map_rotation_rad) · (E − E_anchor, N − N_anchor)
```

`R(θ)` は 2D の回転行列。`map_rotation_rad` が無いときは θ=0 で、アンカーからの差が、そのまま地図の座標になる。

- SLAM が作った地図は、すでに東・北に揃っているので、`map_rotation_rad` は不要 (`rotation_rad` を、もう一度適用すると、二重の回転になるので使わない)。
- 次の地図は、UTM のグリッドからずれているので、`map_rotation_rad` を書く。
  - **シミュレータの地図** (東・北に揃っている): UTM のグリッドとの、子午線収束角のずれ。`tools/scripts/make_sim_gnss_transform.py` が書く。
  - **[waypoint-tool](https://github.com/Chu-son/waypoint-tool) で背景の地図に位置合わせして回した地図**: 位置合わせの回転の逆向き。`mg_waypoint_navigation/waypoint_tool/mg_gnss_transform.wpt_template` で出力する ([waypoint_format.md](../../mg_waypoint_navigation/doc/waypoint_format.md#waypoint-tool-での作成))。
- 変換の実装は `slam_gnss_nav_bridge_node.cpp` (`map_rotation_rad` の読み込みと適用)。ヘディングの角にも、同じ回転を足す。

## 関連

- ブリッジの仕様: [nav_bridge.md](./nav_bridge.md)
- 地図の指定とデータの置き場: [doc/environment.md](../../doc/environment.md#地図の指定)
