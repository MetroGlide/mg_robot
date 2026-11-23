# 地図と GNSS の変換

地図と GNSS の対応点から `gnss_transform.yaml` の剛体変換を求めるツール、地図と GNSS の場所ごとのずれを測るツール、waypoint-tool のグループを動かすツール、シミュレータ用の生成ツール。一覧は [tools の README](../README.md)。

## `generate_static_transforms.py` (地図とGNSSの剛体変換算出)

地図画像上のピクセル座標と、対応する現地での GNSS 緯度経度から、UTM 座標系から地図座標系への剛体変換（回転 + 並進 `[x, y, yaw]`）を Procrustes 解析（SVD）により推定します。

```bash
python3 tools/scripts/generate_static_transforms.py \
  --map-dir /root/ros2_data/map \
  --base-yaml-name gnss_to_map_base_data.yaml \
  --out-yaml-name gnss_to_map_static_transforms.yaml \
  --utm-zone 54
```

---

## `measure_gnss_map_offset.py` (地図と GNSS の場所ごとのずれ)

地図の座標と、GNSS を `gnss_transform.yaml` でブリッジと同じに変換した座標の、場所ごとのずれを測ります。
衛星写真などで目で合わせた地図が、GNSS とどれだけずれているかを確かめ、変換や地図のずらし量を決めるのに使います。

### measure (`make bag-gnss-map-offset`)

走行の rosbag のスキャンを、基準の軌跡で前後数 m の局所の点群にまとめ、地図に照合します。
照合した姿勢 (地図が言う位置) から基準の姿勢 (GNSS が言う位置) を引いたものを、その場所のずれとして出します。

- 基準の軌跡は、`--slam-dir` の SLAM の出力 (UTM に揃った軌跡) か、`--odom-fit` (`/odom` を RTK の測位に剛体で当てはめた軌跡。短い走行向け) のどちらか。
  - SLAM の軌跡は、RTK のない区間を SLAM の形で前後の RTK からつなぐ (外挿)。RTK から離れるほど SLAM のずれが乗る。各点に、RTK の測位までの道のり (`path_to_rtk_m`) を残す。
- 照合は、粗い格子の探索 → 細かい格子の探索 → 距離場での Gauss-Newton の順 (`tools/common/map_align.py`)。各点に、照合の質 (`inlier_ratio`)、曖昧さ (`ambiguity`: 離れた別の候補の尤度の比)、姿勢の共分散 (`cov`) を残す。
- 同じ時刻の GNSS と基準の差 (`gnss.ref_minus_gnss`) も残す。基準の質の目安になる。

```bash
make bag-gnss-map-offset BAG=<bag> MAPS="<dir>/Region1_jikoichi.yaml <dir>/Region2_jikoichi.yaml" SLAM_DIR=<SLAM の出力> TO_TOOLS=1
```

### analyze (`make gnss-map-offset-analyze`)

measure の出力をまとめ、照合の質で絞ったうえで、ずれを次の順に当てはめます。

- `none`: 当てはめない (生のずれ)
- `global`: 地図全体で 1 つの剛体 (回転は地図の原点の周り)。`gnss_transform.yaml` の見直しで直せる分
- `group`: waypoint-tool のグループ (`--wptroj`) ごとの剛体。グループのずらし量の見直しで直せる分

決めるデータと確かめるデータを分けるため、道のりの区切り (`--block-m`) を交互に 2 つの側に振り分け、片方で当てはめてもう片方で確かめた残差を出します (走行は区画を順に通るので、時間で前後半に分けるとグループが重ならない)。
グループごとの表には、プロジェクトの手のずらし量 (`pose_offset`) と、当てはめたずれを打ち消す「直す動き」を並べます。出力は `gnss_map_offset_report.md` と、ずれの矢印の図 `gnss_map_offset.png`。

```bash
make gnss-map-offset-analyze INPUTS="<measure の json> ..." WPTROJ=<プロジェクトの .wptroj> OUT_DIR=/app/tools/data/gnss_map
```

---

## `move_wptroj_groups.py` (waypoint-tool のグループを動かす)

waypoint-tool のプロジェクト (`.wptroj`) のグループを、地図の原点の周りの剛体 `R(dyaw)·p + (dx, dy)` で動かし、別名で保存します。
waypoint-tool の画面の「グループの位置調整」と同じ処理で、動かす量には analyze の「直す動き」を使います。

- 出力の地図に効くのは、グループの中の地図レイヤーの原点 (`origin_override`) と切り抜きの矩形です。グループの `pose_offset` は動かした量の記録にすぎず、書き換えても地図は変わりません。
- 扱うのは、`origin_override` を持つ地図レイヤーだけのグループです。手描きのレイヤーなどを含むグループは止めるので、waypoint-tool の画面で動かします。
- ウェイポイントは動かしません。地図は、保存したプロジェクトを waypoint-tool で開いてエクスポートします。
- グループの境目では、両側のレイヤーの姿勢が違う分だけ壁に段差ができます。エクスポートしたら境目を確かめ、計測をやり直します。

```bash
python3 tools/scripts/move_wptroj_groups.py <元の .wptroj> <保存先の .wptroj> --move ario -0.365 1.943 -0.602 --move yubinkyoku1 -0.724 0.286 -0.005
```

---

## `make_sim_gnss_transform.py` (シミュレータの gnss_transform.yaml)

シミュレータのワールド SDF の `spherical_coordinates` (ワールド原点の緯度経度) から、GNSS ブリッジ (`slam_gnss_nav_bridge_node`) が使う `gnss_transform.yaml` を作ります。
シミュレータの地図は、東・北に揃っていて、UTM のグリッドとは子午線収束角の分だけずれます。そのずれを `map_rotation_rad` に入れます ([slam_gnss_2d/doc/gnss_transform.md](../../slam_gnss_2d/doc/gnss_transform.md))。

```bash
python3 tools/scripts/make_sim_gnss_transform.py mg_simulation/worlds/warehouse.sdf -o mg_simulation/maps/warehouse/gnss_transform.yaml
```
