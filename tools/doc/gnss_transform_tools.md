# 地図と GNSS の変換

地図と GNSS の対応点から `gnss_transform.yaml` の剛体変換を求めるツールと、シミュレータ用の生成ツール。一覧は [tools の README](../README.md)。

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

## `make_sim_gnss_transform.py` (シミュレータの gnss_transform.yaml)

シミュレータのワールド SDF の `spherical_coordinates` (ワールド原点の緯度経度) から、GNSS ブリッジ (`slam_gnss_nav_bridge_node`) が使う `gnss_transform.yaml` を作ります。
シミュレータの地図は、東・北に揃っていて、UTM のグリッドとは子午線収束角の分だけずれます。そのずれを `map_rotation_rad` に入れます ([slam_gnss_2d/doc/gnss_transform.md](../../slam_gnss_2d/doc/gnss_transform.md))。

```bash
python3 tools/scripts/make_sim_gnss_transform.py mg_simulation/worlds/warehouse.sdf -o mg_simulation/maps/warehouse/gnss_transform.yaml
```
