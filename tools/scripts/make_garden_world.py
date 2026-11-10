#!/usr/bin/env python3
"""
make_garden_world.py

植栽 (生垣・花壇) に囲まれた経路のワールドと地図を、配置の定義 (YAML) から作る。2D LiDAR が植栽を見て点が
不安定になり、地図の植栽が太い帯になり、角を抜けると見える物が大きく変わる、という AMCL が弱い条件を
Gazebo で再現するために使う。同じ定義と乱数の種からは、同じワールドと地図ができる。

使い方:
  make_garden_world.py <配置の YAML> --world <出力 SDF> --map-dir <地図の出力ディレクトリ>

出力:
  SDF                     : 植栽の要素 (細い箱)、壁、柱を置いたワールド
  planning.yaml/.pgm      : 計画用の地図。植栽は帯 (要素の範囲を埋めたもの)、壁と柱は実寸
  localization<名>.yaml   : 測位用の地図。走査面の高さに届く要素の周りに、点を散らして描く
                            (SLAM で植栽を溜めた地図の再現)。散らす幅ごとに 1 枚 (localization_variants)
  waypoints.yaml          : 経路の角を通る waypoint (mg_waypoint_navigation の version 2.0)

配置の YAML の項目は、mg_simulation/worlds/garden_layout.yaml の注釈を参照。
"""
import argparse
import math
import os
from typing import Dict, List, Tuple

import numpy as np
import yaml
from PIL import Image

OCCUPIED_PIXEL = 0
FREE_PIXEL = 254


def distance_to_polyline(points: np.ndarray, polyline: np.ndarray) -> np.ndarray:
    """各点から折れ線 (閉じない) までの最短距離。"""
    best = np.full(points.shape[0], np.inf)
    for a, b in zip(polyline[:-1], polyline[1:]):
        ab = b - a
        t = np.clip(((points - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
        nearest = a + t[:, None] * ab
        best = np.minimum(best, np.hypot(*(points - nearest).T))
    return best


def make_hedge_elements(layout: Dict, rng: np.random.Generator) -> List[Dict]:
    """経路の両側の帯 (経路からの距離が [inner, inner + depth]) に、植栽の要素を散らす。"""
    route = np.array(layout["route"], dtype=float)
    hedge = layout["hedge"]
    inner, depth = float(hedge["inner_m"]), float(hedge["depth_m"])
    low = route.min(axis=0) - inner - depth
    high = route.max(axis=0) + inner + depth
    area = float(np.prod(high - low))
    candidates = rng.uniform(low, high, size=(int(area * float(hedge["density_per_m2"])), 2))
    distance = distance_to_polyline(candidates, route)
    keep = (distance >= inner) & (distance <= inner + depth)
    for gap in layout.get("hedge_gaps", []):
        keep &= np.hypot(*(candidates - np.array(gap["at"], dtype=float)).T) > float(gap["radius_m"])
    elements = []
    for x, y in candidates[keep]:
        size = rng.uniform(*hedge["size_m"])
        elements.append({"x": float(x), "y": float(y), "yaw": float(rng.uniform(0.0, math.pi)),
                         "sx": float(size), "sy": float(rng.uniform(0.02, size)),
                         "height": float(rng.uniform(*hedge["height_m"]))})
    return elements


def box_link_xml(name: str, x: float, y: float, yaw: float, sx: float, sy: float, height: float,
                 color: str) -> str:
    geometry = f"<geometry><box><size>{sx:.3f} {sy:.3f} {height:.3f}</size></box></geometry>"
    return (f"      <collision name='{name}_c'><pose>{x:.3f} {y:.3f} {height / 2:.3f} 0 0 {yaw:.3f}</pose>"
            f"{geometry}</collision>\n"
            f"      <visual name='{name}_v'><pose>{x:.3f} {y:.3f} {height / 2:.3f} 0 0 {yaw:.3f}</pose>"
            f"{geometry}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>\n")


def wall_box(wall: Dict) -> Tuple[float, float, float, float, float]:
    """壁 (from, to, thickness) の中心・向き・長さ・厚さ。"""
    (x0, y0), (x1, y1) = wall["from"], wall["to"]
    return ((x0 + x1) / 2, (y0 + y1) / 2, math.atan2(y1 - y0, x1 - x0),
            math.hypot(x1 - x0, y1 - y0), float(wall.get("thickness_m", 0.2)))


def write_world(path: str, name: str, layout: Dict, elements: List[Dict]) -> None:
    links = []
    for i, e in enumerate(elements):
        links.append(box_link_xml(f"h{i}", e["x"], e["y"], e["yaw"], e["sx"], e["sy"], e["height"],
                                  "0.2 0.5 0.2 1"))
    for i, wall in enumerate(layout.get("walls", [])):
        cx, cy, yaw, length, thickness = wall_box(wall)
        links.append(box_link_xml(f"w{i}", cx, cy, yaw, length, thickness, float(wall.get("height_m", 2.0)),
                                  "0.7 0.7 0.7 1"))
    for i, post in enumerate(layout.get("posts", [])):
        radius = float(post.get("radius_m", 0.15))
        links.append(box_link_xml(f"p{i}", float(post["at"][0]), float(post["at"][1]), 0.0,
                                  2 * radius, 2 * radius, float(post.get("height_m", 2.0)), "0.4 0.3 0.2 1"))
    geo = layout["spherical_coordinates"]
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"""<?xml version='1.0' encoding='UTF-8'?>
<!-- tools/scripts/make_garden_world.py で生成。手で編集せず、配置の YAML を変えて作り直す -->
<sdf version='1.7'>
  <world name='{name}'>
    <physics type="ode">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='libgz-sim-physics-system.so'/>
    <plugin name='gz::sim::systems::UserCommands' filename='libgz-sim-user-commands-system.so'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='libgz-sim-scene-broadcaster-system.so'/>
    <plugin name="gz::sim::systems::Sensors" filename="libgz-sim-sensors-system.so">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::Imu" filename="libgz-sim-imu-system.so"/>
    <plugin name="gz::sim::systems::NavSat" filename="libgz-sim-navsat-system.so"/>
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.3 0.7 0.9 1</background>
      <shadows>0</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>-0.3 0.2 -1</direction>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{geo['latitude_deg']}</latitude_deg>
      <longitude_deg>{geo['longitude_deg']}</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry></collision>
        <visual name='visual'><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.6 0.55 0.45 1</ambient><diffuse>0.6 0.55 0.45 1</diffuse></material></visual>
      </link>
    </model>
    <model name='garden'>
      <static>true</static>
      <link name='link'>
{''.join(links)}      </link>
    </model>
  </world>
</sdf>
""")


class Raster:
    def __init__(self, low: np.ndarray, high: np.ndarray, resolution: float) -> None:
        self.origin = low
        self.resolution = resolution
        self.width = int(math.ceil((high[0] - low[0]) / resolution))
        self.height = int(math.ceil((high[1] - low[1]) / resolution))
        self.occupied = np.zeros((self.height, self.width), dtype=bool)

    def centers(self) -> np.ndarray:
        cols, rows = np.meshgrid(np.arange(self.width), np.arange(self.height))
        return np.stack([self.origin[0] + (cols + 0.5) * self.resolution,
                         self.origin[1] + (rows + 0.5) * self.resolution], axis=-1)

    def mark_points(self, xy: np.ndarray) -> None:
        col = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(int)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        self.occupied[row[inside], col[inside]] = True

    def mark_box(self, cx: float, cy: float, yaw: float, sx: float, sy: float) -> None:
        """回転した長方形を、その内部の点を細かく打って塗る。"""
        step = self.resolution / 2.0
        u, v = np.meshgrid(np.arange(-sx / 2, sx / 2 + 1e-9, step), np.arange(-sy / 2, sy / 2 + 1e-9, step))
        u, v = u.ravel(), v.ravel()
        c, s = math.cos(yaw), math.sin(yaw)
        self.mark_points(np.stack([cx + c * u - s * v, cy + s * u + c * v], axis=1))

    def save(self, directory: str, name: str) -> None:
        image = np.where(self.occupied, OCCUPIED_PIXEL, FREE_PIXEL).astype(np.uint8)[::-1]
        Image.fromarray(image).save(os.path.join(directory, f"{name}.pgm"))
        meta = {"image": f"{name}.pgm", "mode": "trinary", "resolution": self.resolution,
                "origin": [float(self.origin[0]), float(self.origin[1]), 0.0], "negate": 0,
                "occupied_thresh": 0.65, "free_thresh": 0.25}
        with open(os.path.join(directory, f"{name}.yaml"), "w", encoding="utf-8") as f:
            yaml.safe_dump(meta, f, sort_keys=False)


def mark_fixed(raster: Raster, layout: Dict) -> None:
    for wall in layout.get("walls", []):
        cx, cy, yaw, length, thickness = wall_box(wall)
        raster.mark_box(cx, cy, yaw, length, thickness)
    for post in layout.get("posts", []):
        radius = float(post.get("radius_m", 0.15))
        raster.mark_box(float(post["at"][0]), float(post["at"][1]), 0.0, 2 * radius, 2 * radius)


def write_maps(directory: str, layout: Dict, elements: List[Dict], rng: np.random.Generator) -> None:
    route = np.array(layout["route"], dtype=float)
    hedge = layout["hedge"]
    margin = float(hedge["inner_m"]) + float(hedge["depth_m"]) + float(layout.get("map_margin_m", 3.0))
    corners = [np.array(w["from"]) for w in layout.get("walls", [])] + \
        [np.array(w["to"]) for w in layout.get("walls", [])] + \
        [np.array(p["at"]) for p in layout.get("posts", [])]
    points = np.vstack([route] + ([np.array(corners)] if corners else []))
    low = np.floor(points.min(axis=0) - margin)
    high = np.ceil(points.max(axis=0) + margin)
    resolution = float(layout.get("resolution", 0.05))
    os.makedirs(directory, exist_ok=True)

    # 計画用: 植栽は要素の範囲 (経路からの距離が帯の中) を埋めた帯。隙間 (hedge_gaps) は帯から外す
    planning = Raster(low, high, resolution)
    centers = planning.centers().reshape(-1, 2)
    distance = distance_to_polyline(centers, route).reshape(planning.height, planning.width)
    inner, depth = float(hedge["inner_m"]), float(hedge["depth_m"])
    band = (distance >= inner) & (distance <= inner + depth)
    for gap in layout.get("hedge_gaps", []):
        gx, gy = gap["at"]
        band &= np.hypot(planning.centers()[..., 0] - gx, planning.centers()[..., 1] - gy) > float(gap["radius_m"])
    planning.occupied |= band
    mark_fixed(planning, layout)
    planning.save(directory, "planning")

    # 測位用: 走査面の高さに届く要素だけを、その周りに点を散らして描く
    scan_height = float(layout["scan_height_m"])
    visible = [e for e in elements if e["height"] >= scan_height]
    for variant in layout["localization_variants"]:
        raster = Raster(low, high, resolution)
        smear = float(variant["smear_m"])
        for e in visible:
            raster.mark_box(e["x"], e["y"], e["yaw"], e["sx"], e["sy"])
            if smear > 0.0:
                count = int(variant.get("points_per_element", 6))
                raster.mark_points(np.array([e["x"], e["y"]]) + rng.normal(0.0, smear, size=(count, 2)))
        mark_fixed(raster, layout)
        raster.save(directory, "localization" + variant.get("suffix", ""))


def write_waypoints(path: str, route: np.ndarray) -> None:
    waypoints = []
    for i, point in enumerate(route[1:], start=1):
        heading = math.atan2(*(point - route[i - 1])[::-1])
        entry = {"index": i - 1,
                 "pose": {"position": {"x": float(point[0]), "y": float(point[1]), "z": 0.0},
                          "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(heading / 2),
                                          "w": math.cos(heading / 2)}}}
        if i == len(route) - 1:
            entry["navigation"] = {"is_through_point": False}
        waypoints.append(entry)
    with open(path, "w", encoding="utf-8") as f:
        f.write("# tools/scripts/make_garden_world.py で配置の YAML の route から生成\n")
        yaml.safe_dump({"version": "2.0",
                        "defaults": {"reach_tolerance": 0.5, "through_tolerance": 1.5, "is_through_point": True},
                        "waypoints": waypoints}, f, sort_keys=False, allow_unicode=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="植栽に囲まれた経路のワールドと地図を作る")
    parser.add_argument("layout", help="配置の YAML")
    parser.add_argument("--world", required=True, help="出力する SDF")
    parser.add_argument("--map-dir", required=True, help="地図と waypoint の出力ディレクトリ")
    args = parser.parse_args()

    with open(args.layout, encoding="utf-8") as f:
        layout = yaml.safe_load(f)
    rng = np.random.default_rng(int(layout.get("seed", 0)))
    elements = make_hedge_elements(layout, rng)
    name = os.path.splitext(os.path.basename(args.world))[0]
    write_world(args.world, name, layout, elements)
    write_maps(args.map_dir, layout, elements, rng)
    write_waypoints(os.path.join(args.map_dir, "waypoints.yaml"), np.array(layout["route"], dtype=float))
    print(f"保存: {args.world} (植栽の要素 {len(elements)} 個), {args.map_dir}")


if __name__ == "__main__":
    main()
