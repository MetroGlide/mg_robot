from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Literal, Sequence, Tuple

Frame = Literal["map", "world", "robot"]


@dataclass(frozen=True)
class Pose:
    """解決済みの 2.5D 姿勢 (x, y, z, yaw)。どの座標系かは利用側が管理する。"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    def compose(self, other: "Pose") -> "Pose":
        """self を基準とした相対姿勢 other を self の親座標系へ変換する。"""
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return Pose(
            x=self.x + other.x * c - other.y * s,
            y=self.y + other.x * s + other.y * c,
            z=self.z + other.z,
            yaw=normalize_angle(self.yaw + other.yaw),
        )

    def inverse(self) -> "Pose":
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return Pose(
            x=-(self.x * c + self.y * s),
            y=-(-self.x * s + self.y * c),
            z=-self.z,
            yaw=normalize_angle(-self.yaw),
        )

    def distance_xy(self, other: "Pose") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass
class PoseSpec:
    """シナリオ上の姿勢指定。

    frame:
      map   : ナビゲーションの地図座標 (デフォルト)
      world : シミュレータのワールド座標
      robot : 現在のロボット姿勢を基準とした相対座標
    """
    frame: Frame = "map"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    def as_pose(self) -> Pose:
        return Pose(self.x, self.y, self.z, self.yaw)


@dataclass
class Pose2DSpec:
    """座標系を持たない姿勢指定 (プロファイルの座標変換定義などに使う)。"""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def as_pose(self) -> Pose:
        return Pose(self.x, self.y, 0.0, self.yaw)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float):
    """(x, y, z, w) を返す。"""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


Point = Tuple[float, float]


def transform_polygon(vertices: Sequence[Point], pose: Pose) -> List[Point]:
    """ローカル座標の多角形の頂点を、pose を基準に親座標系へ変換する。"""
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return [(pose.x + x * c - y * s, pose.y + x * s + y * c) for x, y in vertices]


def rectangle_vertices(size_x: float, size_y: float) -> List[Point]:
    """中心を原点とする矩形の頂点 (反時計回り)。"""
    hx = size_x / 2.0
    hy = size_y / 2.0
    return [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length_sq))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def point_in_polygon(p: Point, polygon: Sequence[Point]) -> bool:
    """偶奇規則 (半直線法) による内外判定。"""
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if (yi > p[1]) != (yj > p[1]) and p[0] < (xj - xi) * (p[1] - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _cross(o: Point, a: Point, b: Point) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    d1 = _cross(c, d, a)
    d2 = _cross(c, d, b)
    d3 = _cross(a, b, c)
    d4 = _cross(a, b, d)
    return d1 * d2 < 0.0 and d3 * d4 < 0.0


def _edges(polygon: Sequence[Point]) -> List[Tuple[Point, Point]]:
    return [(polygon[i], polygon[(i + 1) % len(polygon)]) for i in range(len(polygon))]


def polygon_polygon_distance(a: Sequence[Point], b: Sequence[Point]) -> float:
    """2 つの多角形の最短距離。重なっている (内包・交差を含む) 場合は 0。"""
    if point_in_polygon(a[0], b) or point_in_polygon(b[0], a):
        return 0.0
    edges_a = _edges(a)
    edges_b = _edges(b)
    if any(_segments_intersect(p, q, r, s) for p, q in edges_a for r, s in edges_b):
        return 0.0
    distances = [point_segment_distance(v, r, s) for v in a for r, s in edges_b]
    distances += [point_segment_distance(v, p, q) for v in b for p, q in edges_a]
    return min(distances)


def polygon_circle_distance(polygon: Sequence[Point], center: Point, radius: float) -> float:
    """多角形と円の最短距離 (表面間)。重なっている場合は 0。"""
    if point_in_polygon(center, polygon):
        return 0.0
    edge_distance = min(point_segment_distance(center, p, q) for p, q in _edges(polygon))
    return max(0.0, edge_distance - radius)
