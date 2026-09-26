"""自己位置の異常を判定する計算 (ROS に依存しない)。

姿勢は SE(2) の (x, y, yaw)。
"""
import math
from typing import Tuple

Pose2 = Tuple[float, float, float]


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compose(a: Pose2, b: Pose2) -> Pose2:
    """SE(2) の合成 a * b。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap_angle(a[2] + b[2]))


def inverse(a: Pose2) -> Pose2:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap_angle(-a[2]))


def implied_map_odom(map_base: Pose2, odom_base: Pose2) -> Pose2:
    """AMCL の姿勢 (map->base) と、その時刻のオドメトリ (odom->base) が示す map->odom。

    オドメトリが正しければ map->odom はゆっくりしか変わらないので、連続する推定で大きく変わったら
    AMCL が飛んだことになる。
    """
    return compose(map_base, inverse(odom_base))


def pose_jump(previous: Pose2, current: Pose2, odom_base: Pose2) -> Tuple[float, float]:
    """2 つの map->odom の違いが、ロボット (odom_base の位置) の map 上の姿勢に与える飛び (位置 [m]、|yaw| [rad])。

    map->odom の並進どうしを比べると、odom の原点から離れるほど yaw の小さな違いが大きな位置の差になる
    (500 m 走った後では 0.01 rad で 5 m)。同じ odom_base に前後の map->odom を適用して比べる。
    """
    before = compose(previous, odom_base)
    after = compose(current, odom_base)
    return (math.hypot(after[0] - before[0], after[1] - before[1]),
            abs(wrap_angle(after[2] - before[2])))


def mahalanobis_sq_xy(dx: float, dy: float, var_x: float, var_y: float) -> float:
    """対角共分散の 2 次元のマハラノビス距離の二乗。分散が 0 以下のときは無限大にならないよう下限をかける。"""
    floor = 1e-6
    return dx * dx / max(var_x, floor) + dy * dy / max(var_y, floor)


def amcl_gnss_d2(
    amcl_xy: Tuple[float, float], amcl_var: Tuple[float, float],
    gps_xy: Tuple[float, float], gps_var: float, extra_var: float = 0.0,
) -> float:
    """AMCL と GNSS の位置の差のマハラノビス距離の二乗 (2 自由度)。

    extra_var は、2 つの時刻のずれの間にロボットが動いた分などの追加の分散 [m^2]。
    """
    return mahalanobis_sq_xy(
        amcl_xy[0] - gps_xy[0], amcl_xy[1] - gps_xy[1],
        amcl_var[0] + gps_var + extra_var, amcl_var[1] + gps_var + extra_var)
