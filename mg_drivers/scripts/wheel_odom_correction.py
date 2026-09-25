"""ホイールオドメトリの補正の計算 (ROS に依存しない)。

wheel_odom_corrector_node.py から使う。ROS の環境がなくても単体テストできるよう分けている。

補正のモデル (差動二輪):
  並進  : ds'   = k_v * ds
  旋回  : dyaw' = k_w * dyaw + yaw_bias_per_meter * ds'
k_v は車輪半径 (空気圧・摩耗・荷重)、k_w は実効トレッド幅、yaw_bias_per_meter は左右の車輪半径差で
走行距離に比例して曲がる分を表す。
"""
import math
from typing import Optional, Tuple


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(z: float, w: float) -> float:
    """z 軸まわりの回転だけを持つクォータニオンから yaw を求める。"""
    return 2.0 * math.atan2(z, w)


def shift_stamp(sec: int, nanosec: int, offset_sec: float) -> Tuple[int, int]:
    """スタンプ (sec, nanosec) を offset_sec 秒だけ過去へずらす。"""
    total_ns = sec * 1_000_000_000 + nanosec - int(offset_sec * 1e9)
    return total_ns // 1_000_000_000, total_ns % 1_000_000_000


class OdomCorrector:
    """生のオドメトリ姿勢の増分を補正して積算し直す。"""

    def __init__(self, k_v: float, k_w: float, yaw_bias_per_meter: float,
                 reset_jump_m: float) -> None:
        self.k_v = k_v
        self.k_w = k_w
        self.yaw_bias_per_meter = yaw_bias_per_meter
        self.reset_jump_m = reset_jump_m
        self._prev_raw: Optional[Tuple[float, float, float]] = None
        self._pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def update(self, raw_x: float, raw_y: float, raw_yaw: float) -> Tuple[float, float, float]:
        """生の姿勢を受け取り、補正した姿勢 (x, y, yaw) を返す。"""
        if self._prev_raw is None:
            self._prev_raw = (raw_x, raw_y, raw_yaw)
            self._pose = (raw_x, raw_y, raw_yaw)
            return self._pose

        prev_x, prev_y, prev_yaw = self._prev_raw
        self._prev_raw = (raw_x, raw_y, raw_yaw)
        dx = raw_x - prev_x
        dy = raw_y - prev_y
        if math.hypot(dx, dy) > self.reset_jump_m:
            # ドライバの再起動などで姿勢が飛んだ。補正した姿勢を生の姿勢に合わせ直す
            self._pose = (raw_x, raw_y, raw_yaw)
            return self._pose

        dyaw = wrap_angle(raw_yaw - prev_yaw)
        # 生の増分を、区間の中間の向きの車体座標系に直す
        mid = prev_yaw + 0.5 * dyaw
        c, s = math.cos(mid), math.sin(mid)
        body_x = c * dx + s * dy
        body_y = -s * dx + c * dy

        body_x *= self.k_v
        body_y *= self.k_v
        dyaw = self.k_w * dyaw + self.yaw_bias_per_meter * body_x

        x, y, yaw = self._pose
        mid = yaw + 0.5 * dyaw
        c, s = math.cos(mid), math.sin(mid)
        self._pose = (x + c * body_x - s * body_y,
                      y + s * body_x + c * body_y,
                      wrap_angle(yaw + dyaw))
        return self._pose

    def correct_twist(self, vx: float, wz: float) -> Tuple[float, float]:
        """速度 (vx, wz) を補正する。"""
        vx_corrected = self.k_v * vx
        return vx_corrected, self.k_w * wz + self.yaw_bias_per_meter * vx_corrected
