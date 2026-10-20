"""tf2 のバッファから TF を引く、入力のアダプタの窓口。ノード (購読) もオフライン評価 (bag) も使う。"""
from typing import Optional

import tf2_ros
from rclpy.time import Time

from ..core.geometry import Pose2
from .adapters.base import AdapterContext
from .adapters.standard import yaw_from_quaternion


class TfBufferContext(AdapterContext):
    def __init__(self) -> None:
        self.buffer = tf2_ros.Buffer()

    def lookup(self, parent: str, child: str, stamp: Optional[float] = None) -> Optional[Pose2]:
        when = Time() if stamp is None else Time(nanoseconds=int(stamp * 1e9))
        try:
            t = self.buffer.lookup_transform(parent, child, when).transform
        except tf2_ros.TransformException:
            return None
        return (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
