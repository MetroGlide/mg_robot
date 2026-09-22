"""深度画像から点群配列を作る、ROS 非依存の numpy ヘルパー。"""

from typing import Optional

import numpy as np


def build_point_array(
    depth_m: np.ndarray,
    u_norm: np.ndarray,
    v_norm: np.ndarray,
    min_depth: float,
    max_depth: float,
    bgr: Optional[np.ndarray] = None,
) -> Optional[np.ndarray]:
    """有効画素の点群を float32 の (N, 3) または (N, 4: x,y,z,rgb) 配列にして返す。

    有効画素が無い場合は None を返す。
    """
    valid = (depth_m > min_depth) & (depth_m < max_depth) & np.isfinite(depth_m)
    z = depth_m[valid]
    if z.size == 0:
        return None

    points = np.empty((z.size, 3 if bgr is None else 4), dtype=np.float32)
    points[:, 0] = u_norm[valid] * z
    points[:, 1] = v_norm[valid] * z
    points[:, 2] = z
    if bgr is not None:
        # uint32 に pack した rgb を float32 として格納する (PCL の rgb 形式)
        bgr_valid = bgr[valid].astype(np.uint32)
        rgb_packed = (bgr_valid[:, 2] << 16) | (bgr_valid[:, 1] << 8) | bgr_valid[:, 0]
        points[:, 3] = rgb_packed.view(np.float32)
    return points
