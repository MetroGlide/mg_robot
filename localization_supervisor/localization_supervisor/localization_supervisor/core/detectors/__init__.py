"""標準の判定。設定の type に書く登録名と、クラスの対応。"""
from .covariance import Covariance
from .pose_consistency import PoseConsistency
from .pose_jump import PoseJump
from .scan_map import ScanMap
from .submap_map import SubmapMap

STANDARD_DETECTORS = {
    'pose_consistency': PoseConsistency,
    'pose_jump': PoseJump,
    'covariance': Covariance,
    'scan_map': ScanMap,
    'submap_map': SubmapMap,
}
