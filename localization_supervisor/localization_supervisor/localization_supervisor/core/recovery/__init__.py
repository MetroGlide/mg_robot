"""復旧: 候補を作るもの・評価するもの・選び方。設定の type に書く登録名と、クラスの対応。"""
from .generators import AnchorMotion, PoseSource, SubmapMatch
from .scorers import AnchorAgreement, FootprintFree, ScanMatch, SourceAgreement

STANDARD_GENERATORS = {
    'pose_source': PoseSource,
    'anchor_motion': AnchorMotion,
    'submap_match': SubmapMatch,
}

STANDARD_SCORERS = {
    'scan_match': ScanMatch,
    'footprint_free': FootprintFree,
    'source_agreement': SourceAgreement,
    'anchor_agreement': AnchorAgreement,
}
