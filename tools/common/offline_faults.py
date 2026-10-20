"""再生評価の故障の定義 (tools/common/faults.py) を、オフライン評価 (localization_supervisor の Snapshot の
段階での故障注入) の定義に直す。情報源の名前は mg_localization_supervisor/config/supervision.yaml に合わせる。

  kidnap     -> pose_offset (amcl)。再生評価と違い、EKF は引きずられない (followers で割合を指定できる)
  amcl_drift -> pose_drift (amcl)
  gnss_bias  -> pose_offset (gnss) の区間
  gnss_drop  -> source_drop (gnss)
  scan_drop  -> scan_sector_drop (map_scan)
  odom_scale -> 直せない (記録済みの EKF・AMCL はすでにオドメトリを使っているため)
"""

from typing import Any, Dict, List, Mapping, Optional

from tools.common.faults import Fault

AMCL = "amcl"
GNSS = "gnss"
SCAN = "map_scan"


def to_offline(faults: List[Fault], followers: Optional[Mapping[str, float]] = None) -> Dict[str, Any]:
    """{"faults": [...]} (localization_supervisor.core.faults.parse_faults の入力) を作る。"""
    followers = dict(followers or {})
    converted: List[Dict[str, Any]] = []
    for fault in faults:
        p = fault.params
        if fault.type == "kidnap":
            entry = {"type": "pose_offset", "source": AMCL, "at": fault.start, "offset": p["offset"]}
            if followers:
                entry["followers"] = followers
        elif fault.type == "amcl_drift":
            entry = {"type": "pose_drift", "source": AMCL, "start": fault.start, "end": fault.end,
                     "rate": p["rate"]}
            if followers:
                entry["followers"] = followers
        elif fault.type == "gnss_bias":
            bias = p["bias"]
            entry = {"type": "pose_offset", "source": GNSS, "start": fault.start, "end": fault.end,
                     "offset": {"x": bias.get("x", 0.0), "y": bias.get("y", 0.0)}}
        elif fault.type == "gnss_drop":
            entry = {"type": "source_drop", "source": GNSS, "start": fault.start, "end": fault.end}
        elif fault.type == "scan_drop":
            entry = {"type": "scan_sector_drop", "source": SCAN, "start": fault.start, "end": fault.end,
                     "sector_deg": p["sector_deg"]}
        else:
            raise ValueError(f"{fault.type} はオフライン評価で注入できません (再生評価を使ってください)")
        converted.append(entry)
    return {"faults": converted}
