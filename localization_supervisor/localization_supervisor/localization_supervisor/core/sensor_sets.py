"""センサ構成: 情報源の欠けから、今のセンサ構成 (全部 / 例: LiDAR なし) を決める (ROS に依存しない)。

組ごとに、欠けると入る情報源 (missing) を書く。どれかが missing_after_sec 届かなければその構成に入り、全部が
present_after_sec 届き続ければ出る (全部のセンサ構成 = 空の名前に戻る)。missing_after_sec は、センサの途絶の見張りが
起動し直しを試す時間より長くする (起動し直しで戻る欠けでは構成を変えない)。上から順に、最初に当てはまる組にする。
構成の速度の上限は、走行制約の速度の上限と小さい方を使う (0 は上限なし)。
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Tuple

from .interfaces import SourceKind

ALL = ''


@dataclass
class SensorSetConfig:
    name: str = ''
    missing: List[str] = field(default_factory=list)
    missing_after_sec: float = 30.0
    present_after_sec: float = 5.0
    speed_limit_mps: float = 0.0


def latest_stamp(snapshot, name: str, kind: SourceKind) -> Optional[float]:
    """情報源の最も新しい標本の時刻 (姿勢と地図・スキャンの情報源)。"""
    if kind == SourceKind.POSE:
        samples = snapshot.pose_history(name).samples()
        return samples[-1].stamp if samples else None
    if kind == SourceKind.REFERENCE:
        scan = snapshot.reference(name).scan
        return None if scan is None else scan.stamp
    return None


class SensorSetTracker:
    """周期ごとに情報源の届き方を見て、今のセンサ構成を決める。"""

    def __init__(self, configs: List[SensorSetConfig], kinds: Mapping[str, SourceKind]) -> None:
        self.configs = configs
        self.kinds = dict(kinds)
        self.current = ALL
        self._started: Optional[float] = None
        # 情報源ごとの、最後に届いた時刻 (まだなら起動の時刻) と、届き続けている始まり
        self._present_since: Dict[str, Optional[float]] = {}

    def config(self, name: str) -> Optional[SensorSetConfig]:
        return next((c for c in self.configs if c.name == name), None)

    def update(self, snapshot) -> Tuple[str, Optional[Tuple[str, str]]]:
        """(今のセンサ構成, 変わったなら (前, 後))。"""
        now = snapshot.stamp
        if self._started is None:
            self._started = now
        missing_for: Dict[str, float] = {}
        present_for: Dict[str, float] = {}
        for name in {s for c in self.configs for s in c.missing}:
            stamp = latest_stamp(snapshot, name, self.kinds[name])
            last = self._started if stamp is None else max(stamp, self._started)
            missing_for[name] = now - last
            fresh = stamp is not None and now - stamp <= 1.0
            if not fresh:
                self._present_since[name] = None
            elif self._present_since.get(name) is None:
                self._present_since[name] = now
            since = self._present_since.get(name)
            present_for[name] = -math.inf if since is None else now - since
        wanted = ALL
        for c in self.configs:
            if any(missing_for[s] >= c.missing_after_sec for s in c.missing):
                wanted = c.name
                break
        before = self.current
        if wanted != before:
            # 全部のセンサ構成へ戻るのは、欠けていた情報源が全部 present_after_sec 届き続けてから
            leaving = self.config(before)
            if wanted == ALL and leaving is not None and not all(
                    present_for[s] >= leaving.present_after_sec for s in leaving.missing):
                return before, None
            self.current = wanted
            return wanted, (before, wanted)
        return before, None
