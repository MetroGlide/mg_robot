"""1 周期の入力 (Snapshot) と、それを作る標本の置き場 (SampleStore)。ROS に依存しない。

入力のアダプタ (ノードの購読、オフライン評価の rosbag の読み込み) は、標本を SampleStore に入れる。
周期ごとに SampleStore.snapshot(stamp) で Snapshot を作り、判定に渡す。

Snapshot は、その周期の中で使い回せる計算結果のキャッシュ (cached) を持つ。
地図の距離場のように周期をまたいで使い回すものは、ReferenceData が持つ。
"""
import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Mapping, Optional, Tuple

from .geometry import Pose2
from .grid import LocalDistanceField, OccupancyMap, build_local_distance_field
from .interfaces import MotionSample, PoseSample, ScanSample, SnapshotLike, SourceKind
from .timeline import AlignmentConfig, History, MotionHistory, Timeline


class DistanceFieldCache:
    """姿勢の周りの距離場を、ロボットが rebuild_distance_m 動くまで使い回す。"""

    def __init__(self, rebuild_distance_m: float = 5.0) -> None:
        self.rebuild_distance_m = rebuild_distance_m
        self._fields: Dict[float, Tuple[Tuple[float, float], Optional[LocalDistanceField]]] = {}
        self.builds = 0

    def clear(self) -> None:
        self._fields.clear()

    def around(self, occupancy: OccupancyMap, x: float, y: float, radius: float) -> Optional[LocalDistanceField]:
        cached = self._fields.get(radius)
        if cached is not None:
            center, built = cached
            if math.hypot(x - center[0], y - center[1]) <= self.rebuild_distance_m:
                return built
        built = build_local_distance_field(occupancy, x, y, radius)
        self.builds += 1
        self._fields[radius] = ((x, y), built)
        return built


@dataclass
class ReferenceData:
    """地図とスキャン。map_version は地図が替わるたびに増える。"""
    occupancy: Optional[OccupancyMap] = None
    map_version: int = 0
    scan: Optional[ScanSample] = None
    fields: DistanceFieldCache = field(default_factory=DistanceFieldCache)

    def set_map(self, occupancy: OccupancyMap) -> None:
        self.occupancy = occupancy
        self.map_version += 1
        self.fields.clear()


class SampleStore:
    """情報源ごとの標本の履歴。情報源の名前と種類は、設定から決まる。"""

    def __init__(
        self, kinds: Mapping[str, SourceKind], motion_priority: Tuple[str, ...] = (),
        horizon_sec: float = 30.0, max_extrapolation_sec: float = 0.5,
        alignment: Optional[AlignmentConfig] = None,
    ) -> None:
        self.kinds = dict(kinds)
        self.poses: Dict[str, History[PoseSample]] = {}
        self.motions: Dict[str, MotionHistory] = {}
        self.references: Dict[str, ReferenceData] = {}
        for name, kind in self.kinds.items():
            if kind == SourceKind.POSE:
                self.poses[name] = History(horizon_sec)
            elif kind == SourceKind.MOTION:
                self.motions[name] = MotionHistory(horizon_sec, max_extrapolation_sec)
            else:
                self.references[name] = ReferenceData()
        order = list(motion_priority) + [n for n in self.motions if n not in motion_priority]
        self.timeline = Timeline([(n, self.motions[n]) for n in order], alignment or AlignmentConfig())
        self.extras: Dict[str, Any] = {}

    def add_pose(self, name: str, sample: PoseSample) -> None:
        self.poses[name].add(sample)

    def add_motion(self, name: str, sample: MotionSample) -> None:
        self.motions[name].add(sample)

    def set_map(self, name: str, occupancy: OccupancyMap) -> None:
        self.references[name].set_map(occupancy)

    def set_scan(self, name: str, scan: ScanSample) -> None:
        self.references[name].scan = scan

    def clear_history(self) -> None:
        """姿勢と動きの履歴を捨てる (時刻が戻ったとき、外からの再初期化など)。"""
        for history in self.poses.values():
            history.clear()
        for motion in self.motions.values():
            motion.clear()

    def snapshot(self, stamp: float) -> 'Snapshot':
        return Snapshot(stamp, self, dict(self.extras))


class Snapshot(SnapshotLike):
    def __init__(self, stamp: float, store: SampleStore, extras: Dict[str, Any]) -> None:
        self.stamp = stamp
        self.extras = extras
        self._store = store
        self._cache: Dict[Any, Any] = {}

    @property
    def timeline(self) -> Timeline:
        return self._store.timeline

    def pose(self, name: str, max_age: Optional[float] = None) -> Optional[PoseSample]:
        """最新の標本 (時刻は移さない)。max_age より古ければ None。"""
        sample = self._store.poses[name].latest()
        if sample is None or (max_age is not None and self.stamp - sample.stamp > max_age):
            return None
        return sample

    def pose_history(self, name: str) -> History[PoseSample]:
        return self._store.poses[name]

    def pose_aligned(
        self, name: str, max_age: Optional[float] = None, stamp: Optional[float] = None,
    ) -> Optional[Tuple[PoseSample, float]]:
        """最新の標本を stamp (既定は Snapshot の時刻) に移したものと、元の標本の古さ [s]。"""
        sample = self.pose(name, max_age)
        if sample is None:
            return None
        target = self.stamp if stamp is None else stamp
        key = ('pose_aligned', name, sample.stamp, target)
        aligned = self.cached(key, lambda: self.timeline.align(sample, target))
        if aligned is None:
            return None
        return aligned, self.stamp - sample.stamp

    def motion_at(self, name: str, stamp: float) -> Optional[Pose2]:
        return self._store.motions[name].at(stamp)

    def reference(self, name: str) -> ReferenceData:
        return self._store.references[name]

    def cached(self, key: Any, compute: Callable[[], Any]) -> Any:
        if key not in self._cache:
            self._cache[key] = compute()
        return self._cache[key]
