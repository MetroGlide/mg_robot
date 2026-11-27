"""1 周期の入力 (Snapshot) と、それを作る標本の置き場 (SampleStore)。ROS に依存しない。

入力のアダプタ (ノードの購読、オフライン評価の rosbag の読み込み) は、標本を SampleStore に入れる。
周期ごとに SampleStore.snapshot(stamp) で Snapshot を作り、判定に渡す。

Snapshot は、その周期の中で使い回せる計算結果のキャッシュ (cached) を持つ。
地図の距離場のように周期をまたいで使い回すものは、ReferenceData が持つ。
"""
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, Mapping, Optional, Tuple

import numpy as np

from .geometry import Pose2, wrap_angle
from .grid import LocalDistanceField, OccupancyMap, build_local_distance_field
from .interfaces import HypothesisSample, MotionSample, PoseSample, ScanSample, SnapshotLike, SourceKind
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


class ScaleHistory:
    """情報源の分散にかけるよう、受け取る側に出した倍率の履歴 (出した時刻と倍率)。"""

    def __init__(self, delay_sec: float, horizon_sec: float) -> None:
        self.delay_sec = delay_sec
        self._horizon = horizon_sec
        self._entries: Deque[Tuple[float, float]] = deque()

    def note(self, stamp: float, scale: float) -> None:
        if self._entries and stamp < self._entries[-1][0]:
            self._entries.clear()
        self._entries.append((stamp, scale))
        while len(self._entries) > 1 and stamp - self._entries[1][0] > self._horizon:
            self._entries.popleft()

    def at(self, stamp: float) -> float:
        """stamp の標本に効いていた倍率 (出してから delay_sec 後に効く。履歴がなければ 1)。"""
        scale = 1.0
        for noted, value in self._entries:
            if noted + self.delay_sec > stamp:
                break
            scale = value
        return scale


@dataclass
class HoldConfig:
    """動いたときだけ標本を出す姿勢の情報源 (AMCL など) の、最後の標本を今の推定として使い続ける範囲。

    最後の標本の後の動き (動きの情報源で積算) が distance_m と rotation_rad の両方以下のあいだは、
    標本が古くても、今の時刻に移したものを使う。
    """
    distance_m: float
    rotation_rad: float = math.inf


@dataclass
class _Held:
    sample: PoseSample
    # 元の標本の時刻 (これより新しい標本が届いたら、そこから持ち直す)
    origin: float
    distance: float
    rotation: float


class SampleStore:
    """情報源ごとの標本の履歴。情報源の名前と種類は、設定から決まる。

    track_scale した姿勢の情報源は、受け取る側が分散に倍率をかけて配信するので、標本の位置の分散を、
    その時点で効いていた倍率で割って、倍率をかける前に戻してから残す (判定・候補・残差は、倍率に左右されない)。

    holds に書いた姿勢の情報源は、snapshot のたびに、最後の標本をその時刻へ移して持っておく (HoldConfig)。
    止まっているあいだに標本が古くなっても、判定が「分からない」にならない。
    """

    def __init__(
        self, kinds: Mapping[str, SourceKind], motion_priority: Tuple[str, ...] = (),
        horizon_sec: float = 30.0, max_extrapolation_sec: float = 0.5,
        alignment: Optional[AlignmentConfig] = None, holds: Optional[Mapping[str, HoldConfig]] = None,
    ) -> None:
        self.kinds = dict(kinds)
        self.poses: Dict[str, History[PoseSample]] = {}
        self.motions: Dict[str, MotionHistory] = {}
        self.references: Dict[str, ReferenceData] = {}
        self.hypotheses: Dict[str, History[HypothesisSample]] = {}
        for name, kind in self.kinds.items():
            if kind == SourceKind.POSE:
                self.poses[name] = History(horizon_sec)
            elif kind == SourceKind.MOTION:
                self.motions[name] = MotionHistory(horizon_sec, max_extrapolation_sec)
            elif kind == SourceKind.HYPOTHESES:
                self.hypotheses[name] = History(horizon_sec)
            else:
                self.references[name] = ReferenceData()
        order = list(motion_priority) + [n for n in self.motions if n not in motion_priority]
        self.timeline = Timeline([(n, self.motions[n]) for n in order], alignment or AlignmentConfig())
        self._horizon_sec = horizon_sec
        self._scales: Dict[str, ScaleHistory] = {}
        self._holds: Dict[str, HoldConfig] = dict(holds or {})
        for name in self._holds:
            if name not in self.poses:
                raise ValueError(f'hold: {name!r} is not a pose source')
        self._held: Dict[str, _Held] = {}

    def track_scale(self, name: str, delay_sec: float) -> None:
        """name の標本の分散に、受け取る側がかけた倍率を割り戻す。"""
        self._scales[name] = ScaleHistory(delay_sec, self._horizon_sec)

    def note_scale(self, name: str, stamp: float, scale: float) -> None:
        """name の分散にかけるよう出した倍率を記録する (track_scale していなければ何もしない)。"""
        if name in self._scales:
            self._scales[name].note(stamp, scale)

    def add_pose(self, name: str, sample: PoseSample) -> None:
        scales = self._scales.get(name)
        if scales is not None:
            factor = scales.at(sample.stamp)
            if factor != 1.0:
                cov = np.array(sample.cov, dtype=float, copy=True)
                cov[0, 0] /= factor
                cov[1, 1] /= factor
                sample = PoseSample(sample.stamp, sample.pose, cov)
        self.poses[name].add(sample)

    def add_motion(self, name: str, sample: MotionSample) -> None:
        self.motions[name].add(sample)

    def add_hypotheses(self, name: str, sample: HypothesisSample) -> None:
        self.hypotheses[name].add(sample)

    def set_map(self, name: str, occupancy: OccupancyMap) -> None:
        self.references[name].set_map(occupancy)

    def set_scan(self, name: str, scan: ScanSample) -> None:
        self.references[name].scan = scan

    def held(self, name: str) -> Optional[PoseSample]:
        """name の最後の標本を、最後に snapshot した時刻へ移したもの。持っていなければ None。"""
        held = self._held.get(name)
        return None if held is None else held.sample

    def shift_held(self, name: str, delta: Pose2) -> None:
        """持っている標本を delta (x, y, yaw) だけずらす (推定器を初期化し直したことにする)。"""
        held = self._held.get(name)
        if held is not None:
            pose = held.sample.pose
            moved = (pose[0] + delta[0], pose[1] + delta[1], wrap_angle(pose[2] + delta[2]))
            held.sample = PoseSample(held.sample.stamp, moved, held.sample.cov)

    def _refresh_held(self, stamp: float) -> None:
        for name, hold in self._holds.items():
            latest = self.poses[name].latest()
            if latest is None:
                self._held.pop(name, None)
                continue
            held = self._held.get(name)
            if held is None or held.origin < latest.stamp:
                held = _Held(latest, latest.stamp, 0.0, 0.0)
            if stamp <= held.sample.stamp:
                self._held[name] = held
                continue
            motion = self.timeline.motion(held.sample.stamp, stamp)
            aligned = None if motion is None else self.timeline.align(held.sample, stamp)
            if motion is None or aligned is None:
                self._held.pop(name, None)
                continue
            distance = held.distance + motion.distance
            rotation = held.rotation + motion.rotation
            if distance > hold.distance_m or rotation > hold.rotation_rad:
                self._held.pop(name, None)
                continue
            self._held[name] = _Held(aligned, held.origin, distance, rotation)

    def snapshot(self, stamp: float) -> 'Snapshot':
        self._refresh_held(stamp)
        return Snapshot(stamp, self)


class Snapshot(SnapshotLike):
    def __init__(self, stamp: float, store: SampleStore) -> None:
        self.stamp = stamp
        self._store = store
        self._cache: Dict[Any, Any] = {}

    @property
    def timeline(self) -> Timeline:
        return self._store.timeline

    def pose(self, name: str, max_age: Optional[float] = None) -> Optional[PoseSample]:
        """最新の標本 (時刻は移さない)。max_age より古ければ、持っている標本 (SampleStore.held) か None。"""
        sample = self._store.poses[name].latest()
        if sample is None or max_age is None or self.stamp - sample.stamp <= max_age:
            return sample
        held = self._store.held(name)
        if held is None or held.stamp < sample.stamp or self.stamp - held.stamp > max_age:
            return None
        return held

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

    def hypotheses(self, name: str, max_age: Optional[float] = None) -> Optional[HypothesisSample]:
        """最新の仮説の集合 (時刻は移さない)。max_age より古ければ None。"""
        sample = self._store.hypotheses[name].latest()
        if sample is None or (max_age is not None and self.stamp - sample.stamp > max_age):
            return None
        return sample

    def align(self, sample: PoseSample, stamp: Optional[float] = None) -> Optional[PoseSample]:
        """sample を stamp (既定は Snapshot の時刻) に、動きの情報源で移す。移せなければ None。"""
        return self.timeline.align(sample, self.stamp if stamp is None else stamp)

    def reference(self, name: str) -> ReferenceData:
        return self._store.references[name]

    def cached(self, key: Any, compute: Callable[[], Any]) -> Any:
        if key not in self._cache:
            self._cache[key] = compute()
        return self._cache[key]
