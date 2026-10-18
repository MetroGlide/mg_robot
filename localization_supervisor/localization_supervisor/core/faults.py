"""Snapshot の段階での故障注入 (オフライン評価用。ROS に依存しない)。

記録済みの入力の標本を書き換えて、正解の付いた異常の事例を作る。時刻は評価の開始からの経過秒。
書式は再生評価の故障注入と同じ形 (faults: のリスト。type と、at または start / end と、種類ごとの引数)。

  - {type: pose_offset, source: amcl, at: 60.0, offset: {x: 3.0, y: 0.0, yaw: 0.0}}
        姿勢を地図座標でずらす (end を書かなければ、その後ずっと)。followers: {ekf: 0.5} で、融合の結果も
        その割合だけずらす (推定器が引きずられた状態)
  - {type: pose_drift, source: amcl, start: 40.0, end: 100.0, rate: {x: 0.05, y: 0.0, yaw: 0.0}}
        ずれを、1 秒あたり rate ずつ増やす。end の後はずれを保つ (hold: false なら元に戻す)。followers も使える
  - {type: source_drop, source: gnss, start: 100.0, end: 140.0}
        標本を捨てる
  - {type: scan_sector_drop, source: map_scan, start: 50.0, end: 80.0, sector_deg: [-90.0, 90.0]}
        センサ座標の角度の範囲の点を欠測にする
  - {type: scan_thin, source: map_scan, start: 50.0, end: 80.0, keep_ratio: 0.3}
        点を等間隔に間引く
"""
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping

import numpy as np

from .geometry import wrap_angle
from .interfaces import MotionSample, PoseSample, ScanSample, SourceKind
from .snapshot import SampleStore

REQUIRED: Dict[str, List[str]] = {
    'pose_offset': ['source', 'offset'],
    'pose_drift': ['source', 'rate'],
    'source_drop': ['source'],
    'scan_sector_drop': ['source', 'sector_deg'],
    'scan_thin': ['source', 'keep_ratio'],
}
OPEN_ENDED = {'pose_offset'}


@dataclass
class Fault:
    type: str
    start: float
    end: float
    source: str
    params: Dict[str, Any] = field(default_factory=dict)

    def active(self, t: float) -> bool:
        return self.start <= t < self.end

    def label_end(self, run_end: float) -> float:
        """評価で「異常がある」とみなす区間の終わり。ずれを残す故障は、評価の終わりまで。"""
        persists = self.type == 'pose_drift' and self.params.get('hold', True)
        return run_end if persists else min(self.end, run_end)


def parse_faults(data: Mapping[str, Any]) -> List[Fault]:
    faults: List[Fault] = []
    for i, raw in enumerate(data.get('faults', []) or []):
        kind = raw.get('type')
        if kind not in REQUIRED:
            raise ValueError(f'faults[{i}]: unknown type {kind!r} (known: {sorted(REQUIRED)})')
        params = {k: v for k, v in raw.items() if k not in ('type', 'at', 'start', 'end', 'source')}
        for key in REQUIRED[kind]:
            if key not in raw and key not in params:
                raise ValueError(f'faults[{i}] ({kind}): {key!r} is required')
        if 'at' in raw:
            start = float(raw['at'])
        elif 'start' in raw:
            start = float(raw['start'])
        else:
            raise ValueError(f'faults[{i}] ({kind}): at or start is required')
        if 'end' in raw:
            end = float(raw['end'])
        elif kind in OPEN_ENDED:
            end = math.inf
        else:
            raise ValueError(f'faults[{i}] ({kind}): end is required')
        if end <= start:
            raise ValueError(f'faults[{i}] ({kind}): end must be after start')
        faults.append(Fault(kind, start, end, raw['source'], params))
    faults.sort(key=lambda f: f.start)
    return faults


def _vector(raw: Mapping[str, Any]) -> np.ndarray:
    return np.array([float(raw.get('x', 0.0)), float(raw.get('y', 0.0)), float(raw.get('yaw', 0.0))])


class FaultInjector:
    def __init__(self, faults: List[Fault], t0: float) -> None:
        self.faults = faults
        self.t0 = t0

    def pose_offset(self, source: str, stamp: float) -> np.ndarray:
        """source の stamp の標本に足すずれ (x, y, yaw)。"""
        t = stamp - self.t0
        total = np.zeros(3)
        for fault in self.faults:
            if fault.type not in ('pose_offset', 'pose_drift') or t < fault.start:
                continue
            if fault.source == source:
                share = 1.0
            else:
                share = float(fault.params.get('followers', {}).get(source, 0.0))
                if share == 0.0:
                    continue
            if fault.type == 'pose_offset':
                if t < fault.end:
                    total += share * _vector(fault.params['offset'])
            else:
                if t >= fault.end and not fault.params.get('hold', True):
                    continue
                elapsed = min(t, fault.end) - fault.start
                total += share * elapsed * _vector(fault.params['rate'])
        return total

    def dropped(self, source: str, stamp: float) -> bool:
        t = stamp - self.t0
        return any(f.type == 'source_drop' and f.source == source and f.active(t) for f in self.faults)

    def scan(self, source: str, scan: ScanSample) -> ScanSample:
        t = scan.stamp - self.t0
        ranges = scan.ranges
        for fault in self.faults:
            if fault.source != source or not fault.active(t):
                continue
            if fault.type == 'scan_sector_drop':
                angles = np.degrees(scan.angle_min + scan.angle_increment * np.arange(ranges.size))
                angles = (angles + 180.0) % 360.0 - 180.0
                low, high = fault.params['sector_deg']
                ranges = np.where((angles >= low) & (angles <= high), np.inf, ranges)
            elif fault.type == 'scan_thin':
                keep = max(1, int(round(ranges.size * float(fault.params['keep_ratio']))))
                mask = np.full(ranges.size, np.inf)
                index = np.linspace(0, ranges.size - 1, keep).astype(int)
                mask[index] = ranges[index]
                ranges = mask
        if ranges is scan.ranges:
            return scan
        return ScanSample(scan.stamp, ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                          scan.range_max, scan.sensor_pose)


class FaultInjectingStore(SampleStore):
    """標本を入れるときに故障を注入する SampleStore。"""

    def __init__(self, injector: FaultInjector, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.injector = injector

    def add_pose(self, name: str, sample: PoseSample) -> None:
        if self.injector.dropped(name, sample.stamp):
            return
        offset = self.injector.pose_offset(name, sample.stamp)
        if offset.any():
            pose = (sample.pose[0] + offset[0], sample.pose[1] + offset[1], wrap_angle(sample.pose[2] + offset[2]))
            sample = PoseSample(sample.stamp, pose, sample.cov)
        super().add_pose(name, sample)

    def add_motion(self, name: str, sample: MotionSample) -> None:
        if not self.injector.dropped(name, sample.stamp):
            super().add_motion(name, sample)

    def set_scan(self, name: str, scan: ScanSample) -> None:
        if not self.injector.dropped(name, scan.stamp):
            super().set_scan(name, self.injector.scan(name, scan))


def validate_sources(faults: List[Fault], kinds: Mapping[str, SourceKind]) -> None:
    """故障の source と followers が設定の情報源と種類に合うか。"""
    for fault in faults:
        kind = kinds.get(fault.source)
        if kind is None:
            raise ValueError(f'fault {fault.type}: unknown source {fault.source!r}')
        if fault.type in ('pose_offset', 'pose_drift'):
            names = [fault.source] + list(fault.params.get('followers', {}))
            if any(kinds.get(n) != SourceKind.POSE for n in names):
                raise ValueError(f'fault {fault.type}: {names} must be pose sources')
        if fault.type.startswith('scan_') and kind != SourceKind.REFERENCE:
            raise ValueError(f'fault {fault.type}: {fault.source!r} must be a reference source')
