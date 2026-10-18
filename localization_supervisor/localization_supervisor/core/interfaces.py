"""プラグインの約束 (外部に公開する基底クラスと値の型)。ROS に依存しない。

約束の版は CONTRACT_VERSION。版 0 の間は変更してよい (doc/plugins.md)。

  情報源の種類
    pose      : 地図座標の絶対の姿勢と共分散 (PoseSample の履歴)
    motion    : 相対的な動き (MotionSample の履歴。odom->base のような、ゆっくりずれる座標での姿勢)
    reference : 地図とスキャン (ReferenceData)
  プラグイン
    Detector       : 情報源を比べて Evidence を返す
    FusionStrategy : 判定のインスタンスの Evidence を総合して FusionResult を返す
"""
import enum
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .geometry import Pose2

CONTRACT_VERSION = 0


class SourceKind(str, enum.Enum):
    POSE = 'pose'
    MOTION = 'motion'
    REFERENCE = 'reference'


@dataclass
class PoseSample:
    """地図座標の姿勢。cov は x, y, yaw の 3x3 の共分散。"""
    stamp: float
    pose: Pose2
    cov: np.ndarray


@dataclass
class MotionSample:
    """動きの情報源の座標 (odom など) での姿勢。2 つの時刻の差が、その間の動きになる。"""
    stamp: float
    pose: Pose2


@dataclass
class ScanSample:
    """距離センサの 1 回分の走査。sensor_pose は base から見たセンサの姿勢 (分からなければ None)。"""
    stamp: float
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    sensor_pose: Optional[Pose2] = None


class Status(str, enum.Enum):
    NORMAL = 'normal'
    ANOMALY = 'anomaly'
    UNKNOWN = 'unknown'


@dataclass
class Evidence:
    """判定の 1 回分の結果。

    score は正規化した強さで、1.0 が異常のしきい値に当たる (しきい値を超えるほど大きい)。
    判定できないときは status が UNKNOWN で、score は使わない。values は記録用の値。
    """
    status: Status
    score: float = 0.0
    values: Dict[str, float] = field(default_factory=dict)

    @staticmethod
    def unknown(**values: float) -> 'Evidence':
        return Evidence(Status.UNKNOWN, 0.0, dict(values))

    @staticmethod
    def from_score(score: float, **values: float) -> 'Evidence':
        """score が 1.0 を超えれば異常とする。"""
        return Evidence(Status.ANOMALY if score > 1.0 else Status.NORMAL, float(score), dict(values))


@dataclass(frozen=True)
class ParamSpec:
    """プラグインのパラメータの宣言。default が None のものは必須。"""
    type: type
    default: Any = None
    description: str = ''


class Plugin:
    """プラグインの共通の部分。

    PARAMS に、パラメータの名前と ParamSpec を宣言する。読み込むときに、型と必須を検証し、
    既定値を補った dict を self.params に入れる。
    """
    PARAMS: Mapping[str, ParamSpec] = {}

    def __init__(self, name: str, params: Mapping[str, Any]) -> None:
        self.name = name
        self.params = dict(params)


class Detector(Plugin):
    """判定。比べる情報源は、引数の名前 (SOURCES のキー) で受け取る。

    SOURCES  : 引数の名前と、求める情報源の種類
    SUBJECTS : 異常のとき、疑いをかける引数 (情報源の切り分けに使う)。既定は reference 以外のすべて
    EXTRAS   : Snapshot.extras から読む値の名前
    """
    SOURCES: Mapping[str, SourceKind] = {}
    SUBJECTS: Optional[Sequence[str]] = None
    EXTRAS: Sequence[str] = ()

    def __init__(self, name: str, params: Mapping[str, Any], sources: Mapping[str, str]) -> None:
        super().__init__(name, params)
        self.sources = dict(sources)

    def subjects(self) -> List[str]:
        """疑いをかける情報源の名前。"""
        args = self.SUBJECTS
        if args is None:
            args = [arg for arg, kind in self.SOURCES.items() if kind != SourceKind.REFERENCE]
        return [self.sources[arg] for arg in args]

    def evaluate(self, snapshot: 'SnapshotLike') -> Evidence:
        raise NotImplementedError

    def after_step(self, evidence: Evidence, normal: bool) -> None:
        """総合と状態遷移の後に呼ばれる。normal は、その周期の状態が NORMAL か (正常時の基準の学習などに使う)。"""

    def reset(self) -> None:
        """内部の状態を捨てる (地図の切り替え、外からの再初期化など)。"""


@dataclass
class InstanceInfo:
    """総合が使う、判定のインスタンスの情報。"""
    name: str
    subjects: List[str]
    sources: List[str]
    weight: float = 1.0
    allowance: float = 1.0
    decay: float = 1.0
    max_score: float = 5.0
    options: Dict[str, Any] = field(default_factory=dict)


@dataclass
class FusionResult:
    """総合の結果。

    level       : 総合の強さ (方式ごとの単位)
    suspect     : 疑いの段階に達した
    confirmed   : 異常を確認した
    cleared     : 異常がなくなった (疑い・異常から正常に戻ってよい)
    anomaly     : その周期に異常と判定したインスタンスがある
    blame       : 疑わしい独立の情報源の名前。決めきれなければ 'unknown'、異常がなければ ''
    contaminated: 汚れていると見なす、融合の結果の情報源
    accumulated : インスタンスごとの累積
    reason      : 最も寄与したインスタンスの名前
    """
    level: float
    suspect: bool
    confirmed: bool
    cleared: bool
    anomaly: bool = False
    blame: str = ''
    contaminated: List[str] = field(default_factory=list)
    accumulated: Dict[str, float] = field(default_factory=dict)
    reason: str = ''


class FusionStrategy(Plugin):
    """判定のインスタンスの Evidence を総合する。"""

    def __init__(
        self, name: str, params: Mapping[str, Any], instances: Sequence[InstanceInfo],
        independent: Sequence[str], derived: Mapping[str, Sequence[str]],
    ) -> None:
        super().__init__(name, params)
        self.instances = {info.name: info for info in instances}
        self.independent = list(independent)
        self.derived = {k: list(v) for k, v in derived.items()}

    def update(self, evidences: Mapping[str, Evidence], active: bool) -> FusionResult:
        """evidences はインスタンスの名前ごとの結果。active が False の周期 (猶予) は累積を進めない。"""
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError


class SnapshotLike:
    """Detector が使う Snapshot の窓口 (core/snapshot.py の Snapshot が実装する)。"""
    stamp: float
    extras: Dict[str, Any]

    def pose(self, name: str, max_age: Optional[float] = None) -> Optional[PoseSample]:
        raise NotImplementedError

    def pose_history(self, name: str) -> Any:
        raise NotImplementedError

    def pose_aligned(
        self, name: str, max_age: Optional[float] = None, stamp: Optional[float] = None,
    ) -> Optional[Tuple[PoseSample, float]]:
        raise NotImplementedError

    def motion_at(self, name: str, stamp: float) -> Optional[Pose2]:
        raise NotImplementedError

    def reference(self, name: str) -> Any:
        raise NotImplementedError

    def cached(self, key: Any, compute: Any) -> Any:
        raise NotImplementedError
