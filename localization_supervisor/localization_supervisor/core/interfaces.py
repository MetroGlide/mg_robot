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


@dataclass
class SearchWindow:
    """照合で探す範囲。中心の姿勢の周りを ±range、step 刻みで探す。"""
    range_xy_m: float
    step_xy_m: float
    range_yaw_rad: float = 0.0
    step_yaw_rad: float = 0.05


@dataclass
class MatchPeak:
    """照合で見つけた姿勢の候補。ratio は占有セルの近くにある点の割合、mean_distance はその距離の平均。"""
    pose: Pose2
    ratio: float
    mean_distance: float


class Matcher(Plugin):
    """点群を地図に照合する。"""

    def match(
        self, field: Any, points_base: np.ndarray, center: Pose2, window: SearchWindow, max_peaks: int = 1,
    ) -> List[MatchPeak]:
        """points_base (base の座標の点 (N, 2)) が地図 (距離場 field) に最もよく合う姿勢を、一致の良い順に
        最大 max_peaks 個 (互いに離れたもの) 返す。判断できなければ空。"""
        raise NotImplementedError


@dataclass
class Candidate:
    """復旧の候補。cov は x, y, yaw の 3x3 の共分散。scores はスコアラーごとの評価 (0〜1)。"""
    pose: Pose2
    cov: np.ndarray
    origin: str
    scores: Dict[str, float] = field(default_factory=dict)
    total: Optional[float] = None
    vetoed_by: str = ''
    info: Dict[str, float] = field(default_factory=dict)


@dataclass
class RecoveryContext:
    """復旧の候補を作る・評価するときの入力。

    snapshot     : 今の周期の Snapshot
    anchors      : アンカー (信頼できた時点の姿勢と動き)
    suspect_since: 疑いが始まった時刻 (分からなければ None)
    scans        : 動きとともに残したスキャン (ScanBuffer。設定がなければ None)
    safety       : 安全の信号の設定 (SafetyConfig。なければ None)
    matcher      : 照合
    """
    snapshot: 'SnapshotLike'
    anchors: Any = None
    suspect_since: Optional[float] = None
    scans: Any = None
    safety: Any = None
    matcher: Optional[Matcher] = None


class _SourcedPlugin(Plugin):
    """引数の名前で情報源を受け取るプラグイン (判定と同じ約束)。"""
    SOURCES: Mapping[str, SourceKind] = {}
    # 設定に必要な項目 ('anchors'、'safety')
    REQUIRES: Sequence[str] = ()

    def __init__(self, name: str, params: Mapping[str, Any], sources: Mapping[str, str]) -> None:
        super().__init__(name, params)
        self.sources = dict(sources)

    def reset(self) -> None:
        """内部の状態を捨てる。"""


class CandidateGenerator(_SourcedPlugin):
    """復旧の候補を作る。再初期化が要るあいだ、周期ごとに呼ばれる (作れなければ空を返す)。

    status に、直近の generate の結果の短い理由 (作れたら ok、作れなければその理由) を入れる。
    候補を選べなかったときのイベントに添える。
    """
    status = ''

    def generate(self, context: RecoveryContext) -> List[Candidate]:
        raise NotImplementedError


class CandidateScorer(_SourcedPlugin):
    """候補を評価する。"""

    def score(self, candidate: Candidate, context: RecoveryContext) -> Optional[float]:
        """0〜1 (大きいほど良い)。評価できなければ None。"""
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
