"""走り方: 自己位置の状況と程度から、対処 (通常 / 減速) と速度の上限を決める (ROS に依存しない)。

  状況 (Situation): 監督と門の状態から決める。1 つ軽い状況へ戻るのは、地図との照合で確かめたときだけ
  程度 (Degree)   : 起こりうる誤差 ÷ 余裕 (footprint から計画用地図の通れない所までの距離) の区分
  対処            : 状況 × 程度の対応表。正常の状況では程度を使わない (正常時に減速しない)

止める対処は持たない。止めるのは門と監督の、時間と距離に上限のある仕組みだけにする (止まったままにしない)。
"""
import enum
from dataclasses import dataclass, field
from typing import Dict, Optional


class Situation(enum.Enum):
    UNINITIALIZED = 'uninitialized'   # 確かめた姿勢がない (起動の直後、地図の切り替えの後)
    NORMAL = 'normal'
    SUSPECT = 'suspect'               # 異常の疑い (監督の疑い、門が閉じた直後)
    UNDECIDED = 'undecided'           # 照合で決まらない (門が閉じたまま、監督が復旧できていない)
    UNOBSERVED = 'unobserved'         # 地図との照合で確かめないまま、一定の距離を走った
    RECOVERING = 'recovering'         # 監督が切り離して復旧している


class Degree(enum.IntEnum):
    NONE = 0     # 程度を使わない (正常)
    LIGHT = 1
    MEDIUM = 2
    HEAVY = 3


class Action(enum.Enum):
    NORMAL = 'normal'
    SLOW = 'slow'


# 状況ごとの起こりうる誤差の下限の既定 [m] (例。値は設定が正)
DEFAULT_FLOORS = {'uninitialized': 2.0, 'suspect': 1.0, 'undecided': 1.5, 'unobserved': 1.0, 'recovering': 1.5}
DEFAULT_TABLE = {
    'uninitialized': {'light': 'normal', 'medium': 'slow', 'heavy': 'slow'},
    'suspect': {'light': 'normal', 'medium': 'slow', 'heavy': 'slow'},
    'undecided': {'light': 'slow', 'medium': 'slow', 'heavy': 'slow'},
    'unobserved': {'light': 'slow', 'medium': 'slow', 'heavy': 'slow'},
    'recovering': {'light': 'slow', 'medium': 'slow', 'heavy': 'slow'},
}


@dataclass
class ConductConfig:
    # 最後に確かめてから走った道のり 1 m あたりに増やす、起こりうる誤差 [m/m]
    error_per_m: float = 0.05
    # 状況ごとの起こりうる誤差の下限 [m]
    floors: Dict[str, float] = field(default_factory=lambda: dict(DEFAULT_FLOORS))
    # 確かめないまま、これだけ走ったら「見えない」 [m]
    unobserved_after_m: float = 25.0
    # 門が閉じてから、これだけ決まらなければ「決めきれない」 [s]
    undecided_after_sec: float = 3.0
    # 程度の区分 (起こりうる誤差 ÷ 余裕)。medium_ratio 以上で中、heavy_ratio 以上で重い
    medium_ratio: float = 0.5
    heavy_ratio: float = 1.0
    # 程度・状況を軽くするのは、軽い側がこの時間続いてから (上げるのはすぐ)
    lower_hold_sec: float = 3.0
    # 減速の速度の上限 [m/s]
    slow_speed_mps: float = 0.3
    # 状況 × 程度 → 対処 ('normal' | 'slow')
    table: Dict[str, Dict[str, str]] = field(default_factory=lambda: {k: dict(v) for k, v in DEFAULT_TABLE.items()})


@dataclass
class ConductInputs:
    """周期ごとの入力。"""
    stamp: float
    situation: Situation
    # 最後に確かめてから走った道のり [m] (確かめた姿勢がなければ None)
    since_confirmed_m: Optional[float]
    # footprint から計画用地図の通れない所までの距離 [m] (分からなければ None)
    clearance_m: Optional[float]
    # 地図との照合で確かめた周期か (疑いから戻す条件)
    confirmed: bool = False
    # 新しい判定の結果に異常があるか (None はこの周期に新しい判定がない)。確かめた後に異常があれば、確かめを取り消す
    contradicted: Optional[bool] = None


@dataclass
class Conduct:
    situation: Situation
    degree: Degree
    possible_error_m: float
    clearance_m: Optional[float]
    action: Action
    # 速度の上限 [m/s] (0 は上限なし)
    speed_limit_mps: float


class ConductPlanner:
    """状況を確かめるまで保ち、程度を決め、対応表で対処を選ぶ。"""

    def __init__(self, config: ConductConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._situation = Situation.UNINITIALIZED
        self._degree = Degree.NONE
        self._lighter_since: Optional[float] = None
        # 疑いの後に確かめ、その後の判定に異常がない
        self._confirmed = False

    @property
    def situation(self) -> Situation:
        return self._situation

    def update(self, inputs: ConductInputs) -> Conduct:
        cfg = self.config
        situation = self._next_situation(inputs)
        if situation == Situation.NORMAL:
            degree, error = Degree.NONE, 0.0
        else:
            error = max(cfg.floors.get(situation.value, 0.0), cfg.error_per_m * (inputs.since_confirmed_m or 0.0))
            degree = self._next_degree(inputs.stamp, self._degree_of(error, inputs.clearance_m))
        self._degree = degree
        action = Action.NORMAL
        if situation != Situation.NORMAL:
            action = Action(cfg.table.get(situation.value, {}).get(degree.name.lower(), Action.NORMAL.value))
        speed = cfg.slow_speed_mps if action == Action.SLOW else 0.0
        return Conduct(situation, degree, error, inputs.clearance_m, action, speed)

    def _next_situation(self, inputs: ConductInputs) -> Situation:
        """重い状況へはすぐに移る。疑い・見えないから正常へは、確かめ、その後の判定に異常がないときだけ戻る。"""
        wanted = inputs.situation
        current = self._situation
        if wanted != Situation.NORMAL and wanted != current:
            self._confirmed = False
        if inputs.confirmed:
            self._confirmed = True
        if inputs.contradicted:
            self._confirmed = False
        if wanted == Situation.NORMAL and current in (Situation.SUSPECT, Situation.UNOBSERVED):
            if not self._confirmed or inputs.contradicted is None:
                return current
        if wanted == Situation.NORMAL:
            self._confirmed = False
        self._situation = wanted
        return wanted

    def _degree_of(self, error: float, clearance: Optional[float]) -> Degree:
        cfg = self.config
        if clearance is None:
            # 余裕が分からなければ、通れない所が近いものとして扱う
            return Degree.HEAVY
        ratio = error / max(clearance, 1e-3)
        if ratio >= cfg.heavy_ratio:
            return Degree.HEAVY
        if ratio >= cfg.medium_ratio:
            return Degree.MEDIUM
        return Degree.LIGHT

    def _next_degree(self, stamp: float, wanted: Degree) -> Degree:
        """上げるのはすぐ、下げるのは軽い側が lower_hold_sec 続いてから。"""
        current = self._degree if self._degree != Degree.NONE else wanted
        if wanted >= current:
            self._lighter_since = None
            return wanted
        if self._lighter_since is None:
            self._lighter_since = stamp
        if stamp - self._lighter_since >= self.config.lower_hold_sec:
            self._lighter_since = None
            return wanted
        return current
