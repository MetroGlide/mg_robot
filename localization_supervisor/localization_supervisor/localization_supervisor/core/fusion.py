"""判定の総合 (標準の FusionStrategy) と、疑わしい情報源の切り分け。ROS に依存しない。

  cusum       : インスタンスごとの漏れのある累積 S_i = max(0, decay_i * S_i + weight_i * (score_i - allowance_i))。
                総合の強さは S = Σ S_i。大きく飛ぶ場合 (1 回の大きな score) も、少しずつずれる場合
                (しきい値を少し超える score の積み重ね) も、同じ仕組みで扱う。
                異常がなくなった (cleared) のは、その周期に判定できたインスタンスがあり、どれも異常でないとき
                (累積が減りきるのを待たない。状態遷移が clear_ticks 回続くのを確かめる)
  consecutive : インスタンスごとに、続けて異常だった回数を数える (従来の監視と同じ振る舞い)

切り分け (両方の方式で共通):
  - 独立の情報源だけを疑う。インスタンスの累積の、疑いが始まってからの最大値を、そのインスタンスが疑いを
    かける情報源 (subjects) に足す。最も多い情報源と次点の差 (他と共有しない疑い) が blame_margin 以上なら、
    その情報源を blame とする。決めきれなければ unknown。
    2 つの情報源の食い違い (pose_consistency など) は両方に同じだけ足すので、それだけでは決まらない。
    ただし、一方を vouch (正常なら、その情報源が正しいと示す) する判定のインスタンスが、直近で「判定できて正常」
    なら、その一方は疑わず、もう一方だけに足す (両方が vouch されているときは、両方に足す)。
    一度だけの強い証拠 (飛び) も、最大値を使うので、時間が経っても薄まらない
  - 融合の結果 (derived_from を持つ情報源) が関わるインスタンスは blame に数えず、今の累積があれば
    その情報源を contaminated (出力が汚れている) とする。ただし、比べた相手の独立の情報源が blame なら、
    その食い違いは相手のせいとみなし、融合の結果を疑わない
"""
from typing import Collection, Dict, List, Mapping, Set, Tuple

from .interfaces import Evidence, FusionResult, FusionStrategy, InstanceInfo, ParamSpec, Status

VOUCH_HOLD_TICKS = ParamSpec(int, 15, 'vouch する判定が判定できない周期が続いても、正常と示していると見なす周期の数')


def attribute(
    peaks: Mapping[str, float], accumulated: Mapping[str, float], instances: Mapping[str, InstanceInfo],
    independent: List[str], derived: Mapping[str, List[str]], blame_margin: float,
    vouched: Collection[str] = (),
) -> Tuple[str, List[str]]:
    """(blame, contaminated) を決める。peaks は疑いが始まってからの累積の最大値、accumulated は今の累積。
    vouched は、正常と示されている情報源。"""
    if not any(v > 0.0 for v in accumulated.values()):
        return '', []
    suspicion: Dict[str, float] = {}
    for name, value in peaks.items():
        subjects = instances[name].subjects
        if value <= 0.0 or any(s in derived for s in subjects):
            continue
        targets = [s for s in subjects if s in independent]
        targets = [s for s in targets if s not in vouched] or targets
        for subject in targets:
            suspicion[subject] = suspicion.get(subject, 0.0) + value
    ranked = sorted(suspicion.values(), reverse=True)
    blame = 'unknown'
    if ranked and ranked[0] - (ranked[1] if len(ranked) > 1 else 0.0) >= blame_margin:
        blame = max(suspicion, key=suspicion.get)
    # 融合の結果と独立の情報源を比べた判定は、相手が blame なら、その食い違いで融合の結果を疑わない
    contaminated: List[str] = []
    for name, value in accumulated.items():
        subjects = instances[name].subjects
        if value <= 0.0 or blame in subjects:
            continue
        contaminated.extend(s for s in subjects if s in derived and s not in contaminated)
    return blame, contaminated


class _Peaks:
    """インスタンスごとの累積の、疑いが始まってからの最大値。"""

    def __init__(self, names) -> None:
        self.values: Dict[str, float] = {name: 0.0 for name in names}

    def update(self, accumulated: Mapping[str, float]) -> Dict[str, float]:
        for name, value in accumulated.items():
            self.values[name] = max(self.values[name], value)
        return dict(self.values)


class _Vouch:
    """vouch を持つ判定のインスタンスが、正常と示している情報源。

    直近に判定できたときの結果が正常なら示している。判定できない周期が hold_ticks を超えるか、異常になれば外れる
    (スキャンの照合のように、数周期に 1 回しか判定できないものがあるため、すぐには外さない)。
    """

    def __init__(self, instances: Mapping[str, InstanceInfo], hold_ticks: int) -> None:
        self._instances = instances
        self._hold = hold_ticks
        # 正常と示しているインスタンスの名前 -> 最後に正常だった周期からの周期の数
        self._age: Dict[str, int] = {}

    def update(self, evidences: Mapping[str, Evidence], active: bool) -> Set[str]:
        if active:
            for name, info in self._instances.items():
                if not info.vouches:
                    continue
                evidence = evidences.get(name)
                if evidence is None or evidence.status == Status.UNKNOWN:
                    if name in self._age:
                        self._age[name] += 1
                elif evidence.status == Status.NORMAL:
                    self._age[name] = 0
                else:
                    self._age.pop(name, None)
        return {source for name, age in self._age.items() if age <= self._hold
                for source in self._instances[name].vouches}


def _reason(accumulated: Mapping[str, float]) -> str:
    positive = [(v, k) for k, v in accumulated.items() if v > 0.0]
    return max(positive)[1] if positive else ''


class CusumFusion(FusionStrategy):
    PARAMS = {
        'suspect_level': ParamSpec(float, 1.0, '総合の強さがこれ以上で疑い'),
        'confirm_level': ParamSpec(float, 5.0, '総合の強さがこれ以上で異常を確認'),
        'blame_margin': ParamSpec(float, 1.0, '他と共有しない疑いがこれ以上の情報源を blame とする'),
        'vouch_hold_ticks': VOUCH_HOLD_TICKS,
    }

    def __init__(self, name, params, instances, independent, derived) -> None:
        super().__init__(name, params, instances, independent, derived)
        self.reset()

    def reset(self) -> None:
        self._acc: Dict[str, float] = {name: 0.0 for name in self.instances}
        self._peaks = _Peaks(self.instances)
        self._vouch = _Vouch(self.instances, self.params.get('vouch_hold_ticks', VOUCH_HOLD_TICKS.default))

    def update(self, evidences: Mapping[str, Evidence], active: bool) -> FusionResult:
        p = self.params
        known = anomaly = False
        if active:
            for name, evidence in evidences.items():
                info = self.instances.get(name)
                if info is None or evidence.status == Status.UNKNOWN:
                    continue
                # 重みが 0 のインスタンスは、記録のためだけに判定する (総合に効かない)
                if info.weight <= 0.0:
                    self._acc[name] = 0.0
                    continue
                known = True
                anomaly = anomaly or evidence.status == Status.ANOMALY
                score = min(evidence.score, info.max_score)
                self._acc[name] = max(0.0, info.decay * self._acc[name] + info.weight * (score - info.allowance))
        level = sum(self._acc.values())
        if level <= 0.0:
            self._peaks = _Peaks(self.instances)
        blame, contaminated = attribute(
            self._peaks.update(self._acc), self._acc, self.instances, self.independent, self.derived,
            p['blame_margin'], self._vouch.update(evidences, active))
        return FusionResult(
            level=level, suspect=level >= p['suspect_level'], confirmed=level >= p['confirm_level'],
            cleared=known and not anomaly, anomaly=anomaly, blame=blame, contaminated=contaminated,
            accumulated=dict(self._acc), reason=_reason(self._acc))


class ConsecutiveFusion(FusionStrategy):
    """インスタンスごとの、続けて異常だった回数で判断する。

    インスタンスの options:
      suspect_after     : この回数続けば疑い (既定 1)
      confirm_after     : この回数続けば異常を確認 (0 なら、単独では確認しない。既定 3)
      corroborate_after : このインスタンスが疑いの間に異常になっていて、他のインスタンスがこの回数続けば
                          異常を確認 (0 なら使わない。既定 0)
    異常がなくなった (cleared) のは、その周期に判定できたインスタンスがあり、どれも異常でないとき。
    """
    PARAMS = {
        'blame_margin': ParamSpec(float, 1.0, '他と共有しない疑い (回数) がこれ以上の情報源を blame とする'),
        'vouch_hold_ticks': VOUCH_HOLD_TICKS,
    }

    def __init__(self, name, params, instances, independent, derived) -> None:
        super().__init__(name, params, instances, independent, derived)
        self.reset()

    def reset(self) -> None:
        self._runs: Dict[str, int] = {name: 0 for name in self.instances}
        self._fired: Dict[str, bool] = {name: False for name in self.instances}
        self._suspect = False
        self._peaks = _Peaks(self.instances)
        self._vouch = _Vouch(self.instances, self.params.get('vouch_hold_ticks', VOUCH_HOLD_TICKS.default))

    def _option(self, name: str, key: str, default: int) -> int:
        return int(self.instances[name].options.get(key, default))

    def update(self, evidences: Mapping[str, Evidence], active: bool) -> FusionResult:
        known = anomaly = False
        if active:
            for name, evidence in evidences.items():
                if name not in self.instances or evidence.status == Status.UNKNOWN:
                    continue
                known = True
                if evidence.status == Status.ANOMALY:
                    anomaly = True
                    self._runs[name] += 1
                    self._fired[name] = True
                else:
                    self._runs[name] = 0
        if not self._suspect:
            # 疑いの前は、この周期に異常になったものだけを覚える
            self._fired = {name: active and evidences.get(name, Evidence.unknown()).status == Status.ANOMALY
                           for name in self.instances}
        suspect = confirmed = False
        level = 0.0
        for name in self.instances:
            run = self._runs[name]
            suspect_after = self._option(name, 'suspect_after', 1)
            confirm_after = self._option(name, 'confirm_after', 3)
            corroborate_after = self._option(name, 'corroborate_after', 0)
            if run >= suspect_after or (corroborate_after and self._fired[name]):
                suspect = True
            if confirm_after and run >= confirm_after:
                confirmed = True
            if corroborate_after and self._fired[name]:
                others = [self._runs[o] for o in self.instances if o != name]
                if others and max(others) >= corroborate_after:
                    confirmed = True
            if confirm_after:
                level = max(level, run / confirm_after)
        suspect = suspect or confirmed
        self._suspect = suspect
        accumulated = {name: float(run) for name, run in self._runs.items()}
        for name, fired in self._fired.items():
            if fired and accumulated[name] == 0.0:
                accumulated[name] = 1.0
        if not any(accumulated.values()):
            self._peaks = _Peaks(self.instances)
        blame, contaminated = attribute(
            self._peaks.update(accumulated), accumulated, self.instances, self.independent, self.derived,
            self.params['blame_margin'], self._vouch.update(evidences, active))
        return FusionResult(
            level=level, suspect=suspect, confirmed=confirmed, cleared=known and not anomaly,
            anomaly=anomaly, blame=blame, contaminated=contaminated, accumulated=accumulated,
            reason=_reason(accumulated))


STANDARD_FUSIONS = {
    'cusum': CusumFusion,
    'consecutive': ConsecutiveFusion,
}
