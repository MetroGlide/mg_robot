"""オフライン評価の集計: 故障の区間 (正解) と、周期ごとの結果から、検知率・誤検知・検知までの時間を出す。

ROS に依存しない。時刻は評価の開始からの経過秒。

  検知 (系全体)   : 故障の開始から、区間の終わりまでに、状態が NORMAL 以外 (疑い) / 確認
                    (ISOLATED・RECOVERING・DEGRADED) になったか。なったまでの時間と、確認したときの blame
  誤検知 (系全体) : どの故障の区間 (と、その後の settle_sec) にも入らない時刻に、確認に移った回数
  判定のインスタンスごと : 故障ごとに、区間の中で初めて異常と判定するまでの時間と、
                    区間の外で判定できた周期のうち、異常と判定した割合 (誤検知率)

検知の対象は、姿勢をずらす故障 (pose_offset・pose_drift) だけ。標本が欠けるだけの故障 (source_drop・scan_*) は、
記録済みの推定器の出力を変えないので、その区間も「異常がない」区間として誤検知を数える。
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from .faults import Fault
from .interfaces import Status
from .state_machine import State

CONFIRMED_STATES = (State.ISOLATED, State.RECOVERING, State.DEGRADED)
ERROR_FAULTS = ('pose_offset', 'pose_drift')


@dataclass
class TickRecord:
    t: float
    state: State
    level: float
    blame: str
    statuses: Dict[str, Status]
    scores: Dict[str, float] = field(default_factory=dict)


@dataclass
class ReinitRecord:
    """監督が再初期化を指示した 1 回。error_m は、候補と真の姿勢 (故障を注入する前の記録) の距離 (分からなければ None)。"""
    t: float
    origin: str
    error_m: Optional[float] = None
    error_yaw_rad: Optional[float] = None
    total: Optional[float] = None


@dataclass
class FaultOutcome:
    type: str
    source: str
    start: float
    end: float
    suspect_after: Optional[float]
    confirm_after: Optional[float]
    blame: str
    instance_after: Dict[str, Optional[float]]
    expected: bool = True   # 検知の対象か (姿勢をずらす故障)
    # 区間の中で最初に再初期化を指示するまで、その候補の誤差、そのあと NORMAL に戻るまで (開始からの秒)
    reinit_after: Optional[float] = None
    reinit_error_m: Optional[float] = None
    recovered_after: Optional[float] = None


@dataclass
class InstanceStats:
    detected: int
    faults: int
    false_positive_ticks: int
    clean_known_ticks: int

    @property
    def false_positive_rate(self) -> Optional[float]:
        return None if self.clean_known_ticks == 0 else self.false_positive_ticks / self.clean_known_ticks


@dataclass
class Report:
    faults: List[FaultOutcome]
    instances: Dict[str, InstanceStats]
    false_alarms: int
    clean_ticks: int
    clean_not_normal_ticks: int
    reinits: List[ReinitRecord] = field(default_factory=list)
    # 再初期化の指示のうち、候補の誤差が ok_threshold_m を超えたもの (誤った復旧)
    wrong_reinits: int = 0
    # 故障の区間の外での再初期化の指示
    spurious_reinits: int = 0
    # 候補を選べず見送った回数 (理由ごと)
    skipped: Dict[str, int] = field(default_factory=dict)

    @property
    def expected(self) -> List[FaultOutcome]:
        return [f for f in self.faults if f.expected]

    @property
    def detected(self) -> int:
        return sum(1 for f in self.expected if f.confirm_after is not None)


def _windows(faults: Sequence[Fault], run_end: float, settle_sec: float):
    return [(f.start, f.label_end(run_end) + settle_sec) for f in faults if f.type in ERROR_FAULTS]


def evaluate(
    records: Sequence[TickRecord], faults: Sequence[Fault], settle_sec: float = 30.0,
    reinits: Sequence[ReinitRecord] = (), skipped: Optional[Dict[str, int]] = None, ok_threshold_m: float = 1.0,
) -> Report:
    if not records:
        return Report([], {}, 0, 0, 0)
    run_end = records[-1].t
    windows = _windows(faults, run_end, settle_sec)
    names = sorted({name for r in records for name in r.statuses})

    outcomes: List[FaultOutcome] = []
    for fault in faults:
        end = fault.label_end(run_end)
        inside = [r for r in records if fault.start <= r.t <= end]
        suspect = next((r for r in inside if r.state != State.NORMAL), None)
        confirm = next((r for r in inside if r.state in CONFIRMED_STATES), None)
        first = {}
        for name in names:
            hit = next((r for r in inside if r.statuses.get(name) == Status.ANOMALY), None)
            first[name] = None if hit is None else hit.t - fault.start
        reinit = next((x for x in reinits if fault.start <= x.t <= end + settle_sec), None)
        recovered = None
        if reinit is not None:
            back = next((r for r in records if r.t > reinit.t and r.state == State.NORMAL), None)
            recovered = None if back is None else back.t - fault.start
        outcomes.append(FaultOutcome(
            type=fault.type, source=fault.source, start=fault.start, end=end,
            suspect_after=None if suspect is None else suspect.t - fault.start,
            confirm_after=None if confirm is None else confirm.t - fault.start,
            blame='' if confirm is None else confirm.blame, instance_after=first,
            expected=fault.type in ERROR_FAULTS,
            reinit_after=None if reinit is None else reinit.t - fault.start,
            reinit_error_m=None if reinit is None else reinit.error_m, recovered_after=recovered))

    def clean(t: float) -> bool:
        return not any(lo <= t <= hi for lo, hi in windows)

    clean_records = [r for r in records if clean(r.t)]
    false_alarms = 0
    previous = State.NORMAL
    for record in records:
        if record.state in CONFIRMED_STATES and previous not in CONFIRMED_STATES and clean(record.t):
            false_alarms += 1
        previous = record.state

    instances = {}
    for name in names:
        known = [r for r in clean_records if r.statuses.get(name, Status.UNKNOWN) != Status.UNKNOWN]
        expected = [o for o in outcomes if o.expected]
        instances[name] = InstanceStats(
            detected=sum(1 for o in expected if o.instance_after[name] is not None), faults=len(expected),
            false_positive_ticks=sum(1 for r in known if r.statuses[name] == Status.ANOMALY),
            clean_known_ticks=len(known))
    return Report(
        faults=outcomes, instances=instances, false_alarms=false_alarms, clean_ticks=len(clean_records),
        clean_not_normal_ticks=sum(1 for r in clean_records if r.state != State.NORMAL),
        reinits=list(reinits),
        wrong_reinits=sum(1 for x in reinits if x.error_m is not None and x.error_m > ok_threshold_m),
        spurious_reinits=sum(1 for x in reinits if clean(x.t)), skipped=dict(skipped or {}))


def _seconds(value: Optional[float]) -> str:
    return '-' if value is None else f'{value:.1f}'


def format_report(report: Report, title: str = '') -> str:
    """集計を Markdown の表にする。"""
    lines = [f'# オフライン評価{": " + title if title else ""}', '']
    lines.append(f'- 検知の対象の故障: {len(report.expected)} 件、確認できたもの: {report.detected} 件'
                 f' (標本が欠けるだけの故障: {len(report.faults) - len(report.expected)} 件)')
    lines.append(f'- 故障の区間の外で確認に移った回数 (誤検知): {report.false_alarms}')
    if report.clean_ticks:
        lines.append(f'- 故障の区間の外で NORMAL 以外だった周期の割合: '
                     f'{report.clean_not_normal_ticks / report.clean_ticks:.3f} ({report.clean_ticks} 周期中)')
    if report.reinits or report.skipped:
        skipped = ', '.join(f'{reason} {count}' for reason, count in sorted(report.skipped.items())) or 'なし'
        lines.append(f'- 再初期化の指示: {len(report.reinits)} 回 (候補の誤差が大きかったもの {report.wrong_reinits} 回、'
                     f'故障の区間の外 {report.spurious_reinits} 回)。見送り: {skipped}')
    if report.faults:
        names = sorted(report.instances)
        recovery = bool(report.reinits)
        lines += ['', '## 故障ごと (開始からの秒。- は区間の中で検知せず)', '',
                  '| 故障 | 情報源 | 検知の対象 | 開始 [s] | 疑い | 確認 | blame | '
                  + ('再初期化 | 候補の誤差 [m] | 復旧 | ' if recovery else '') + ' | '.join(names) + ' |',
                  '|---|---|---|---|---|---|---|' + ('---|---|---|' if recovery else '') + '---|' * len(names)]
        for f in report.faults:
            extra = (f'{_seconds(f.reinit_after)} | {_seconds(f.reinit_error_m)} | {_seconds(f.recovered_after)} | '
                     if recovery else '')
            lines.append(
                f'| {f.type} | {f.source} | {"○" if f.expected else "-"} | {f.start:.1f} | {_seconds(f.suspect_after)} | '
                f'{_seconds(f.confirm_after)} | {f.blame or "-"} | {extra}'
                + ' | '.join(_seconds(f.instance_after[n]) for n in names) + ' |')
    lines += ['', '## 判定のインスタンスごと', '',
              '| インスタンス | 検知した故障 | 誤検知率 (区間の外で異常と判定した割合) | 区間の外で判定できた周期 |',
              '|---|---|---|---|']
    for name, stats in sorted(report.instances.items()):
        rate = stats.false_positive_rate
        lines.append(f'| {name} | {stats.detected}/{stats.faults} | {"-" if rate is None else f"{rate:.3f}"} | '
                     f'{stats.clean_known_ticks} |')
    return '\n'.join(lines) + '\n'
