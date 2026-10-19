"""復旧の計画: 候補を作り、評価し、再初期化してよいものを選ぶ (ROS に依存しない)。

  1. 候補を作るもの (generators) が候補を作る (設定の順)
  2. 評価するもの (scorers) が候補を 0〜1 で評価する。veto_below を下回った候補は捨てる
  3. 評価の重み付き平均 (total) で順位を付ける。次の場合は再初期化しない
       - 最良の total が min_score 未満 (low_score)
       - 最良と、それから separation_m 以上離れた次点の差が ambiguity_margin 未満 (ambiguous)
     評価するものがない (または、どれも評価できなかった) ときは、最初に作られた候補を選ぶ (unscored)
"""
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from ..interfaces import Candidate, CandidateGenerator, CandidateScorer, Matcher, RecoveryContext


@dataclass
class GeneratorConfig:
    name: str
    type: str
    cls: type
    sources: Dict[str, str]
    params: Dict[str, Any]
    enabled: bool = True


@dataclass
class ScorerConfig:
    name: str
    type: str
    cls: type
    sources: Dict[str, str]
    params: Dict[str, Any]
    weight: float = 1.0
    # この値を下回った候補は捨てる (None は捨てない)
    veto_below: Optional[float] = None
    enabled: bool = True


@dataclass
class SelectionConfig:
    ambiguity_margin: float = 0.1
    min_score: float = 0.0
    separation_m: float = 1.0
    # 候補を選べなかったとき、次に計画するまでの間隔 [s]
    retry_sec: float = 2.0


@dataclass
class RecoveryPlan:
    candidates: List[Candidate]
    chosen: Optional[Candidate]
    # selected / unscored / no_candidates / all_vetoed / low_score / ambiguous
    reason: str
    elapsed_sec: float = 0.0
    margin: float = 0.0
    # 候補を作るものごとの、直近の結果の理由 (作れなければその理由)
    generators: Dict[str, str] = field(default_factory=dict)


class RecoveryPlanner:
    def __init__(
        self, generators: List[CandidateGenerator], scorers: List[CandidateScorer],
        scorer_configs: List[ScorerConfig], selection: SelectionConfig, matcher: Optional[Matcher],
    ) -> None:
        self.generators = generators
        self.scorers = scorers
        self.scorer_configs = scorer_configs
        self.selection = selection
        self.matcher = matcher

    def reset(self) -> None:
        for plugin in [*self.generators, *self.scorers]:
            plugin.reset()

    def plan(self, context: RecoveryContext, exclude: str = '') -> RecoveryPlan:
        """exclude は、疑わしい情報源の名前。それを使う候補を作るものと評価するものは、使わない。"""
        started = time.monotonic()
        context.matcher = self.matcher
        candidates: List[Candidate] = []
        notes: Dict[str, str] = {}
        for generator in self.generators:
            if exclude and exclude in generator.sources.values():
                notes[generator.name] = f'excluded:{exclude}'
                continue
            candidates.extend(generator.generate(context))
            notes[generator.name] = generator.status
        if not candidates:
            return RecoveryPlan([], None, 'no_candidates', time.monotonic() - started, generators=notes)
        scorers = [(s, c) for s, c in zip(self.scorers, self.scorer_configs)
                   if not (exclude and exclude in s.sources.values())]
        for candidate in candidates:
            self._score(candidate, context, scorers)
        alive = [c for c in candidates if not c.vetoed_by]
        plan = self._select(candidates, alive)
        plan.elapsed_sec = time.monotonic() - started
        plan.generators = notes
        return plan

    @staticmethod
    def _score(candidate: Candidate, context: RecoveryContext, scorers) -> None:
        weighted = total = 0.0
        for scorer, config in scorers:
            value = scorer.score(candidate, context)
            if value is None:
                continue
            candidate.scores[config.name] = value
            if config.veto_below is not None and value < config.veto_below:
                candidate.vetoed_by = config.name
            weighted += config.weight * value
            total += config.weight
        candidate.total = weighted / total if total > 0.0 else None

    def _select(self, candidates: List[Candidate], alive: List[Candidate]) -> RecoveryPlan:
        if not alive:
            return RecoveryPlan(candidates, None, 'all_vetoed')
        scored = [c for c in alive if c.total is not None]
        if not scored:
            return RecoveryPlan(candidates, alive[0], 'unscored')
        ranked = sorted(scored, key=lambda c: -c.total)
        best = ranked[0]
        if best.total < self.selection.min_score:
            return RecoveryPlan(candidates, None, 'low_score')
        rival = next((c for c in ranked[1:] if math.hypot(
            c.pose[0] - best.pose[0], c.pose[1] - best.pose[1]) >= self.selection.separation_m), None)
        margin = best.total - rival.total if rival is not None else best.total
        if rival is not None and margin < self.selection.ambiguity_margin:
            return RecoveryPlan(candidates, None, 'ambiguous', margin=margin)
        return RecoveryPlan(candidates, best, 'selected', margin=margin)
