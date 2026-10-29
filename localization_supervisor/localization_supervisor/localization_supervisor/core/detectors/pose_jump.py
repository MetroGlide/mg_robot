"""姿勢の情報源が、動きの情報源の示す動きから飛んだか、ずれていったか。"""
from collections import deque
from typing import Deque, Optional, Tuple

from ..geometry import Pose2, implied_map_odom, pose_jump
from ..interfaces import Detector, Evidence, ParamSpec, SnapshotLike, SourceKind


class PoseJump(Detector):
    """新しい姿勢の標本ごとに、姿勢と動きが示す map->odom を求め、比べる標本からの変化を見る。

    動きが正しければ map->odom はゆっくりしか変わらないので、大きく変われば姿勢が飛んだことになる。
    window_sec が 0 なら前の標本と比べる (飛び)。正なら window_sec 以上前の最も新しい標本と比べ、
    1 回ごとは小さくても、続けて同じ向きにずれていくもの (旋回中に向きが回りすぎるなど) を見る。
    新しい標本がない周期は判定できない。比べる標本との間が window_sec + max_gap_sec を超えたら比べない。
    """
    SOURCES = {'pose': SourceKind.POSE, 'motion': SourceKind.MOTION}
    SUBJECTS = ('pose',)
    PARAMS = {
        'threshold_m': ParamSpec(float, 0.6),
        'threshold_rad': ParamSpec(float, 0.35),
        'max_gap_sec': ParamSpec(float, 5.0),
        'window_sec': ParamSpec(float, 0.0),
    }

    def __init__(self, name, params, sources) -> None:
        super().__init__(name, params, sources)
        self.reset()

    def reset(self) -> None:
        self._last_stamp: Optional[float] = None
        self._history: Deque[Tuple[float, Pose2]] = deque()

    def _reference(self, stamp: float) -> Optional[Tuple[float, Pose2]]:
        """stamp と比べる過去の標本。window_sec 以上前の標本のうち最も新しいもの (0 なら直前の標本)。"""
        window = self.params['window_sec']
        while len(self._history) >= 2 and self._history[1][0] <= stamp - window:
            self._history.popleft()
        if self._history and self._history[0][0] <= stamp - window:
            return self._history[0]
        return None

    def evaluate(self, snapshot: SnapshotLike) -> Evidence:
        p = self.params
        history = snapshot.pose_history(self.sources['pose'])
        new = [s for s in history.samples() if self._last_stamp is None or s.stamp > self._last_stamp]
        if not new:
            return Evidence.unknown()
        self._last_stamp = new[-1].stamp
        worst: Optional[Evidence] = None
        for sample in new:
            odom_base = snapshot.motion_at(self.sources['motion'], sample.stamp)
            if odom_base is None:
                continue
            map_odom = implied_map_odom(sample.pose, odom_base)
            reference = self._reference(sample.stamp)
            self._history.append((sample.stamp, map_odom))
            if reference is None or sample.stamp - reference[0] > p['window_sec'] + p['max_gap_sec']:
                continue
            dpos, dyaw = pose_jump(reference[1], map_odom, odom_base)
            score = max(dpos / p['threshold_m'], dyaw / p['threshold_rad'])
            if worst is None or score > worst.score:
                worst = Evidence.from_score(score, jump_m=dpos, jump_rad=dyaw)
        return worst if worst is not None else Evidence.unknown()
