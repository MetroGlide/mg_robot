"""姿勢の情報源が、動きの情報源の示す動きから飛んだか。"""
from typing import Optional

from ..geometry import Pose2, implied_map_odom, pose_jump
from ..interfaces import Detector, Evidence, ParamSpec, SnapshotLike, SourceKind


class PoseJump(Detector):
    """新しい姿勢の標本ごとに、姿勢と動きが示す map->odom を求め、前の標本からの変化を見る。

    動きが正しければ map->odom はゆっくりしか変わらないので、大きく変われば姿勢が飛んだことになる。
    新しい標本がない周期は判定できない。前の標本から max_gap_sec 以上空いたら比べない。
    """
    SOURCES = {'pose': SourceKind.POSE, 'motion': SourceKind.MOTION}
    SUBJECTS = ('pose',)
    PARAMS = {
        'threshold_m': ParamSpec(float, 0.6),
        'threshold_rad': ParamSpec(float, 0.35),
        'max_gap_sec': ParamSpec(float, 5.0),
    }

    def __init__(self, name, params, sources) -> None:
        super().__init__(name, params, sources)
        self.reset()

    def reset(self) -> None:
        self._last_stamp: Optional[float] = None
        self._prev: Optional[Pose2] = None
        self._prev_stamp: Optional[float] = None

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
            previous, previous_stamp = self._prev, self._prev_stamp
            self._prev, self._prev_stamp = map_odom, sample.stamp
            if previous is None or sample.stamp - previous_stamp > p['max_gap_sec']:
                continue
            dpos, dyaw = pose_jump(previous, map_odom, odom_base)
            score = max(dpos / p['threshold_m'], dyaw / p['threshold_rad'])
            if worst is None or score > worst.score:
                worst = Evidence.from_score(score, jump_m=dpos, jump_rad=dyaw)
        return worst if worst is not None else Evidence.unknown()
