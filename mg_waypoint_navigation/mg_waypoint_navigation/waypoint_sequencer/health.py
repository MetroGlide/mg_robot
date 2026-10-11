"""シーケンサの進捗の診断の内容。ROS に依存しない。"""
from typing import Dict, Optional, Tuple

from mg_waypoint_navigation.waypoint_sequencer.progress_monitor import ProgressStatus


def progress_health(
    progress: Optional[ProgressStatus], stall_warn_sec: float,
) -> Tuple[str, str, Dict[str, str]]:
    """(レベル 'ok' / 'warn'、メッセージ、values)。

    BT はリカバリーを無制限にくり返すので、詰まっても Nav2 は失敗を返さない。
    位置が進まない時間が stall_warn_sec 以上になったら warn にして、詰まりを知らせる。
    """
    if progress is None:
        return 'ok', 'not navigating', {}
    values = {
        'stalled_sec': f'{progress.stalled_sec:.1f}',
        'number_of_recoveries': str(progress.number_of_recoveries),
    }
    if progress.stalled_sec >= stall_warn_sec:
        return 'warn', f'no progress for {progress.stalled_sec:.0f} s', values
    return 'ok', 'navigating', values
