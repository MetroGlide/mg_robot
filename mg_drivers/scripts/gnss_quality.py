"""NavPVT から GNSS の測位の品質を判定する。ROS に依存しない。

測位の品質 (RTK が Fixed か) は、自己位置推定の初期化や地図への採用の前提になる。
ublox_gps_node の標準の診断 (`ublox_gps_node: fix`) は、RTK の Fixed と Float を区別しないので、
搬送波位相の状態 (flags の bit 6-7) から、ここで判定する。
"""
from typing import Tuple

FLAGS_GNSS_FIX_OK = 1
CARRIER_PHASE_SHIFT = 6
CARRIER_PHASE_MASK = 0b11
CARRIER_PHASE_FLOAT = 1
CARRIER_PHASE_FIXED = 2

# fix_type → (表示名, レベル)
FIX_TYPES = {
    0: ('No Fix', 'error'),
    1: ('DR only', 'warn'),
    2: ('2D Fix', 'warn'),
    3: ('3D Fix', 'warn'),
    4: ('GNSS+DR', 'warn'),
    5: ('Time only', 'error'),
}


def carrier_phase(flags: int) -> int:
    return (flags >> CARRIER_PHASE_SHIFT) & CARRIER_PHASE_MASK


def classify(fix_type: int, flags: int) -> Tuple[str, str]:
    """(表示名, レベル 'ok' / 'warn' / 'error') を返す。

    - RTK Fixed は ok、RTK Float は warn
    - RTK でない測位は warn (fix_type が No Fix・Time only なら error)。
      DOP や精度の条件を満たしていない (FIX_OK なし) 測位は、いずれにしても ok にしない
    """
    carrier = carrier_phase(flags)
    if carrier == CARRIER_PHASE_FIXED:
        return 'RTK Fixed', 'ok'
    if carrier == CARRIER_PHASE_FLOAT:
        return 'RTK Float', 'warn'
    return FIX_TYPES.get(fix_type, (f'fix {fix_type}', 'warn'))
