"""MMI (スイッチ・RGB LED・ブザーの操作盤マイコン) の通信と表示の判断。ROS に依存しない。

通信は USB CDC の固定長のバイナリ:

- PC → MMI: 8 バイト `23 CMD DATA1..DATA6`。`0x92` はブザー・RGB LED の制御とハートビートを兼ねる
- MMI → PC: 4 バイト `23 61/62/63 01 00` (SW1/SW2/SW3 の押下。離したときは通知しない)

表示は 2 種類だけ: シーケンサが開始要求を受け付けられる間は LED1 を橙で点滅させてメロディを鳴らし、
それ以外は緑で点灯する。物理スイッチを押すと、MMI が自分でブザーを止め、点滅を点灯に変える。
"""
import enum
from typing import List, Optional

HEADER = 0x23
CMD_CONTROL = 0x92

_SWITCH_CMDS = {0x61: 1, 0x62: 2, 0x63: 3}
_EVENT_SIZE = 4

_LED_SOLID = 0x01
_LED_BLINK = 0x02
_BUZZER_REPEAT = 0x02
_COLOR_GREEN = 0x03
_COLOR_ORANGE = 0x05
_MELODY_WAITING = 0x04


def control_packet(buzzer_control: int = 0, buzzer_data: int = 0,
                   led1_control: int = 0, led1_data: int = 0,
                   led2_control: int = 0, led2_data: int = 0) -> bytes:
    """`0x92` の 8 バイト。control が 0 の機能は変えない。"""
    return bytes([HEADER, CMD_CONTROL, buzzer_control, buzzer_data,
                  led1_control, led1_data, led2_control, led2_data])


# 全機能を変えず、通信が生きていることだけを伝える
HEARTBEAT = control_packet()
LED1_GREEN_SOLID = control_packet(led1_control=_LED_SOLID, led1_data=_COLOR_GREEN)
LED1_ORANGE_BLINK = control_packet(led1_control=_LED_BLINK, led1_data=_COLOR_ORANGE)
# 「再開待ち」のメロディを 3 秒ごとに 3 回 (MMI が自分で止める)
MELODY_WAITING = control_packet(buzzer_control=_BUZZER_REPEAT, buzzer_data=_MELODY_WAITING)


class SwitchEventParser:
    """MMI → PC のバイト列から、スイッチの押下を取り出す。

    読み出しの境界は 4 バイトにそろわないので、溜めてから解析する。
    先頭が `0x23` でない、または既知の通知の形でなければ 1 バイト捨てて同期を取り直す。
    """

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[int]:
        """受信したバイト列を渡し、押されたスイッチの番号 (1-3) を受信順に返す。"""
        self._buffer.extend(data)
        switches = []
        while self._buffer:
            if self._buffer[0] != HEADER:
                del self._buffer[0]
                continue
            if len(self._buffer) < _EVENT_SIZE:
                break
            switch = _SWITCH_CMDS.get(self._buffer[1])
            if switch is not None and self._buffer[2] == 0x01 and self._buffer[3] == 0x00:
                switches.append(switch)
                del self._buffer[:_EVENT_SIZE]
            else:
                del self._buffer[0]
        return switches

    def reset(self) -> None:
        """溜めている途中のバイトを捨てる (ポートを開き直したとき)。"""
        self._buffer.clear()


class Display(enum.Enum):
    WAITING = 'waiting'
    NOT_WAITING = 'not_waiting'


class IndicatorController:
    """シーケンサの状態から、MMI へ送るパケットを決める。時刻は単調増加の秒。

    - LED の指示は表示が変わるときだけ送る (毎回送ると、押下で点灯に変わった表示を上書きする)
    - メロディは「受け付けられる」に変わったときに 1 回だけ送る
    - シーケンサの状態を初めて受け取るまでは LED1 に何も送らない (状態が分からないのに
      緑で「正常」に見せない)
    - 状態が status_timeout_sec 届かなければハートビートを止める。MMI が LED2 を赤にする
    - 押下のあと reblink_after_sec たっても受け付けられるままなら、点灯に変わった橙を点滅に戻す
    """

    def __init__(self, status_timeout_sec: float, reblink_after_sec: float) -> None:
        self._status_timeout = status_timeout_sec
        self._reblink_after = reblink_after_sec
        self._last_status_time: Optional[float] = None
        self._acceptable = False
        # 最後に MMI へ送った表示。None なら、次の状態で表示を送り直す
        self._shown: Optional[Display] = None
        self._pressed_at: Optional[float] = None

    @property
    def shown(self) -> Optional[Display]:
        return self._shown

    def status_fresh(self, now: float) -> bool:
        """シーケンサの状態が届いているか。false の間はハートビートを送らない。"""
        return (self._last_status_time is not None
                and now - self._last_status_time <= self._status_timeout)

    def on_status(self, trigger_acceptable: bool, now: float) -> List[bytes]:
        if not self.status_fresh(now):
            self._shown = None
        self._last_status_time = now
        self._acceptable = trigger_acceptable
        wanted = Display.WAITING if trigger_acceptable else Display.NOT_WAITING
        if wanted == self._shown:
            return []
        self._shown = wanted
        self._pressed_at = None
        if wanted == Display.WAITING:
            return [LED1_ORANGE_BLINK, MELODY_WAITING]
        return [LED1_GREEN_SOLID]

    def on_press(self, now: float) -> None:
        if self._shown == Display.WAITING:
            self._pressed_at = now

    def on_connected(self) -> List[bytes]:
        """ポートを開いたとき。MMI が起動し直していても今の表示に戻す (メロディは送らない)。"""
        self._pressed_at = None
        if self._shown == Display.WAITING:
            return [LED1_ORANGE_BLINK]
        if self._shown == Display.NOT_WAITING:
            return [LED1_GREEN_SOLID]
        return []

    def tick(self, now: float) -> List[bytes]:
        if (self._pressed_at is None or self._shown != Display.WAITING
                or now - self._pressed_at < self._reblink_after):
            return []
        self._pressed_at = None
        if self._acceptable and self.status_fresh(now):
            return [LED1_ORANGE_BLINK]
        return []
