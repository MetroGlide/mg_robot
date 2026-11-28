"""MMI (スイッチ・RGB LED・ブザーの操作盤マイコン) の通信と表示の判断。ROS に依存しない。

通信は USB CDC の固定長のバイナリ:

- PC → MMI: 8 バイト `23 CMD DATA1..DATA6`。`0x92` はブザー・RGB LED の制御とハートビートを兼ねる
- MMI → PC: 4 バイト `23 61/62/63 01 00` (SW1/SW2/SW3 の押下。離したときは通知しない)

LED1 は、シーケンサの「いまの活動」の状態ランプの意味 (人がどうすればよいか・動いているか) に合わせる。
色は意味ごとに決め (定義色)、動いていれば点灯、止まっていれば点滅にする (MMI の点滅は 1 種類だけ)。
今までの表示 (受け付けられる間は橙の点滅、それ以外は緑の点灯) にも切り替えられる。
シーケンサが開始要求を受け付けられるようになったら、メロディを鳴らす。
物理スイッチを押すと、MMI が自分でブザーを止め、点滅を点灯に変える。
"""
from dataclasses import dataclass
from typing import List, Mapping, Optional

HEADER = 0x23
CMD_CONTROL = 0x92

_SWITCH_CMDS = {0x61: 1, 0x62: 2, 0x63: 3}
_EVENT_SIZE = 4

# LED の Control (定義色の点灯・点滅)
LED_SOLID = 0x01
LED_BLINK = 0x02
_BUZZER_REPEAT = 0x02
# 定義色 (MMI の仕様書の色番号)
COLOR_OFF = 0x00
COLOR_RED = 0x01
COLOR_YELLOW = 0x02
COLOR_GREEN = 0x03
COLOR_BLUE = 0x04
COLOR_ORANGE = 0x05
COLOR_WHITE = 0x06
_MELODY_WAITING = 0x04

# 活動の lamp_motion のうち、点灯にするもの (それ以外は点滅)
_MOVING = 'moving'


def control_packet(buzzer_control: int = 0, buzzer_data: int = 0,
                   led1_control: int = 0, led1_data: int = 0,
                   led2_control: int = 0, led2_data: int = 0) -> bytes:
    """`0x92` の 8 バイト。control が 0 の機能は変えない。"""
    return bytes([HEADER, CMD_CONTROL, buzzer_control, buzzer_data,
                  led1_control, led1_data, led2_control, led2_data])


# 全機能を変えず、通信が生きていることだけを伝える
HEARTBEAT = control_packet()
LED1_GREEN_SOLID = control_packet(led1_control=LED_SOLID, led1_data=COLOR_GREEN)
LED1_ORANGE_BLINK = control_packet(led1_control=LED_BLINK, led1_data=COLOR_ORANGE)
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


@dataclass(frozen=True)
class Led:
    """LED 1 つの表示 (Control と定義色)。"""
    control: int
    color: int

    def packet(self) -> bytes:
        return control_packet(led1_control=self.control, led1_data=self.color)

    def describe(self) -> str:
        mode = 'blink' if self.control == LED_BLINK else 'solid'
        return f'{mode}:{self.color:#04x}'


_WAITING = Led(LED_BLINK, COLOR_ORANGE)
_NOT_WAITING = Led(LED_SOLID, COLOR_GREEN)


class IndicatorController:
    """シーケンサの状態と「いまの活動」から、MMI へ送るパケットを決める。時刻は単調増加の秒。

    - LED の指示は表示が変わるときだけ送る (毎回送ると、押下で点灯に変わった表示を上書きする)
    - メロディは「受け付けられる」に変わったときに 1 回だけ送る
    - シーケンサの状態を初めて受け取るまでは LED1 に何も送らない (状態が分からないのに
      正常に見せない)
    - 状態が status_timeout_sec 届かなければハートビートを止める。MMI が LED2 を赤にする
    - 活動に合わせる設定で、活動が activity_timeout_sec 届かなければ unknown_color の点灯にする
    - 押下のあと reblink_after_sec たっても点滅の表示のままなら、点灯に変わった点滅を送り直す
    """

    def __init__(self, status_timeout_sec: float, reblink_after_sec: float,
                 follow_activity: bool = False, activity_timeout_sec: float = 3.0,
                 colors: Optional[Mapping[str, int]] = None,
                 unknown_color: int = COLOR_WHITE) -> None:
        self._status_timeout = status_timeout_sec
        self._reblink_after = reblink_after_sec
        self._follow_activity = follow_activity
        self._activity_timeout = activity_timeout_sec
        self._colors = dict(colors or {})
        self._unknown_color = unknown_color
        self._last_status_time: Optional[float] = None
        self._acceptable = False
        self._activity: Optional[tuple] = None
        self._last_activity_time: Optional[float] = None
        # 最後に MMI へ送った表示。None なら、次の状態で表示を送り直す
        self._shown: Optional[Led] = None
        self._pressed_at: Optional[float] = None

    @property
    def shown(self) -> Optional[Led]:
        return self._shown

    def status_fresh(self, now: float) -> bool:
        """シーケンサの状態が届いているか。false の間はハートビートを送らない。"""
        return (self._last_status_time is not None
                and now - self._last_status_time <= self._status_timeout)

    def _activity_fresh(self, now: float) -> bool:
        return (self._last_activity_time is not None
                and now - self._last_activity_time <= self._activity_timeout)

    def _wanted(self, now: float) -> Optional[Led]:
        if not self.status_fresh(now):
            return None
        if not self._follow_activity:
            return _WAITING if self._acceptable else _NOT_WAITING
        if self._activity is None or not self._activity_fresh(now):
            return Led(LED_SOLID, self._unknown_color)
        response, motion = self._activity
        color = self._colors.get(response, self._unknown_color)
        solid = motion == _MOVING or color == COLOR_OFF
        return Led(LED_SOLID if solid else LED_BLINK, color)

    def _refresh(self, now: float) -> List[bytes]:
        wanted = self._wanted(now)
        if wanted is None:
            self._shown = None
            return []
        if wanted == self._shown:
            return []
        self._shown = wanted
        self._pressed_at = None
        return [wanted.packet()]

    def on_status(self, trigger_acceptable: bool, now: float) -> List[bytes]:
        was_acceptable = self._acceptable and self.status_fresh(now)
        if not self.status_fresh(now):
            self._shown = None
        self._last_status_time = now
        self._acceptable = trigger_acceptable
        packets = self._refresh(now)
        if trigger_acceptable and not was_acceptable:
            packets.append(MELODY_WAITING)
        return packets

    def on_activity(self, response: str, motion: str, now: float) -> List[bytes]:
        self._activity = (response, motion)
        self._last_activity_time = now
        return self._refresh(now)

    def on_press(self, now: float) -> None:
        if self._shown is not None and self._shown.control == LED_BLINK:
            self._pressed_at = now

    def on_connected(self) -> List[bytes]:
        """ポートを開いたとき。MMI が起動し直していても今の表示に戻す (メロディは送らない)。"""
        self._pressed_at = None
        return [] if self._shown is None else [self._shown.packet()]

    def tick(self, now: float) -> List[bytes]:
        packets = self._refresh(now)
        if packets or self._pressed_at is None or now - self._pressed_at < self._reblink_after:
            return packets
        self._pressed_at = None
        if self._shown is not None and self._shown.control == LED_BLINK:
            return [self._shown.packet()]
        return []
