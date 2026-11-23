import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from mmi_protocol import (  # noqa: E402
    HEARTBEAT,
    LED1_GREEN_SOLID,
    LED1_ORANGE_BLINK,
    MELODY_WAITING,
    Display,
    IndicatorController,
    SwitchEventParser,
)

SW1 = bytes.fromhex("23610100")
SW2 = bytes.fromhex("23620100")
SW3 = bytes.fromhex("23630100")


# ---------------------------------------------------------------------------
# 送信パケット (マイコン担当と合わせたバイト列)
# ---------------------------------------------------------------------------

def test_packets_match_the_agreed_bytes():
    assert HEARTBEAT == bytes.fromhex("2392000000000000")
    assert LED1_GREEN_SOLID == bytes.fromhex("2392000001030000")
    assert LED1_ORANGE_BLINK == bytes.fromhex("2392000002050000")
    assert MELODY_WAITING == bytes.fromhex("2392020400000000")


# ---------------------------------------------------------------------------
# 受信の解析
# ---------------------------------------------------------------------------

def test_parses_each_switch():
    parser = SwitchEventParser()
    assert parser.feed(SW1 + SW2 + SW3) == [1, 2, 3]


def test_frame_split_across_reads():
    parser = SwitchEventParser()
    assert parser.feed(SW1[:1]) == []
    assert parser.feed(SW1[1:3]) == []
    assert parser.feed(SW1[3:]) == [1]


def test_skips_garbage_before_and_between_frames():
    parser = SwitchEventParser()
    assert parser.feed(b"\x00\xff" + SW1 + b"\x10" + SW2) == [1, 2]


def test_resyncs_after_a_header_with_an_unknown_command():
    parser = SwitchEventParser()
    # 0x23 の後が未知のコマンドなら 1 バイトだけ捨て、続く本物の通知を拾う
    assert parser.feed(bytes.fromhex("2399") + SW1) == [1]


def test_rejects_a_frame_with_wrong_data_bytes():
    parser = SwitchEventParser()
    assert parser.feed(bytes.fromhex("23610200")) == []
    assert parser.feed(SW1) == [1]


def test_reset_drops_a_partial_frame():
    parser = SwitchEventParser()
    parser.feed(SW1[:2])
    parser.reset()
    assert parser.feed(SW1[2:]) == []
    assert parser.feed(SW1) == [1]


# ---------------------------------------------------------------------------
# 表示の判断
# ---------------------------------------------------------------------------

def _controller():
    return IndicatorController(status_timeout_sec=2.0, reblink_after_sec=1.5)


def test_sends_nothing_and_no_heartbeat_before_the_first_status():
    c = _controller()
    assert c.shown is None
    assert not c.status_fresh(0.0)
    assert c.on_connected() == []
    assert c.tick(10.0) == []


def test_first_status_not_acceptable_shows_green():
    c = _controller()
    assert c.on_status(False, 0.0) == [LED1_GREEN_SOLID]
    assert c.shown == Display.NOT_WAITING
    assert c.status_fresh(0.5)


def test_becoming_acceptable_sends_orange_blink_and_melody_once():
    c = _controller()
    c.on_status(False, 0.0)
    assert c.on_status(True, 0.1) == [LED1_ORANGE_BLINK, MELODY_WAITING]
    # 同じ状態が続く間は送らない (メロディを鳴らし直さない)
    assert c.on_status(True, 0.2) == []
    assert c.on_status(True, 2.0) == []


def test_leaving_acceptable_shows_green():
    c = _controller()
    c.on_status(True, 0.0)
    assert c.on_status(False, 0.1) == [LED1_GREEN_SOLID]


def test_press_while_waiting_reblinks_if_still_acceptable():
    c = _controller()
    c.on_status(True, 0.0)
    c.on_press(1.0)
    c.on_status(True, 1.5)
    assert c.tick(2.0) == []
    c.on_status(True, 2.4)
    assert c.tick(2.5) == [LED1_ORANGE_BLINK]
    # 1 回の押下につき 1 回だけ
    assert c.tick(3.0) == []


def test_press_accepted_switches_to_green_without_reblink():
    c = _controller()
    c.on_status(True, 0.0)
    c.on_press(1.0)
    assert c.on_status(False, 1.1) == [LED1_GREEN_SOLID]
    assert c.tick(5.0) == []


def test_press_while_not_waiting_does_nothing():
    c = _controller()
    c.on_status(False, 0.0)
    c.on_press(1.0)
    c.on_status(False, 2.0)
    assert c.tick(3.0) == []


def test_heartbeat_stops_when_status_is_stale():
    c = _controller()
    c.on_status(False, 0.0)
    assert c.status_fresh(2.0)
    assert not c.status_fresh(2.1)


def test_display_is_resent_after_status_comes_back():
    c = _controller()
    c.on_status(True, 0.0)
    assert c.on_status(True, 10.0) == [LED1_ORANGE_BLINK, MELODY_WAITING]


def test_no_reblink_when_status_went_stale():
    c = _controller()
    c.on_status(True, 0.0)
    c.on_press(0.5)
    assert c.tick(5.0) == []


def test_reconnect_restores_the_display_without_melody():
    c = _controller()
    c.on_status(True, 0.0)
    assert c.on_connected() == [LED1_ORANGE_BLINK]
    c.on_status(False, 0.1)
    assert c.on_connected() == [LED1_GREEN_SOLID]
