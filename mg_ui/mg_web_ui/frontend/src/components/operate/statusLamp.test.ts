import { describe, expect, it } from "vitest";
import { ACTIVITY_CODE } from "../../types";
import {
  ACTIVITY_LAMPS,
  ACTIVITY_MEANINGS,
  BLINK_PERIOD_MS,
  STALE_LAMP,
  UNKNOWN_LAMP,
  lampFor,
  lampLabel,
} from "./statusLamp";

describe("ACTIVITY_LAMPS", () => {
  it("全ての活動の code にランプがある", () => {
    expect(Object.keys(ACTIVITY_LAMPS).sort()).toEqual(
      Object.values(ACTIVITY_CODE).sort(),
    );
  });

  it("赤は止まり自動では戻れない活動だけ", () => {
    const red = Object.entries(ACTIVITY_LAMPS)
      .filter(([, lamp]) => lamp.color === "red")
      .map(([code]) => code)
      .sort();
    expect(red).toEqual(["error", "no_progress", "sensor_timeout_stop"]);
  });

  it("点滅は止まっている活動だけ (走っている活動は点灯)", () => {
    for (const [code, m] of Object.entries(ACTIVITY_MEANINGS)) {
      const lamp = ACTIVITY_LAMPS[code as keyof typeof ACTIVITY_LAMPS];
      expect(lamp.blinkMs === null, code).toBe(
        m.motion === "moving" || lamp.color === "off",
      );
    }
  });

  it("異常で止まっている活動は速い点滅、待っている活動は遅い点滅", () => {
    expect(ACTIVITY_LAMPS.error).toEqual({
      color: "red",
      blinkMs: BLINK_PERIOD_MS.fast,
    });
    expect(ACTIVITY_LAMPS.recovery).toEqual({
      color: "yellow",
      blinkMs: BLINK_PERIOD_MS.fast,
    });
    expect(ACTIVITY_LAMPS.obstacle_stop).toEqual({
      color: "yellow",
      blinkMs: BLINK_PERIOD_MS.slow,
    });
    expect(ACTIVITY_LAMPS.paused).toEqual({
      color: "blue",
      blinkMs: BLINK_PERIOD_MS.slow,
    });
    expect(ACTIVITY_LAMPS.goal_reached).toEqual({
      color: "green",
      blinkMs: BLINK_PERIOD_MS.slow,
    });
    expect(ACTIVITY_LAMPS.idle).toEqual({ color: "off", blinkMs: null });
  });

  it("開始待ちは MMI の LED1 と同じ橙の点滅", () => {
    for (const code of [
      ACTIVITY_CODE.TRIGGER_WAIT,
      ACTIVITY_CODE.READY_TO_START,
    ]) {
      expect(ACTIVITY_LAMPS[code]).toEqual({
        color: "orange",
        blinkMs: BLINK_PERIOD_MS.slow,
      });
    }
  });
});

describe("lampFor", () => {
  it("届かないときと表にない code は灰", () => {
    expect(lampFor(null)).toBe(STALE_LAMP);
    expect(STALE_LAMP.color).toBe("gray");
    expect(lampFor("future_code")).toBe(UNKNOWN_LAMP);
    expect(lampFor("navigating").color).toBe("green");
  });
});

describe("lampLabel", () => {
  it("色と点滅の速さを並べる", () => {
    expect(lampLabel(lampFor("recovery"))).toBe("黄・速い点滅");
    expect(lampLabel(lampFor("ready_to_start"))).toBe("橙・遅い点滅");
    expect(lampLabel(lampFor("paused"))).toBe("青・遅い点滅");
    expect(lampLabel(lampFor("navigating"))).toBe("緑");
    expect(lampLabel(lampFor("idle"))).toBe("消灯");
  });
});
