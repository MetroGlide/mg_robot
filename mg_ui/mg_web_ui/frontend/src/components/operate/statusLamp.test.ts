import { describe, expect, it } from "vitest";
import { ACTIVITY_CODE } from "../../types";
import {
  ACTIVITY_LAMPS,
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
  it("届かないときは赤、表にない code は灰", () => {
    expect(lampFor(null)).toBe(STALE_LAMP);
    expect(lampFor("future_code")).toBe(UNKNOWN_LAMP);
    expect(lampFor("navigating").color).toBe("green");
  });
});

describe("lampLabel", () => {
  it("色と点滅の速さを並べる", () => {
    expect(lampLabel(lampFor("recovery"))).toBe("黄・速い点滅");
    expect(lampLabel(lampFor("ready_to_start"))).toBe("橙・遅い点滅");
    expect(lampLabel(lampFor("paused"))).toBe("青");
    expect(lampLabel(lampFor("idle"))).toBe("消灯");
  });
});
