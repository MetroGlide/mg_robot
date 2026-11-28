import { describe, expect, it } from "vitest";
import {
  BLINK_PERIOD_MS,
  MOTION_BLINK_MS,
  MOTION_LABELS,
  RESPONSE_COLORS,
  RESPONSE_LABELS,
  STALE_LAMP,
  UNKNOWN_LAMP,
  lampFor,
  lampLabel,
  lampOf,
} from "./statusLamp";

describe("lampOf", () => {
  it("色は人がどうすればよいか、点滅は動いているかで決まる", () => {
    expect(lampOf("normal", "moving")).toEqual({
      color: "green",
      blinkMs: null,
    });
    expect(lampOf("act", "handling")).toEqual({
      color: "red",
      blinkMs: BLINK_PERIOD_MS.fast,
    });
    expect(lampOf("watch", "waiting")).toEqual({
      color: "yellow",
      blinkMs: BLINK_PERIOD_MS.slow,
    });
    expect(lampOf("operator", "waiting").color).toBe("blue");
    expect(lampOf("start", "waiting")).toEqual({
      color: "orange",
      blinkMs: BLINK_PERIOD_MS.slow,
    });
  });

  it("消灯は点滅しない", () => {
    expect(lampOf("none", "waiting")).toEqual({ color: "off", blinkMs: null });
  });

  it("意味が分からなければ灰", () => {
    expect(lampOf("future", "moving")).toBe(UNKNOWN_LAMP);
    expect(lampOf("normal", "future")).toBe(UNKNOWN_LAMP);
    expect(lampOf("", "")).toBe(UNKNOWN_LAMP);
  });

  it("表示名の表と見た目の表が同じ意味をそろえている", () => {
    expect(Object.keys(RESPONSE_LABELS).sort()).toEqual(
      Object.keys(RESPONSE_COLORS).sort(),
    );
    expect(Object.keys(MOTION_LABELS).sort()).toEqual(
      Object.keys(MOTION_BLINK_MS).sort(),
    );
  });
});

describe("lampFor", () => {
  it("届かないときは灰、届いたら活動の意味から決める", () => {
    expect(lampFor(null)).toBe(STALE_LAMP);
    expect(STALE_LAMP.color).toBe("gray");
    expect(lampFor({ lamp_response: "watch", lamp_motion: "moving" })).toEqual({
      color: "yellow",
      blinkMs: null,
    });
  });
});

describe("lampLabel", () => {
  it("色と点滅の速さを並べる", () => {
    expect(lampLabel(lampOf("watch", "handling"))).toBe("黄・速い点滅");
    expect(lampLabel(lampOf("start", "waiting"))).toBe("橙・遅い点滅");
    expect(lampLabel(lampOf("normal", "moving"))).toBe("緑");
    expect(lampLabel(lampOf("none", "moving"))).toBe("消灯");
  });
});
