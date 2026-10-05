import { describe, expect, it } from "vitest";
import {
  gnssDisplay,
  localizationTone,
  quaternionToYawDeg,
  SEQUENCER_STEPS,
  sequencerStep,
} from "./displayState";

describe("gnssDisplay", () => {
  it("測位の種類を表示名と色にする", () => {
    expect(gnssDisplay(-1)).toEqual({ label: "No Fix", tone: "error" });
    expect(gnssDisplay(2)).toEqual({ label: "RTK", tone: "ok" });
    expect(gnssDisplay(9).tone).toBe("neutral");
  });
});

describe("quaternionToYawDeg", () => {
  it("z 軸まわりの回転を度にする", () => {
    expect(quaternionToYawDeg({ x: 0, y: 0, z: 0, w: 1 })).toBeCloseTo(0);
    const half = Math.PI / 4;
    expect(
      quaternionToYawDeg({ x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) }),
    ).toBeCloseTo(90);
  });
});

describe("localizationTone", () => {
  it("NORMAL は ok、DEGRADED は error、途中の状態は warn", () => {
    expect(localizationTone(0)).toBe("ok");
    expect(localizationTone(1)).toBe("warn");
    expect(localizationTone(3)).toBe("warn");
    expect(localizationTone(4)).toBe("error");
  });
});

describe("sequencerStep", () => {
  it("通常の状態は、そのままステッパーの段になる", () => {
    for (const step of SEQUENCER_STEPS) {
      expect(sequencerStep(step.id).currentId).toBe(step.id);
    }
  });

  it("SUSPENDED は走行中の段を warn にする", () => {
    expect(sequencerStep("SUSPENDED")).toEqual({
      currentId: "NAVIGATING",
      tone: "warn",
    });
  });

  it("ERROR は error、未受信は段なし", () => {
    expect(sequencerStep("ERROR").tone).toBe("error");
    expect(sequencerStep(null)).toEqual({ currentId: null, tone: "neutral" });
  });
});
