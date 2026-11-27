import { describe, expect, it } from "vitest";
import { LocalizationCheck, LocalizationStatus } from "../../types";
import { localizationDetail } from "./localizationText";

function check(name: string, status: number): LocalizationCheck {
  return { name, detector: "pose_consistency", sources: [], status, score: 0, accumulated: 0, values: [] };
}

function status(overrides: Partial<LocalizationStatus>): LocalizationStatus {
  return {
    state: 0,
    blame: "",
    contaminated: [],
    isolated_sources: [],
    hold_requested: false,
    fusion_level: 0,
    trusted: true,
    in_grace: false,
    footprint_in_lethal: false,
    lethal_depth_m: 0,
    situation: "",
    degree: 0,
    possible_error_m: -1,
    clearance_m: -1,
    speed_limit: 0,
    sensor_set: "",
    verify_id: 0,
    verify_result: "",
    profile: "",
    last_event: "",
    checks: [],
    ...overrides,
  };
}

describe("localizationDetail", () => {
  it("正常なら何も並べない", () => {
    expect(localizationDetail(status({ checks: [check("amcl_gnss", 0), check("scan_ekf", 2)] }))).toBe("");
  });

  it("疑わしい情報源と異常の判定を並べる", () => {
    expect(
      localizationDetail(
        status({ state: 4, blame: "amcl", checks: [check("amcl_gnss", 1), check("amcl_jump", 1), check("x", 0)] }),
      ),
    ).toBe("疑義: amcl · amcl_gnss, amcl_jump");
  });

  it("障害物の中にいれば深さを添える", () => {
    expect(localizationDetail(status({ footprint_in_lethal: true, lethal_depth_m: 0.123 }))).toBe(
      "障害物内 0.12 m",
    );
    expect(localizationDetail(status({ footprint_in_lethal: true, lethal_depth_m: -1 }))).toBe("障害物内");
  });

  it("自己位置状況・速度の上限・センサ構成を並べる (正常の状況は並べない)", () => {
    expect(localizationDetail(status({ situation: "normal" }))).toBe("");
    expect(
      localizationDetail(status({ situation: "suspect", speed_limit: 0.4, sensor_set: "no_lidar" })),
    ).toBe("状況: suspect · 上限 0.40 m/s · センサ: no_lidar");
  });
});
