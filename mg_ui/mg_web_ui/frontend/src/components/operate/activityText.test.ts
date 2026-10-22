import { describe, expect, it } from "vitest";
import { NavigationActivity } from "../../types";
import { activityTone, formatActivity } from "./activityText";

function activity(overrides: Partial<NavigationActivity>): NavigationActivity {
  return {
    stamp: { sec: 0, nanosec: 0 },
    code: "idle",
    text: "Waypoint 走行なし",
    detail: "",
    emphasis: 0,
    elapsed_sec: 0,
    index: -1,
    total: -1,
    remaining: -1,
    ...overrides,
  };
}

describe("formatActivity", () => {
  it("該当しない数値は並べない", () => {
    expect(formatActivity(activity({}))).toBe("Waypoint 走行なし");
  });

  it("走行中は地点と残り距離を並べる", () => {
    expect(
      formatActivity(
        activity({
          code: "navigating",
          text: "地点へ走行中",
          index: 3,
          total: 10,
          remaining: 12.34,
        }),
      ),
    ).toBe("地点へ走行中 ・ 地点 4 / 10 ・ 残り 12.3 m");
  });

  it("外部指定のゴールは text だけを並べる", () => {
    expect(
      formatActivity(
        activity({ code: "external_goal", text: "外部指定のゴールへ走行中" }),
      ),
    ).toBe("外部指定のゴールへ走行中");
  });

  it("出発待ちの残りは秒で並べる", () => {
    expect(
      formatActivity(
        activity({ code: "departure_wait", text: "出発待ち", remaining: 2 }),
      ),
    ).toBe("出発待ち ・ 2.0 秒");
  });

  it("障害物での停止は、ポリゴン名と続いている秒数を並べる", () => {
    expect(
      formatActivity(
        activity({
          code: "obstacle_stop",
          text: "障害物で停止中",
          detail: "stop_front",
          elapsed_sec: 5.7,
        }),
      ),
    ).toBe("障害物で停止中 ・ stop_front ・ 5 秒");
  });

  it("遠回りの棄却は、経路の長さと上限、続いている秒数を並べる", () => {
    expect(
      formatActivity(
        activity({
          code: "detour_rejected",
          text: "遠回りの経路を棄却して待機中",
          detail: "経路 152 m / 上限 45 m",
          elapsed_sec: 30.2,
        }),
      ),
    ).toBe("遠回りの経路を棄却して待機中 ・ 経路 152 m / 上限 45 m ・ 30 秒");
  });

  it("1 秒未満は秒数を並べない", () => {
    expect(
      formatActivity(
        activity({ code: "recovery", text: "リカバリー: 後退", elapsed_sec: 0.4 }),
      ),
    ).toBe("リカバリー: 後退");
  });

  it("アクションの detail (type) は並べない", () => {
    expect(
      formatActivity(
        activity({ code: "action", text: "地図切り替えを実行中", detail: "load_map" }),
      ),
    ).toBe("地図切り替えを実行中");
  });

  it("一時停止は要求元を並べる", () => {
    expect(
      formatActivity(
        activity({ code: "paused", text: "一時停止中", detail: "web" }),
      ),
    ).toBe("一時停止中 ・ web");
  });
});

describe("activityTone", () => {
  it("強調を色に対応させる", () => {
    expect(activityTone(activity({ emphasis: 0 }))).toBe("neutral");
    expect(activityTone(activity({ emphasis: 1 }))).toBe("warn");
    expect(activityTone(activity({ emphasis: 2 }))).toBe("error");
  });
});
