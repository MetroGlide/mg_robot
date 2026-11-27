import { describe, expect, it } from "vitest";
import {
  nextActionIndex,
  quaternionToYawDeg,
  waypointSegments,
  waypointWindow,
} from "./displayState";

describe("quaternionToYawDeg", () => {
  it("z 軸まわりの回転を度にする", () => {
    expect(quaternionToYawDeg({ x: 0, y: 0, z: 0, w: 1 })).toBeCloseTo(0);
    const half = Math.PI / 4;
    expect(
      quaternionToYawDeg({ x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) }),
    ).toBeCloseTo(90);
  });
});

describe("waypointSegments", () => {
  it("番号より前は通過済み、番号の地点はいま、後ろはこれから", () => {
    expect(waypointSegments(2, 4, false)).toEqual([
      "passed",
      "passed",
      "current",
      "ahead",
    ]);
  });

  it("全地点に到着したら全て通過済み、地点がなければ空", () => {
    expect(waypointSegments(3, 4, true)).toEqual([
      "passed",
      "passed",
      "passed",
      "passed",
    ]);
    expect(waypointSegments(0, 0, false)).toEqual([]);
  });
});

describe("waypointWindow", () => {
  it("地点が少なければ全部を見せる", () => {
    expect(waypointWindow(3, 10, 5, 15)).toEqual({ start: 0, end: 10 });
  });

  it("地点が多ければ、いまの地点の前後だけを見せ、端では寄せる", () => {
    expect(waypointWindow(100, 200, 5, 15)).toEqual({ start: 95, end: 116 });
    expect(waypointWindow(2, 200, 5, 15)).toEqual({ start: 0, end: 21 });
    expect(waypointWindow(199, 200, 5, 15)).toEqual({ start: 179, end: 200 });
  });
});

describe("nextActionIndex", () => {
  it("いまの地点から先で最初にアクションがある地点", () => {
    const types = [["load_map"], [], [], ["wait_trigger"]];
    expect(nextActionIndex(types, 0)).toBe(0);
    expect(nextActionIndex(types, 1)).toBe(3);
    expect(nextActionIndex([[], []], 0)).toBeNull();
  });
});
