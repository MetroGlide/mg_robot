import { describe, expect, it } from "vitest";
import { quaternionToYawDeg, waypointSegments } from "./displayState";

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
    expect(waypointSegments(2, 4, false)).toEqual(["passed", "passed", "current", "ahead"]);
  });

  it("全地点に到着したら全て通過済み、地点がなければ空", () => {
    expect(waypointSegments(3, 4, true)).toEqual(["passed", "passed", "passed", "passed"]);
    expect(waypointSegments(0, 0, false)).toEqual([]);
  });
});
