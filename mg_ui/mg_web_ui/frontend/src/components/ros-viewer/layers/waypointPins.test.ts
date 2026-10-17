import { describe, expect, it } from "vitest";
import { Marker, MARKER_TYPE } from "../../../types/ros-types";
import { pinState, waypointPinsFromMarkers } from "./waypointPins";

const marker = (
  ns: string,
  id: number,
  overrides: Partial<Marker> = {},
): Marker =>
  ({
    ns,
    id,
    type: MARKER_TYPE.ARROW,
    pose: {
      position: { x: id, y: 2 * id, z: 1 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    },
    color: { r: 1, g: 0, b: 0, a: 1 },
    ...overrides,
  }) as Marker;

describe("waypointPinsFromMarkers", () => {
  it("ウェイポイントの矢印だけを、番号の順に取り出す", () => {
    const pins = waypointPinsFromMarkers([
      marker("waypoints", 2),
      marker("waypoints_text", 1, { type: MARKER_TYPE.TEXT_VIEW_FACING }),
      marker("waypoints", 1),
    ]);
    expect(pins.map((p) => p.index)).toEqual([1, 2]);
    expect(pins[0]).toMatchObject({ x: 1, y: 2, yaw: 0 });
  });

  it("緑を通過点、赤を停止点として扱う", () => {
    const [stop, through] = waypointPinsFromMarkers([
      marker("waypoints", 0),
      marker("waypoints", 1, { color: { r: 0, g: 1, b: 0, a: 1 } }),
    ]);
    expect(stop.through).toBe(false);
    expect(through.through).toBe(true);
  });

  it("姿勢のクォータニオンから向きを求める", () => {
    const [pin] = waypointPinsFromMarkers([
      marker("waypoints", 0, {
        pose: {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2 },
        },
      }),
    ]);
    expect(pin.yaw).toBeCloseTo(Math.PI / 2);
  });
});

describe("pinState", () => {
  it("走行中は、通過済み・目標・これからを区別する", () => {
    expect(pinState(0, 1, true)).toBe("past");
    expect(pinState(1, 1, true)).toBe("current");
    expect(pinState(2, 1, true)).toBe("upcoming");
  });

  it("走行していないときは、すべて通常の表示にする", () => {
    expect(pinState(0, 1, false)).toBe("upcoming");
    expect(pinState(0, null, true)).toBe("upcoming");
  });
});
