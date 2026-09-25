import { describe, expect, it } from "vitest";
import type { LayerKey, OverlayKey } from "./VisualizationContext";
import { presetSettings } from "./visualizationPresets";

const LAYER_KEYS: LayerKey[] = [
  "map",
  "globalCostmap",
  "localCostmap",
  "lidarTop",
  "lidarFront",
  "robotPose",
  "particleCloud",
  "planPath",
  "actualPath",
  "waypointMarkers",
  "collisionPolygons",
  "pointCloud",
  "colorImage",
  "depthImage",
];
const OVERLAY_KEYS: OverlayKey[] = [
  "joystick",
  "velocityGauge",
  "systemMetrics",
  "gpsStatus",
  "gpsMap",
];

const layers = (on: LayerKey[]) =>
  Object.fromEntries(LAYER_KEYS.map((k) => [k, on.includes(k)])) as Record<
    LayerKey,
    boolean
  >;
const overlays = (on: OverlayKey[]) =>
  Object.fromEntries(OVERLAY_KEYS.map((k) => [k, on.includes(k)])) as Record<
    OverlayKey,
    boolean
  >;

const STANDARD = {
  layers: layers(["map", "lidarTop", "robotPose", "planPath"]),
  overlays: overlays(["velocityGauge", "systemMetrics", "gpsStatus", "gpsMap"]),
};

describe("presetSettings", () => {
  it("light enables only the essential layers and the velocity gauge", () => {
    const result = presetSettings("light", STANDARD, overlays([]));

    expect(
      LAYER_KEYS.filter((k) => result.layers[k]).sort(),
    ).toEqual(["map", "planPath", "robotPose", "waypointMarkers"]);
    expect(OVERLAY_KEYS.filter((k) => result.overlays[k])).toEqual([
      "velocityGauge",
    ]);
  });

  it("full enables every layer and overlay except joystick", () => {
    const result = presetSettings("full", STANDARD, overlays([]));

    expect(LAYER_KEYS.every((k) => result.layers[k])).toBe(true);
    expect(result.overlays.gpsMap).toBe(true);
    expect(result.overlays.joystick).toBe(false);
  });

  it("standard restores the default configuration", () => {
    const result = presetSettings("standard", STANDARD, overlays([]));

    expect(result.layers).toEqual(STANDARD.layers);
    expect(result.overlays).toEqual(STANDARD.overlays);
  });

  it.each(["light", "standard", "full"] as const)(
    "%s keeps the current joystick setting",
    (preset) => {
      expect(
        presetSettings(preset, STANDARD, overlays(["joystick"])).overlays
          .joystick,
      ).toBe(true);
      expect(
        presetSettings(preset, STANDARD, overlays([])).overlays.joystick,
      ).toBe(false);
    },
  );

  it("does not mutate the standard configuration", () => {
    const before = JSON.stringify(STANDARD);

    presetSettings("standard", STANDARD, overlays([])).layers.map = false;

    expect(JSON.stringify(STANDARD)).toBe(before);
  });
});
