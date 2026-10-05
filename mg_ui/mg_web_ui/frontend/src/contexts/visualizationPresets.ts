import type { LayerKey, OverlayKey } from "./VisualizationContext";

export type VisualizationPreset = "light" | "standard" | "full";

export const PRESET_LABELS: Record<VisualizationPreset, string> = {
  light: "軽量",
  standard: "標準",
  full: "すべて",
};

export const PRESET_DESCRIPTIONS: Record<VisualizationPreset, string> = {
  light:
    "地図・ロボット・経路・ウェイポイントだけを表示する。実機や端末の負荷を最小にしたいときに使う。",
  standard: "通常の運用向けの構成。",
  full: "すべてのレイヤーとオーバーレイを表示する。点群・カメラ画像・地図タイルを含み、負荷が大きい。",
};

type LayerSet = Record<LayerKey, boolean>;
type OverlaySet = Record<OverlayKey, boolean>;

const LIGHT_LAYERS: ReadonlyArray<LayerKey> = [
  "map",
  "robotPose",
  "planPath",
  "waypointMarkers",
];

/**
 * プリセットに対応するレイヤーとオーバーレイの設定を返す。
 * joystick は走行の操作に関わるため、プリセットでは変えず currentOverlays の値を保つ。
 */
export function presetSettings(
  preset: VisualizationPreset,
  standard: { layers: LayerSet; overlays: OverlaySet },
  currentOverlays: OverlaySet,
): { layers: LayerSet; overlays: OverlaySet } {
  const layerKeys = Object.keys(standard.layers) as LayerKey[];
  const overlayKeys = Object.keys(standard.overlays) as OverlayKey[];
  const all = <K extends string>(keys: K[], value: boolean) =>
    Object.fromEntries(keys.map((key) => [key, value])) as Record<K, boolean>;

  let layers: LayerSet;
  let overlays: OverlaySet;
  switch (preset) {
    case "light":
      layers = {
        ...all(layerKeys, false),
        ...all([...LIGHT_LAYERS], true),
      };
      overlays = {
        ...all(overlayKeys, false),
        velocityGauge: true,
      };
      break;
    case "full":
      layers = all(layerKeys, true);
      overlays = all(overlayKeys, true);
      break;
    default:
      layers = { ...standard.layers };
      overlays = { ...standard.overlays };
  }
  return { layers, overlays: { ...overlays, joystick: currentOverlays.joystick } };
}
