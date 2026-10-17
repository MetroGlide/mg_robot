import { useOpsValue } from "./useOpsValue";

/** 重ねるパネルの背景の不透明度。既定は、半透明にする前の見た目と同じ値 */
export const DEFAULT_PANEL_OPACITY = 0.95;
export const MIN_PANEL_OPACITY = 0.3;
export const MAX_PANEL_OPACITY = 1;

/** 運用ビューのパネルの背景の不透明度(端末ごとに保持する) */
export function usePanelOpacity(): [number, (value: number) => void] {
  return useOpsValue<number>("panel.opacity", DEFAULT_PANEL_OPACITY);
}
