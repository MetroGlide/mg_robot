import { useOpsValue } from "./useOpsValue";

/** 地図でマウスホイール(Ctrl なし)に割り当てる操作 */
export type WheelMode = "pan" | "zoom";

/**
 * 地図のホイールの割り当て。タッチパッドの 2 本指スクロールも同じイベントで届くので、
 * 既定は移動(pan)にし、マウスで使う端末だけ拡大縮小(zoom)に切り替える。端末ごとに保存する。
 */
export function useWheelMode(): [WheelMode, (value: WheelMode) => void] {
  return useOpsValue<WheelMode>("map.wheelMode", "pan");
}
