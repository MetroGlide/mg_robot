import { useOpsValue } from "./useOpsValue";

/**
 * 運用ビューにセンサのレイヤーを重ねるか。ビューをまたいで共有する。
 * 負荷を増やさないよう、再読み込みでは残さず、既定はオフにする。
 */
export function useSensorsVisible(): [boolean, (value: boolean) => void] {
  return useOpsValue<boolean>("operate.sensors", false, false);
}
