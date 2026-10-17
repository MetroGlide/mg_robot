import { useCallback, useSyncExternalStore } from "react";
import { opsViewStore } from "../utils/opsViewStore";

/**
 * 画面の切り替えをまたいで保持する値(useState の代わり)。
 * key は "<ユースケースや部品>.<項目>" の形にする。値は JSON にできるものだけ。
 * 既定値は、保存された値がないときだけ使う。persist=false なら再読み込みでは残さない。
 *
 * 既定値にオブジェクトを渡す場合は、呼び出しのたびに作り直さず、定数にすること(参照が変わると再描画が続く)。
 */
export function useOpsValue<T>(
  key: string,
  fallback: T,
  persist = true,
): [T, (value: T) => void] {
  const value = useSyncExternalStore(
    opsViewStore.subscribe,
    () => opsViewStore.get(key, fallback),
  );
  const setValue = useCallback(
    (next: T) => opsViewStore.set(key, next, persist),
    [key, persist],
  );
  return [value, setValue];
}
