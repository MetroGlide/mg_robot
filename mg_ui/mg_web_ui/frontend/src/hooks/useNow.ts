import { useEffect, useState } from "react";

/**
 * 現在時刻(ミリ秒)。active の間だけ、intervalMs ごとに更新する(止めている間はタイマーを持たない)。
 * 更新で再描画されるのは、このフックを使うコンポーネントだけ。
 */
export function useNow(active: boolean, intervalMs = 1000): number {
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    if (!active) return;
    setNow(Date.now());
    const id = setInterval(() => setNow(Date.now()), intervalMs);
    return () => clearInterval(id);
  }, [active, intervalMs]);
  return now;
}
