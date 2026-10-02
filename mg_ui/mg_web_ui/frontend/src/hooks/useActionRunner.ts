import { useCallback, useState } from "react";
import type { ActionResult } from "../utils/waypointActions";

/** 非同期の操作を 1 つずつ実行し、実行中かどうかと直近の結果を持つ。例外は失敗の結果にする。 */
export function useActionRunner() {
  const [busy, setBusy] = useState(false);
  const [result, setResult] = useState<ActionResult | null>(null);

  const run = useCallback(async (action: () => Promise<ActionResult>) => {
    setBusy(true);
    setResult(null);
    try {
      setResult(await action());
    } catch (e) {
      setResult({
        ok: false,
        text: e instanceof Error ? e.message : String(e),
      });
    } finally {
      setBusy(false);
    }
  }, []);

  return { busy, result, run };
}
