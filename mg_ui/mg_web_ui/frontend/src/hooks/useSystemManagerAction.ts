import { useCallback, useState } from "react";
import { SystemManagerHandle } from "./useSystemManagerClient";

/**
 * system_manager の API を呼び、実行中かどうかと失敗の内容を持つ。
 * system_manager に届かない場合も、例外ではなくエラーの文字列として返す。
 */
export function useSystemManagerAction(sysManager: SystemManagerHandle) {
  const { callApi } = sysManager;
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const call = useCallback(
    async (path: string, body?: unknown) => {
      setLoading(true);
      setError(null);
      try {
        const result = await callApi(path, body);
        if (!result.success) setError(result.message);
      } catch (e) {
        setError(e instanceof Error ? e.message : String(e));
      } finally {
        setLoading(false);
      }
    },
    [callApi],
  );

  return { call, loading, error };
}
