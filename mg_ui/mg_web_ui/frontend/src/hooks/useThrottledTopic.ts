import { useEffect, useState } from "react";
import { FoxgloveClientHandle } from "./useFoxgloveClient";

/**
 * トピックの最新メッセージを返す。再描画は最大 maxHz に間引く。
 *
 * 間引きで捨てるのは途中のメッセージだけで、最後のメッセージは必ず反映する
 * (間引き間隔の終わりに最新の値で更新する)。そのため、停止直前の値を取りこぼさない。
 * E-stop や FSM の状態のように 1 回の変化も見逃せない値は、間引かない useTopicSubscriber を使う。
 */
export function useThrottledTopic<T>(
  client: FoxgloveClientHandle,
  topic: string,
  schemaName: string,
  maxHz: number,
): T | null {
  const [data, setData] = useState<T | null>(null);
  const { subscribe } = client;

  useEffect(() => {
    const intervalMs = 1000 / maxHz;
    let lastEmitAt = 0;
    let pending: T | null = null;
    let timer: ReturnType<typeof setTimeout> | null = null;

    const emit = () => {
      timer = null;
      lastEmitAt = Date.now();
      setData(pending);
    };

    const unsubscribe = subscribe(topic, schemaName, (msg) => {
      pending = msg as T;
      if (timer !== null) return;
      const wait = lastEmitAt + intervalMs - Date.now();
      if (wait <= 0) emit();
      else timer = setTimeout(emit, wait);
    });

    return () => {
      unsubscribe();
      if (timer !== null) clearTimeout(timer);
    };
  }, [subscribe, topic, schemaName, maxHz]);

  return data;
}
