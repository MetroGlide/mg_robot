import { useEffect, useState } from "react";
import { FoxgloveClientHandle } from "./useFoxgloveClient";

export interface Freshness {
  /** 最後の受信からの経過秒。一度も受信していなければ null */
  ageSec: number | null;
  /** 一度も受信していない、または maxAgeSec を超えている */
  stale: boolean;
}

export function evaluateFreshness(
  lastAt: number | null,
  now: number,
  maxAgeSec: number,
): Freshness {
  if (lastAt === null) return { ageSec: null, stale: true };
  const ageSec = Math.max(0, (now - lastAt) / 1000);
  return { ageSec, stale: ageSec > maxAgeSec };
}

/**
 * トピックの鮮度を返す。受信時刻はブラウザの時計で記録されるので、ロボット PC との時計のずれの影響を受けない。
 * 判定は 1 秒ごとに行い、受信のたびには再描画しない。
 */
export function useFreshness(
  client: FoxgloveClientHandle,
  topic: string,
  maxAgeSec: number,
): Freshness {
  const { getLastMessageAt } = client;
  const [freshness, setFreshness] = useState<Freshness>(() =>
    evaluateFreshness(getLastMessageAt(topic), Date.now(), maxAgeSec),
  );

  useEffect(() => {
    const update = () => {
      const next = evaluateFreshness(
        getLastMessageAt(topic),
        Date.now(),
        maxAgeSec,
      );
      // 秒未満の差で再描画しない
      setFreshness((prev) =>
        prev.stale === next.stale &&
        prev.ageSec !== null &&
        next.ageSec !== null &&
        Math.floor(prev.ageSec) === Math.floor(next.ageSec)
          ? prev
          : next,
      );
    };
    update();
    const id = setInterval(update, 1000);
    return () => clearInterval(id);
  }, [getLastMessageAt, topic, maxAgeSec]);

  return freshness;
}
