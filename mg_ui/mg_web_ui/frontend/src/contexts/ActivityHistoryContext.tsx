import {
  createContext,
  ReactNode,
  useContext,
  useEffect,
  useState,
} from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { TOPICS } from "../ros/interfaces";
import { RosLog } from "../types/ros";
import {
  ActivityHistoryEntry,
  appendEntries,
  toActivityEntry,
} from "../utils/activityHistory";
import { parseEvent } from "../utils/logfmt";

/** 保持する件数。これを超えたら古いものから捨てる (イベントの一覧の上限とは別に持つ) */
export const MAX_ACTIVITY_ENTRIES = 1000;
// 表示の更新はまとめて行う (接続した直後に、履歴が一度に届くため)
const FLUSH_INTERVAL_MS = 200;

const ActivityHistoryContext = createContext<ActivityHistoryEntry[]>([]);

/**
 * 走行の活動の履歴 (古い順)。/events のうち sequencer.activity_changed だけを、全画面で共有して溜める。
 * /events は transient_local なので、画面を開き直しても中継が保つ直近の分は届く。再読み込みで消える。
 */
export function ActivityHistoryProvider({
  client,
  children,
}: {
  client: FoxgloveClientHandle;
  children: ReactNode;
}) {
  const [entries, setEntries] = useState<ActivityHistoryEntry[]>([]);
  const { subscribe } = client;

  useEffect(() => {
    const pending: ActivityHistoryEntry[] = [];
    let timer: ReturnType<typeof setTimeout> | null = null;

    const flush = () => {
      timer = null;
      const batch = pending.splice(0, pending.length);
      setEntries((prev) => appendEntries(prev, batch, MAX_ACTIVITY_ENTRIES));
    };

    const unsubscribe = subscribe(
      TOPICS.EVENTS,
      "rcl_interfaces/msg/Log",
      (msg) => {
        const log = msg as RosLog;
        const parsed = parseEvent(log.msg);
        if (parsed === null) return;
        const entry = toActivityEntry({
          name: parsed.name,
          timeMs: log.stamp.sec * 1000 + log.stamp.nanosec / 1e6,
          fields: parsed.fields,
        });
        if (entry === null) return;
        pending.push(entry);
        if (timer === null) timer = setTimeout(flush, FLUSH_INTERVAL_MS);
      },
    );
    return () => {
      unsubscribe();
      if (timer !== null) clearTimeout(timer);
    };
  }, [subscribe]);

  return (
    <ActivityHistoryContext.Provider value={entries}>
      {children}
    </ActivityHistoryContext.Provider>
  );
}

export function useActivityHistory(): ActivityHistoryEntry[] {
  return useContext(ActivityHistoryContext);
}
