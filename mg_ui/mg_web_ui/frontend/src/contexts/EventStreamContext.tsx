import {
  createContext,
  ReactNode,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { TOPICS } from "../ros/interfaces";
import { RosLog } from "../types/ros";
import { parseEvent } from "../utils/logfmt";

export type EventLevel = "DEBUG" | "INFO" | "WARN" | "ERROR" | "FATAL";

const LEVEL_NAME: Record<number, EventLevel> = {
  10: "DEBUG",
  20: "INFO",
  30: "WARN",
  40: "ERROR",
  50: "FATAL",
};

export interface EventEntry {
  /** 受信した順の通し番号 */
  seq: number;
  /** ロボットの時計での時刻(ミリ秒) */
  timeMs: number;
  level: EventLevel;
  node: string;
  name: string;
  domain: string;
  fields: Record<string, string>;
}

/** WARN 以上か */
export function isNotable(level: EventLevel): boolean {
  return level === "WARN" || level === "ERROR" || level === "FATAL";
}

/** 保持する件数。これを超えたら古いものから捨てる */
export const MAX_EVENTS = 500;
// 表示の更新はまとめて行う(接続した直後に、履歴が一度に届くため)
const FLUSH_INTERVAL_MS = 200;
// 購読してからこの間に届いたものは、履歴(以前のイベント)とみなして既読にする
const HISTORY_WINDOW_MS = 1500;

export function toEventEntry(msg: RosLog, seq: number): EventEntry | null {
  const parsed = parseEvent(msg.msg);
  if (parsed === null) return null;
  return {
    seq,
    timeMs: msg.stamp.sec * 1000 + msg.stamp.nanosec / 1e6,
    level: LEVEL_NAME[msg.level] ?? "INFO",
    node: msg.name,
    name: parsed.name,
    domain: parsed.domain,
    fields: parsed.fields,
  };
}

interface EventStreamValue {
  /** 古い順 */
  events: EventEntry[];
  /** 既読にしていない WARN 以上のイベントの数 */
  unreadCount: number;
  markAllRead: () => void;
}

const EventStreamContext = createContext<EventStreamValue>({
  events: [],
  unreadCount: 0,
  markAllRead: () => undefined,
});

/**
 * ROS 側の中継ノード(event_relay_node)が出す /events を受け取り、全画面で共有する。
 * /events は transient_local なので、接続した直後に履歴が届く(それは既読として扱う)。
 */
export function EventStreamProvider({
  client,
  children,
}: {
  client: FoxgloveClientHandle;
  children: ReactNode;
}) {
  const [events, setEvents] = useState<EventEntry[]>([]);
  const [readSeq, setReadSeq] = useState(0);
  const seqRef = useRef(0);
  const { subscribe } = client;

  useEffect(() => {
    const pending: EventEntry[] = [];
    const historyUntil = Date.now() + HISTORY_WINDOW_MS;
    let timer: ReturnType<typeof setTimeout> | null = null;

    const flush = () => {
      timer = null;
      if (pending.length === 0) return;
      const batch = pending.splice(0, pending.length);
      setEvents((prev) => [...prev, ...batch].slice(-MAX_EVENTS));
      if (Date.now() < historyUntil) setReadSeq(batch[batch.length - 1].seq);
    };

    const unsubscribe = subscribe(TOPICS.EVENTS, "rcl_interfaces/msg/Log", (msg) => {
      const entry = toEventEntry(msg as RosLog, ++seqRef.current);
      if (entry === null) return;
      pending.push(entry);
      if (timer === null) timer = setTimeout(flush, FLUSH_INTERVAL_MS);
    });
    return () => {
      unsubscribe();
      if (timer !== null) clearTimeout(timer);
    };
  }, [subscribe]);

  const markAllRead = useCallback(() => setReadSeq(seqRef.current), []);
  const unreadCount = useMemo(
    () => events.filter((e) => e.seq > readSeq && isNotable(e.level)).length,
    [events, readSeq],
  );
  const value = useMemo(
    () => ({ events, unreadCount, markAllRead }),
    [events, unreadCount, markAllRead],
  );

  return (
    <EventStreamContext.Provider value={value}>{children}</EventStreamContext.Provider>
  );
}

export function useEventStream(): EventStreamValue {
  return useContext(EventStreamContext);
}
