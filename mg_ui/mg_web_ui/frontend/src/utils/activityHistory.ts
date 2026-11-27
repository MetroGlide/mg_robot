/**
 * 走行の「いまの活動」の履歴。源はイベント sequencer.activity_changed (waypoint_sequencer_node が、
 * 採った活動の code が変わるたびに出す)。UI は受け取ったものを並べるだけで、判定はしない。
 * React に依存しない。
 */

export const ACTIVITY_CHANGED_EVENT = "sequencer.activity_changed";

export interface ActivityHistoryEntry {
  /** 重複 (接続し直したときに届く履歴) を除くための鍵 */
  key: string;
  /** この活動が始まった時刻 (ロボットの時計。ミリ秒)。イベントの時刻から、すでに続いていた秒数をさかのぼる */
  startMs: number;
  code: string;
  /** 前の活動の code (最初の活動では空) */
  from: string;
  /** 前の活動が続いた秒数 */
  fromSec: number;
  text: string;
  detail: string;
  /** 地点の番号 (0 始まり) と総数。該当しなければ -1 */
  index: number;
  total: number;
}

export interface ActivityEventLike {
  name: string;
  timeMs: number;
  fields: Record<string, string>;
}

function toNumber(value: string | undefined, fallback: number): number {
  if (value === undefined || value === "") return fallback;
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

/** イベントを履歴の 1 件にする。活動の変化のイベントでなければ null */
export function toActivityEntry(
  event: ActivityEventLike,
): ActivityHistoryEntry | null {
  if (event.name !== ACTIVITY_CHANGED_EVENT) return null;
  const { fields } = event;
  const code = fields.to ?? "";
  if (code === "") return null;
  const startMs = event.timeMs - toNumber(fields.since_sec, 0) * 1000;
  return {
    key: `${event.timeMs}:${fields.from ?? ""}:${code}`,
    startMs,
    code,
    from: fields.from ?? "",
    fromSec: toNumber(fields.from_sec, 0),
    text: fields.text ?? "",
    detail: fields.detail ?? "",
    index: toNumber(fields.index, -1),
    total: toNumber(fields.total, -1),
  };
}

/**
 * 履歴に新しい件を足す (古い順)。重複は除き、上限を超えたら古いものから捨てる。
 */
export function appendEntries(
  prev: ActivityHistoryEntry[],
  added: ActivityHistoryEntry[],
  maxEntries: number,
): ActivityHistoryEntry[] {
  const seen = new Set(prev.map((e) => e.key));
  const fresh = added.filter((e) => {
    if (seen.has(e.key)) return false;
    seen.add(e.key);
    return true;
  });
  if (fresh.length === 0) return prev;
  return [...prev, ...fresh]
    .sort((a, b) => a.startMs - b.startMs)
    .slice(-maxEntries);
}

/**
 * 各件の続いた秒数。次の件の前の活動がこの件なら、その秒数 (ロボットが数えた値)。
 * 最新の件 (続いている) と、間の件が欠けて分からないものは null。
 */
export function durationsSec(
  entries: ActivityHistoryEntry[],
): (number | null)[] {
  return entries.map((entry, i) => {
    const next = entries[i + 1];
    if (next === undefined || next.from !== entry.code) return null;
    return next.fromSec;
  });
}

function pad(n: number, width = 2): string {
  return String(n).padStart(width, "0");
}

/** 時刻の表示 (時:分:秒) */
export function clockText(timeMs: number): string {
  const d = new Date(timeMs);
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}

/** 日時の表示 (書き出し用。年-月-日 時:分:秒.ミリ秒) */
export function dateTimeText(timeMs: number): string {
  const d = new Date(timeMs);
  return (
    `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} ` +
    `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`
  );
}

/** 地点の表示 (例: 3 / 10)。該当しなければ空 */
export function waypointText(entry: ActivityHistoryEntry): string {
  return entry.index >= 0 && entry.total > 0
    ? `${entry.index + 1} / ${entry.total}`
    : "";
}

/** 続いた秒数の表示 */
export function durationText(sec: number | null): string {
  if (sec === null) return "";
  if (sec < 60) return `${sec.toFixed(1)} 秒`;
  return `${Math.floor(sec / 60)} 分 ${Math.floor(sec % 60)} 秒`;
}

function tsvCell(value: string): string {
  return value.replace(/[\t\r\n]+/g, " ");
}

/**
 * 履歴を TSV のテキストにする (古い順)。lampText は code からランプの表示名を引く関数。
 */
export function toTsv(
  entries: ActivityHistoryEntry[],
  lampText: (code: string) => string,
): string {
  const durations = durationsSec(entries);
  const header = [
    "start",
    "duration_sec",
    "lamp",
    "code",
    "text",
    "detail",
    "waypoint",
  ];
  const rows = entries.map((entry, i) => [
    dateTimeText(entry.startMs),
    durations[i] === null ? "" : durations[i]!.toFixed(1),
    lampText(entry.code),
    entry.code,
    entry.text,
    entry.detail,
    waypointText(entry),
  ]);
  return (
    [header, ...rows].map((row) => row.map(tsvCell).join("\t")).join("\n") +
    "\n"
  );
}
