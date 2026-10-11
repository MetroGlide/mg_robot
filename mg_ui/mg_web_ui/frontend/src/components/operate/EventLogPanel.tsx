import { useEffect, useMemo, useState } from "react";
import {
  EventEntry,
  EventLevel,
  isNotable,
  useEventStream,
} from "../../contexts/EventStreamContext";
import { Tone } from "../ui/Pill";
import Pill from "../ui/Pill";

const LEVEL_TONE: Record<EventLevel, Tone> = {
  DEBUG: "neutral",
  INFO: "neutral",
  WARN: "warn",
  ERROR: "error",
  FATAL: "error",
};

function timeText(timeMs: number): string {
  const d = new Date(timeMs);
  const pad = (n: number, w = 2) => String(n).padStart(w, "0");
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`;
}

function fieldsText(fields: Record<string, string>): string {
  return Object.entries(fields)
    .map(([k, v]) => `${k}=${v}`)
    .join(" ");
}

function Row({ event }: { event: EventEntry }) {
  return (
    <li
      className={`grid grid-cols-[6.5rem_4.5rem_1fr] items-baseline gap-2 border-b border-line/50 py-1 text-xs ${
        isNotable(event.level) ? "font-medium" : ""
      }`}
    >
      <span className="font-mono text-muted">{timeText(event.timeMs)}</span>
      <span>
        <Pill tone={LEVEL_TONE[event.level]}>{event.level}</Pill>
      </span>
      <span className="min-w-0">
        <span className="font-semibold">{event.name}</span>{" "}
        <span className="text-muted">{event.node}</span>
        <span className="block break-words text-muted">{fieldsText(event.fields)}</span>
      </span>
    </li>
  );
}

/**
 * イベントログ(状態の遷移、アクション、接続の変化、診断のレベルの変化)の一覧。新しい順。
 * 開いている間は、届いたものを既読にする(トップバーの未読の数が増えない)。
 */
export default function EventLogPanel() {
  const { events, markAllRead } = useEventStream();
  const [notableOnly, setNotableOnly] = useState(false);
  const [domain, setDomain] = useState("");

  useEffect(() => {
    markAllRead();
  }, [events, markAllRead]);

  const domains = useMemo(() => [...new Set(events.map((e) => e.domain))].sort(), [events]);
  const shown = useMemo(
    () =>
      events
        .filter((e) => (!notableOnly || isNotable(e.level)) && (!domain || e.domain === domain))
        .slice()
        .reverse(),
    [events, notableOnly, domain],
  );

  return (
    <div>
      <div className="mb-2 flex flex-wrap items-center gap-3 text-xs">
        <label className="flex items-center gap-1.5">
          <input
            type="checkbox"
            checked={notableOnly}
            onChange={(e) => setNotableOnly(e.target.checked)}
          />
          WARN 以上だけ
        </label>
        <label className="flex items-center gap-1.5">
          ドメイン
          <select
            value={domain}
            onChange={(e) => setDomain(e.target.value)}
            className="rounded border border-line bg-surface-elevated px-1 py-0.5"
          >
            <option value="">すべて</option>
            {domains.map((d) => (
              <option key={d} value={d}>
                {d}
              </option>
            ))}
          </select>
        </label>
        <span className="text-muted">{shown.length} 件(保持は最新 500 件)</span>
      </div>
      {shown.length === 0 ? (
        <p className="text-sm text-muted">イベントはありません。</p>
      ) : (
        <ul className="max-h-96 overflow-y-auto">
          {shown.map((event) => (
            <Row key={event.seq} event={event} />
          ))}
        </ul>
      )}
    </div>
  );
}
