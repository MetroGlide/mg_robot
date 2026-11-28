import { useMemo, useState } from "react";
import { Link } from "react-router-dom";
import {
  MAX_ACTIVITY_ENTRIES,
  useActivityHistory,
} from "../../contexts/ActivityHistoryContext";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useNavigationActivity } from "../../hooks/useNavigationActivity";
import ActivityLine from "../../components/operate/ActivityLine";
import { STATUS_LAMP_GUIDE_PATH } from "../../components/operate/RecentActivity";
import StatusLamp from "../../components/operate/StatusLamp";
import {
  LAMP_COLOR_LABELS,
  LampColor,
  lampOf,
  lampLabel,
} from "../../components/operate/statusLamp";
import ActionButton from "../../components/ui/ActionButton";
import WorkLayout from "../../components/work/WorkLayout";
import {
  ActivityHistoryEntry,
  dateTimeText,
  durationText,
  durationsSec,
  toTsv,
  waypointText,
} from "../../utils/activityHistory";

function fileStamp(date: Date): string {
  const pad = (n: number) => String(n).padStart(2, "0");
  return (
    `${date.getFullYear()}${pad(date.getMonth() + 1)}${pad(date.getDate())}_` +
    `${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}`
  );
}

function download(text: string, fileName: string): void {
  const url = URL.createObjectURL(
    new Blob([text], { type: "text/plain;charset=utf-8" }),
  );
  const a = document.createElement("a");
  a.href = url;
  a.download = fileName;
  a.click();
  URL.revokeObjectURL(url);
}

const selectClass =
  "rounded border border-line bg-surface-elevated px-1 py-0.5";

/**
 * 活動の履歴の全件 (保持の上限まで) の一覧。新しい順。ランプの色と code で絞り込み、表示中の分をテキスト (TSV) で書き出す。
 */
export default function ActivityLogPage({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const entries = useActivityHistory();
  const activity = useNavigationActivity(client);
  const [color, setColor] = useState<LampColor | "">("");
  const [code, setCode] = useState("");

  const durations = useMemo(() => durationsSec(entries), [entries]);
  const codes = useMemo(
    () => [...new Set(entries.map((e) => e.code))].sort(),
    [entries],
  );
  const shown = useMemo(() => {
    const rows: { entry: ActivityHistoryEntry; duration: number | null }[] = [];
    entries.forEach((entry, i) => {
      if (color && lampOf(entry.response, entry.motion).color !== color) return;
      if (code && entry.code !== code) return;
      rows.push({ entry, duration: durations[i] });
    });
    return rows.reverse();
  }, [entries, durations, color, code]);

  const exportShown = () => {
    const oldestFirst = shown.map((row) => row.entry).reverse();
    download(
      toTsv(oldestFirst, (e) => lampLabel(lampOf(e.response, e.motion))),
      `activity_${fileStamp(new Date())}.txt`,
    );
  };

  return (
    <WorkLayout
      title="走行の状態の履歴"
      headerRight={
        <>
          <Link
            to={STATUS_LAMP_GUIDE_PATH}
            className="text-xs text-muted underline hover:text-content"
          >
            ランプの見方
          </Link>
          <ActionButton
            label={`表示中の ${shown.length} 件を書き出す`}
            variant="blue"
            disabled={shown.length === 0}
            onClick={exportShown}
          />
        </>
      }
    >
      <ActivityLine activity={activity} className="text-sm" />
      <div className="flex flex-wrap items-center gap-3 text-xs">
        <label className="flex items-center gap-1.5">
          ランプ
          <select
            value={color}
            onChange={(e) => setColor(e.target.value as LampColor | "")}
            className={selectClass}
          >
            <option value="">すべて</option>
            {(Object.keys(LAMP_COLOR_LABELS) as LampColor[]).map((c) => (
              <option key={c} value={c}>
                {LAMP_COLOR_LABELS[c]}
              </option>
            ))}
          </select>
        </label>
        <label className="flex items-center gap-1.5">
          code
          <select
            value={code}
            onChange={(e) => setCode(e.target.value)}
            className={selectClass}
          >
            <option value="">すべて</option>
            {codes.map((c) => (
              <option key={c} value={c}>
                {c}
              </option>
            ))}
          </select>
        </label>
        <span className="text-muted">
          {shown.length} 件 (保持は最新 {MAX_ACTIVITY_ENTRIES}{" "}
          件。再読み込みで消える)
        </span>
      </div>
      {shown.length === 0 ? (
        <p className="text-sm text-muted">記録はありません。</p>
      ) : (
        <table className="w-full text-left text-xs">
          <thead className="text-muted">
            <tr className="border-b border-line">
              <th className="py-1 pr-2 font-medium">開始</th>
              <th className="py-1 pr-2 font-medium">続いた時間</th>
              <th className="py-1 pr-2 font-medium">ランプ</th>
              <th className="py-1 pr-2 font-medium">code</th>
              <th className="py-1 pr-2 font-medium">状態</th>
              <th className="py-1 pr-2 font-medium">補足</th>
              <th className="py-1 font-medium">地点</th>
            </tr>
          </thead>
          <tbody>
            {shown.map(({ entry, duration }) => {
              const lamp = lampOf(entry.response, entry.motion);
              return (
                <tr
                  key={entry.key}
                  className="border-b border-line/50 align-top"
                >
                  <td className="py-1 pr-2 font-mono tabular-nums text-muted">
                    {dateTimeText(entry.startMs)}
                  </td>
                  <td className="py-1 pr-2 tabular-nums">
                    {duration === null ? "-" : durationText(duration)}
                  </td>
                  <td className="py-1 pr-2">
                    <span className="flex items-center gap-1.5">
                      <StatusLamp lamp={lamp} size={10} />
                      {lampLabel(lamp)}
                    </span>
                  </td>
                  <td className="py-1 pr-2 font-mono">{entry.code}</td>
                  <td className="py-1 pr-2">{entry.text}</td>
                  <td className="break-all py-1 pr-2 text-muted">
                    {entry.detail}
                  </td>
                  <td className="py-1 tabular-nums">{waypointText(entry)}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      )}
    </WorkLayout>
  );
}
