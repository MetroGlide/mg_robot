import { useMemo } from "react";
import { Link } from "react-router-dom";
import { useActivityHistory } from "../../contexts/ActivityHistoryContext";
import { NavigationActivity } from "../../types";
import {
  clockText,
  durationText,
  durationsSec,
  waypointText,
} from "../../utils/activityHistory";
import StatusLamp from "./StatusLamp";
import { lampOf } from "./statusLamp";

/** 直近に並べる件数の既定 */
export const RECENT_ACTIVITY_COUNT = 5;
/** 全件の一覧のページ */
export const ACTIVITY_LOG_PATH = "/ops/activity-log";
/** 状態ランプの見方のページ */
export const STATUS_LAMP_GUIDE_PATH = "/ops/status-lamps";

/**
 * 直近の活動の変化を新しい順に count 件並べる。続いている最新の件は、いまの活動の経過秒を出す。
 */
export default function RecentActivity({
  activity,
  count = RECENT_ACTIVITY_COUNT,
}: {
  activity: NavigationActivity | null;
  count?: number;
}) {
  const entries = useActivityHistory();
  const rows = useMemo(() => {
    const durations = durationsSec(entries);
    return entries
      .map((entry, i) => ({ entry, duration: durations[i] }))
      .slice(-count)
      .reverse();
  }, [entries, count]);

  return (
    <div className="mt-2">
      <div className="mb-1 flex items-center text-xs text-muted">
        <span>直近の状態</span>
        <Link
          to={STATUS_LAMP_GUIDE_PATH}
          className="ml-auto underline hover:text-content"
        >
          ランプの見方
        </Link>
        <Link
          to={ACTIVITY_LOG_PATH}
          className="ml-3 underline hover:text-content"
        >
          すべて表示
        </Link>
      </div>
      {rows.length === 0 ? (
        <p className="text-xs text-muted">まだ記録がありません。</p>
      ) : (
        <ul className="space-y-0.5 text-xs">
          {rows.map(({ entry, duration }, i) => {
            const ongoing =
              i === 0 && activity !== null && activity.code === entry.code
                ? activity.elapsed_sec
                : null;
            return (
              <li
                key={entry.key}
                className="grid grid-cols-[auto_4.5rem_1fr_auto] items-center gap-2"
              >
                <StatusLamp
                  lamp={lampOf(entry.response, entry.motion)}
                  size={8}
                />
                <span className="font-mono tabular-nums text-muted">
                  {clockText(entry.startMs)}
                </span>
                <span
                  className="min-w-0 truncate"
                  title={entry.detail || undefined}
                >
                  {entry.text}
                  {waypointText(entry) && (
                    <span className="text-muted">
                      {" "}
                      ・ 地点 {waypointText(entry)}
                    </span>
                  )}
                </span>
                <span className="tabular-nums text-muted">
                  {durationText(duration ?? ongoing)}
                </span>
              </li>
            );
          })}
        </ul>
      )}
    </div>
  );
}
