import { WaypointInfo } from "../../types";
import {
  WaypointSegment,
  nextActionIndex,
  waypointSegments,
  waypointWindow,
} from "./displayState";

const SEGMENT_CLASS: Record<WaypointSegment, string> = {
  passed: "bg-accent",
  current: "bg-accent/50 ring-2 ring-accent",
  ahead: "bg-line",
};

const SEGMENT_LABEL: Record<WaypointSegment, string> = {
  passed: "通過済み",
  current: "いま向かっている",
  ahead: "これから",
};

// 区切りで見せる地点の数 (いまの地点の前と後)。地点が多いときは、全体を細いバーで出し、この範囲だけを区切る
const WINDOW_BEFORE = 5;
const WINDOW_AFTER = 15;

function kindText(wp: WaypointInfo): string {
  return wp.is_through_point
    ? `通過点 (半径 ${wp.through_tolerance.toFixed(1)} m)`
    : "停止点 (到達半径は Nav2 の共通設定)";
}

function actionsText(types: readonly string[]): string {
  return types.length === 0 ? "アクションなし" : types.join(", ");
}

/**
 * 地点の進み具合。全体の細いバー、いまの地点の前後の区切り (停止点とアクションのある地点に印)、
 * 向かっている地点の種類と予定のアクション、その先で次にアクションがある地点。
 * シーケンサの番号と一覧を並べるだけで、いま何をしているかは状態ランプが表す。
 */
export default function WaypointBar({
  currentIndex,
  total,
  goalReached,
  distanceRemaining,
  waypoints,
}: {
  currentIndex: number;
  total: number;
  goalReached: boolean;
  distanceRemaining: number;
  /** ~/waypoints の一覧。届いていない、または総数と合わないときは null */
  waypoints: WaypointInfo[] | null;
}) {
  if (total <= 0) {
    return (
      <p className="text-xs text-muted">
        ウェイポイントが読み込まれていません。
      </p>
    );
  }
  const list =
    waypoints !== null && waypoints.length === total ? waypoints : null;
  const segments = waypointSegments(currentIndex, total, goalReached);
  const { start, end } = waypointWindow(
    currentIndex,
    total,
    WINDOW_BEFORE,
    WINDOW_AFTER,
  );
  const windowed = end - start < total;
  const passedRatio = goalReached
    ? 1
    : Math.min(1, Math.max(0, currentIndex) / total);
  const target = !goalReached && list ? list[currentIndex] : undefined;
  const nextAction =
    !goalReached && list
      ? nextActionIndex(
          list.map((wp) => wp.on_reached_action_types),
          currentIndex + 1,
        )
      : null;

  return (
    <div className="space-y-1.5">
      <div className="flex items-center text-xs tabular-nums text-muted">
        <span>
          {goalReached
            ? `${total} / ${total} 地点 ・ 完了`
            : `${Math.min(currentIndex + 1, total)} / ${total} 地点`}
        </span>
        {!goalReached && (
          <span className="ml-auto">残り {distanceRemaining.toFixed(1)} m</span>
        )}
      </div>

      {windowed && (
        <div
          className="relative h-1.5 rounded-full bg-line"
          aria-label="全体の進み具合"
        >
          <div
            className="h-full rounded-full bg-accent"
            style={{ width: `${passedRatio * 100}%` }}
          />
          {/* 下の区切りが見せている範囲 */}
          <div
            className="absolute -top-0.5 h-2.5 rounded-sm border border-accent"
            style={{
              left: `${(start / total) * 100}%`,
              width: `${((end - start) / total) * 100}%`,
            }}
          />
        </div>
      )}

      <div className="flex items-end gap-1 text-[10px] tabular-nums text-muted">
        {windowed && <span>{start + 1}</span>}
        <ol className="flex flex-1 items-end gap-0.5" aria-label="地点の区切り">
          {segments.slice(start, end).map((segment, offset) => {
            const i = start + offset;
            const wp = list?.[i];
            const hasActions = (wp?.on_reached_action_types.length ?? 0) > 0;
            const stop = wp !== undefined && !wp.is_through_point;
            return (
              <li
                key={i}
                title={
                  `地点 ${i + 1}: ${SEGMENT_LABEL[segment]}` +
                  (wp
                    ? ` ・ ${kindText(wp)} ・ ${actionsText(wp.on_reached_action_types)}`
                    : "")
                }
                className="flex min-w-0 flex-1 flex-col items-center gap-0.5"
              >
                <span
                  className={`h-1 w-1 rounded-full ${hasActions ? "bg-warn" : "bg-transparent"}`}
                />
                <span
                  className={`w-full rounded-sm ${stop ? "h-3" : "h-2"} ${SEGMENT_CLASS[segment]}`}
                />
              </li>
            );
          })}
        </ol>
        {windowed && <span>{end}</span>}
      </div>
      {list && (
        <p className="text-[10px] text-muted">
          高い区切りは停止点、上の点は到着後アクションのある地点
        </p>
      )}

      {target && (
        <p className="text-xs">
          <span className="text-muted">向かう地点 {currentIndex + 1}: </span>
          {kindText(target)} ・ {actionsText(target.on_reached_action_types)}
        </p>
      )}
      {list && nextAction !== null && (
        <p className="text-xs">
          <span className="text-muted">その先で次にアクション: </span>
          地点 {nextAction + 1} (
          {actionsText(list[nextAction].on_reached_action_types)}) ・ あと{" "}
          {nextAction - currentIndex} 地点
        </p>
      )}
    </div>
  );
}
