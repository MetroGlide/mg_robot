import { WaypointSegment, waypointSegments } from "./displayState";

const SEGMENT_CLASS: Record<WaypointSegment, string> = {
  passed: "bg-accent",
  current: "bg-accent/50 ring-2 ring-accent",
  ahead: "bg-line",
};

const SEGMENT_LABEL: Record<WaypointSegment, string> = {
  passed: "通過済み",
  current: "いまの地点",
  ahead: "これから",
};

/**
 * 地点の区切りのバー (通過済み・いまの地点・これから) と、地点の番号・残り距離。
 * いま何をしているかは状態ランプが表すので、ここは進み具合だけを出す。
 */
export default function WaypointBar({
  currentIndex,
  total,
  goalReached,
  distanceRemaining,
}: {
  currentIndex: number;
  total: number;
  goalReached: boolean;
  distanceRemaining: number;
}) {
  if (total <= 0) {
    return (
      <p className="text-xs text-muted">
        ウェイポイントが読み込まれていません。
      </p>
    );
  }
  const segments = waypointSegments(currentIndex, total, goalReached);
  const shown = Math.min(currentIndex + 1, total);
  return (
    <div className="space-y-1">
      <div className="flex items-center text-xs tabular-nums text-muted">
        <span>
          {goalReached
            ? `${total} / ${total} 地点 ・ 完了`
            : `${shown} / ${total} 地点`}
        </span>
        {!goalReached && (
          <span className="ml-auto">残り {distanceRemaining.toFixed(1)} m</span>
        )}
      </div>
      <ol className="flex gap-0.5" aria-label="地点の進み具合">
        {segments.map((segment, i) => (
          <li
            key={i}
            title={`地点 ${i + 1}: ${SEGMENT_LABEL[segment]}`}
            className={`h-2 min-w-[2px] flex-1 rounded-sm ${SEGMENT_CLASS[segment]}`}
          />
        ))}
      </ol>
    </div>
  );
}
