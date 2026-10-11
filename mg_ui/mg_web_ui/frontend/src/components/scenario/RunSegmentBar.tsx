import type { EntryStatus } from "../../types/scenarioTest";
import type { Segment } from "../../utils/scenarioTest";

const SEGMENT_CLASS: Record<EntryStatus, string> = {
  PENDING: "bg-line",
  RUNNING: "bg-accent animate-pulse",
  PASSED: "bg-ok",
  FAILED: "bg-error",
  ERROR: "bg-warn",
};

interface RunSegmentBarProps {
  segments: readonly Segment[];
  /** 渡すと、区切りを押せる(押した区切りの位置を返す) */
  onSelect?: (index: number) => void;
  /** 高さ。一覧に小さく出すときは "h-1.5" など */
  heightClass?: string;
}

/** シナリオ 1 本を 1 区切りとして、結果の色で並べる進捗バー */
export default function RunSegmentBar({
  segments,
  onSelect,
  heightClass = "h-2.5",
}: RunSegmentBarProps) {
  return (
    <div className={`flex w-full gap-0.5 ${heightClass}`}>
      {segments.map((segment, index) => {
        const cls = `${SEGMENT_CLASS[segment.status]} min-w-0 flex-1 rounded-sm`;
        const title = `${segment.label}: ${segment.status}`;
        return onSelect && segment.status !== "PENDING" ? (
          <button
            key={index}
            type="button"
            title={title}
            aria-label={title}
            onClick={() => onSelect(index)}
            className={`${cls} hover:opacity-70`}
          />
        ) : (
          <div key={index} title={title} className={cls} />
        );
      })}
    </div>
  );
}
