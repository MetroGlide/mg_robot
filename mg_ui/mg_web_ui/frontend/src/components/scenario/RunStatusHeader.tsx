import type { RunProgress } from "../../types/scenarioTest";
import {
  countProgress,
  currentEntry,
  failedEntries,
} from "../../utils/scenarioTest";
import ElapsedText from "./ElapsedText";
import EntryStatusBadge from "./EntryStatusBadge";

interface RunStatusHeaderProps {
  run: RunProgress;
  /** シナリオ名 → 説明 */
  descriptions: ReadonlyMap<string, string>;
  onSelectEntry: (resultDir: string) => void;
}

/**
 * 実行状況の見出し。実行中は「いま何本目のどのシナリオか」を説明つきで出し、
 * 終わったあとは結果の要約と、失敗したシナリオの一覧を出す。
 */
export default function RunStatusHeader({
  run,
  descriptions,
  onSelectEntry,
}: RunStatusHeaderProps) {
  if (run.state === "running") {
    const current = currentEntry(run.entries);
    if (!current) {
      return (
        <div className="rounded-lg border-l-4 border-accent bg-surface-sunken px-3 py-2 text-sm">
          次のシナリオを準備しています…
        </div>
      );
    }
    const { entry, index } = current;
    const description = descriptions.get(entry.name);
    return (
      <div className="space-y-1 rounded-lg border-l-4 border-accent bg-surface-sunken px-3 py-2">
        <div className="flex flex-wrap items-baseline gap-x-3 gap-y-1">
          <span className="text-lg font-bold tabular-nums">
            実行中 {index + 1}/{run.entries.length}
          </span>
          <span className="break-all text-sm font-semibold">{entry.name}</span>
          {entry.attempt > 0 && (
            <span className="text-[10px] text-warn">再試行 {entry.attempt}</span>
          )}
          <span className="ml-auto text-xs tabular-nums text-muted">
            このシナリオ <ElapsedText startedAt={entry.started_at} />
            <span className="mx-1">/</span>
            全体 <ElapsedText startedAt={run.started_at} />
          </span>
        </div>
        {description && (
          <p className="line-clamp-3 text-xs text-muted">{description}</p>
        )}
      </div>
    );
  }

  const counts = countProgress(run.entries);
  const failed = failedEntries(run.entries);
  const aborted = run.state === "aborted";
  const allPassed = counts.total > 0 && counts.passed === counts.total;
  return (
    <div className="space-y-1 rounded-lg bg-surface-sunken px-3 py-2">
      <p
        className={`text-sm font-semibold ${
          allPassed ? "text-ok" : failed.length > 0 ? "text-error" : "text-content"
        }`}
      >
        {aborted ? "中断されました" : "完了"}:{" "}
        {allPassed
          ? `${counts.total} 本すべて PASSED`
          : `${counts.done}/${counts.total} 本を実行し、${failed.length} 本が失敗`}
      </p>
      {failed.length > 0 && (
        <ul className="space-y-0.5 text-xs">
          {failed.map((entry) => (
            <li key={entry.result_dir}>
              <button
                type="button"
                onClick={() => onSelectEntry(entry.result_dir)}
                className="flex w-full items-center gap-2 text-left hover:opacity-80"
              >
                <EntryStatusBadge status={entry.status} />
                <span className="break-all font-medium">{entry.result_dir}</span>
                <span className="truncate text-muted">{entry.message}</span>
              </button>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
