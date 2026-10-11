import type { RunProgress } from "../../types/scenarioTest";
import {
  countProgress,
  formatElapsed,
  formatRunTime,
  segmentsFromEntries,
} from "../../utils/scenarioTest";
import ElapsedText from "./ElapsedText";
import EntryStatusBadge from "./EntryStatusBadge";
import RunSegmentBar from "./RunSegmentBar";
import RunStatusHeader from "./RunStatusHeader";

interface RunProgressTableProps {
  run: RunProgress;
  selectedEntry: string | null;
  onSelectEntry: (resultDir: string) => void;
  /** シナリオ名 → 説明(行の tooltip と、実行中の見出しに出す) */
  descriptions: ReadonlyMap<string, string>;
}

function describeOptions(run: RunProgress): string {
  const { options } = run;
  const parts = [options.attach ? "attach" : "毎回起動"];
  if (options.tags) parts.push(`タグ: ${options.tags}`);
  if (options.gui) parts.push("GUI");
  if (options.repeat && options.repeat > 1) parts.push(`×${options.repeat}`);
  if (options.remote_stack) parts.push(`実機PC ${options.remote_stack}`);
  return parts.join(" / ");
}

export default function RunProgressTable({
  run,
  selectedEntry,
  onSelectEntry,
  descriptions,
}: RunProgressTableProps) {
  const counts = countProgress(run.entries);

  return (
    <div className="space-y-2">
      <div className="flex flex-wrap items-center gap-x-3 gap-y-1 text-xs text-content">
        <span
          className="font-mono text-content"
          title={`run_id: ${run.run_id}(UTC)`}
        >
          {formatRunTime(run.run_id)}
        </span>
        <EntryStatusBadge status={run.state} />
        <span>{describeOptions(run)}</span>
        <span className="ml-auto">
          {counts.done}/{counts.total} 完了
          <span className="ml-2 text-ok">PASSED {counts.passed}</span>
          <span className="ml-2 text-error">FAILED {counts.failed}</span>
          <span className="ml-2 text-warn">ERROR {counts.error}</span>
        </span>
      </div>
      <RunStatusHeader
        run={run}
        descriptions={descriptions}
        onSelectEntry={onSelectEntry}
      />
      <RunSegmentBar
        segments={segmentsFromEntries(run.entries)}
        onSelect={(index) => onSelectEntry(run.entries[index].result_dir)}
      />
      <table className="w-full text-sm">
        <thead>
          <tr className="text-left text-xs text-muted">
            <th className="py-1 pr-2 font-normal">シナリオ</th>
            <th className="py-1 pr-2 font-normal">結果</th>
            <th className="py-1 pr-2 font-normal">時間</th>
            <th className="py-1 font-normal">メッセージ</th>
          </tr>
        </thead>
        <tbody>
          {run.entries.map((entry, index) => {
            const running = entry.status === "RUNNING";
            return (
              <tr
                key={`${index}-${entry.result_dir}`}
                title={descriptions.get(entry.name) || undefined}
                onClick={() =>
                  entry.status !== "PENDING" && onSelectEntry(entry.result_dir)
                }
                className={`border-t border-line ${
                  entry.status === "PENDING"
                    ? "text-muted"
                    : "cursor-pointer hover:bg-surface-sunken"
                } ${running ? "border-l-4 border-l-accent bg-surface-sunken font-semibold" : ""} ${
                  selectedEntry === entry.result_dir ? "bg-line" : ""
                }`}
              >
                <td className="py-1 pr-2 break-all">
                  {entry.result_dir}
                  {entry.attempt > 0 && (
                    <span className="ml-1 text-[10px] text-warn">
                      再試行 {entry.attempt}
                    </span>
                  )}
                </td>
                <td className="py-1 pr-2">
                  <EntryStatusBadge status={entry.status} />
                </td>
                <td className="py-1 pr-2 whitespace-nowrap tabular-nums text-content">
                  {running ? (
                    <ElapsedText startedAt={entry.started_at} />
                  ) : entry.status === "PENDING" ? (
                    "-"
                  ) : (
                    formatElapsed(entry.elapsed_sec)
                  )}
                </td>
                <td className="py-1 text-xs text-content break-all">
                  {entry.message}
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}
