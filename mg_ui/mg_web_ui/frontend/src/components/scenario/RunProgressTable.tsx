import type { RunProgress } from "../../types/scenarioTest";
import { countProgress, formatElapsed } from "../../utils/scenarioTest";
import EntryStatusBadge from "./EntryStatusBadge";

interface RunProgressTableProps {
  run: RunProgress;
  selectedEntry: string | null;
  onSelectEntry: (resultDir: string) => void;
}

function describeOptions(run: RunProgress): string {
  const { options } = run;
  const parts = [options.attach ? "attach" : "毎回起動"];
  if (options.gui) parts.push("GUI");
  if (options.repeat && options.repeat > 1) parts.push(`×${options.repeat}`);
  if (options.remote_stack) parts.push(`実機PC ${options.remote_stack}`);
  return parts.join(" / ");
}

export default function RunProgressTable({
  run,
  selectedEntry,
  onSelectEntry,
}: RunProgressTableProps) {
  const counts = countProgress(run.entries);
  const percent = counts.total ? (counts.done / counts.total) * 100 : 0;

  return (
    <div className="space-y-2">
      <div className="flex flex-wrap items-center gap-x-3 gap-y-1 text-xs text-content">
        <span className="font-mono text-content">{run.run_id}</span>
        <EntryStatusBadge status={run.state} />
        <span>{describeOptions(run)}</span>
        <span className="ml-auto">
          {counts.done}/{counts.total} 完了
          <span className="ml-2 text-ok">PASSED {counts.passed}</span>
          <span className="ml-2 text-error">FAILED {counts.failed}</span>
          <span className="ml-2 text-warn">ERROR {counts.error}</span>
        </span>
      </div>
      <div className="h-1.5 w-full rounded bg-surface-sunken">
        <div
          className="h-1.5 rounded bg-accent transition-all"
          style={{ width: `${percent}%` }}
        />
      </div>
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
          {run.entries.map((entry, index) => (
            <tr
              key={`${index}-${entry.result_dir}`}
              onClick={() =>
                entry.status !== "PENDING" && onSelectEntry(entry.result_dir)
              }
              className={`border-t border-line ${
                entry.status === "PENDING"
                  ? "text-muted"
                  : "cursor-pointer hover:bg-surface-sunken"
              } ${selectedEntry === entry.result_dir ? "bg-line" : ""}`}
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
              <td className="py-1 pr-2 whitespace-nowrap text-content">
                {entry.status === "PENDING" || entry.status === "RUNNING"
                  ? "-"
                  : formatElapsed(entry.elapsed_sec)}
              </td>
              <td className="py-1 text-xs text-content break-all">
                {entry.message}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
