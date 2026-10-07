import type { RunSummary } from "../../types/scenarioTest";
import EntryStatusBadge from "./EntryStatusBadge";

interface RunHistoryListProps {
  runs: readonly RunSummary[];
  selectedRunId: string | null;
  onSelect: (runId: string) => void;
}

export default function RunHistoryList({
  runs,
  selectedRunId,
  onSelect,
}: RunHistoryListProps) {
  if (runs.length === 0) {
    return <p className="text-xs text-muted">実行結果はまだありません</p>;
  }
  return (
    <ul className="max-h-72 overflow-y-auto divide-y divide-line text-sm">
      {runs.map((run) => (
        <li key={run.run_id}>
          <button
            type="button"
            onClick={() => onSelect(run.run_id)}
            className={`flex w-full flex-wrap items-center gap-2 px-2 py-1.5 text-left hover:bg-surface-sunken ${
              selectedRunId === run.run_id ? "bg-line" : ""
            }`}
          >
            <span className="font-mono text-content">{run.run_id}</span>
            <EntryStatusBadge status={run.state} />
            {run.options.attach && (
              <span className="text-[10px] text-muted">attach</span>
            )}
            {run.options.remote_stack && (
              <span className="text-[10px] text-muted">実機PC</span>
            )}
            <span className="ml-auto text-xs">
              <span className="text-ok">{run.counts.PASSED ?? 0}</span>
              {" / "}
              <span className="text-error">{run.counts.FAILED ?? 0}</span>
              {" / "}
              <span className="text-warn">{run.counts.ERROR ?? 0}</span>
              <span className="text-muted"> ({run.total})</span>
            </span>
          </button>
        </li>
      ))}
    </ul>
  );
}
