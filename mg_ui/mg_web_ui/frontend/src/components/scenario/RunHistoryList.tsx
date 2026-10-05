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
    return <p className="text-xs text-gray-500">実行結果はまだありません</p>;
  }
  return (
    <ul className="max-h-72 overflow-y-auto divide-y divide-gray-700 text-sm">
      {runs.map((run) => (
        <li key={run.run_id}>
          <button
            type="button"
            onClick={() => onSelect(run.run_id)}
            className={`flex w-full flex-wrap items-center gap-2 px-2 py-1.5 text-left hover:bg-gray-700/50 ${
              selectedRunId === run.run_id ? "bg-gray-700" : ""
            }`}
          >
            <span className="font-mono text-gray-100">{run.run_id}</span>
            <EntryStatusBadge status={run.state} />
            {run.options.attach && (
              <span className="text-[10px] text-gray-400">attach</span>
            )}
            {run.options.remote_stack && (
              <span className="text-[10px] text-gray-400">実機PC</span>
            )}
            <span className="ml-auto text-xs">
              <span className="text-green-400">{run.counts.PASSED ?? 0}</span>
              {" / "}
              <span className="text-red-400">{run.counts.FAILED ?? 0}</span>
              {" / "}
              <span className="text-yellow-400">{run.counts.ERROR ?? 0}</span>
              <span className="text-gray-400"> ({run.total})</span>
            </span>
          </button>
        </li>
      ))}
    </ul>
  );
}
