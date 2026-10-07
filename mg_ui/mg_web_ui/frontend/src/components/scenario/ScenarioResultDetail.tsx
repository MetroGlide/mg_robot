import { useCallback, useEffect, useState } from "react";
import type {
  ProgressEntry,
  ScenarioLogName,
  ScenarioResultData,
} from "../../types/scenarioTest";
import { fetchEntryLog, fetchEntryResult } from "../../utils/scenarioTestApi";
import { formatElapsed } from "../../utils/scenarioTest";
import EntryStatusBadge from "./EntryStatusBadge";

const LOG_TAIL_LINES = 300;

interface ScenarioResultDetailProps {
  runId: string;
  entry: ProgressEntry;
  /** 実機PCで実行した run か(stack.log を選べるようにする) */
  hasStackLog: boolean;
}

export default function ScenarioResultDetail({
  runId,
  entry,
  hasStackLog,
}: ScenarioResultDetailProps) {
  const [result, setResult] = useState<ScenarioResultData | null>(null);
  const [logName, setLogName] = useState<ScenarioLogName>("launch");
  const [log, setLog] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const load = useCallback(async () => {
    setLoading(true);
    try {
      const [r, l] = await Promise.all([
        fetchEntryResult(runId, entry.result_dir),
        fetchEntryLog(runId, entry.result_dir, logName, LOG_TAIL_LINES),
      ]);
      setResult(r);
      setLog(l);
    } catch {
      setResult(null);
      setLog(null);
    } finally {
      setLoading(false);
    }
  }, [runId, entry.result_dir, logName]);

  // シナリオを選び直したとき・結果が出たときに取り直す
  useEffect(() => {
    void load();
  }, [load, entry.status]);

  return (
    <div className="space-y-3 text-sm">
      <div className="flex items-center gap-2">
        <span className="font-medium text-content break-all">
          {entry.result_dir}
        </span>
        <EntryStatusBadge status={entry.status} />
        <button
          type="button"
          onClick={() => void load()}
          disabled={loading}
          className="ml-auto text-xs text-content hover:text-content disabled:opacity-50"
        >
          再読み込み
        </button>
      </div>

      {result ? (
        <>
          <p className="text-xs text-muted">
            実時間 {formatElapsed(result.elapsed_wall_sec)} / sim 時間{" "}
            {formatElapsed(result.elapsed_sim_sec)} / ゴール{" "}
            {result.goals.reached}/{result.goals.total}
            {result.goals.failure && ` (${result.goals.failure})`}
          </p>
          <table className="w-full text-xs">
            <tbody>
              {result.checks.map((check) => (
                <tr key={check.name} className="border-t border-line">
                  <td className="py-1 pr-2 align-top">
                    <EntryStatusBadge status={check.status} />
                  </td>
                  <td className="py-1 pr-2 align-top text-content break-all">
                    {check.name}
                  </td>
                  <td className="py-1 text-muted break-all">
                    {check.message}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
          {result.events.length > 0 && (
            <details className="text-xs">
              <summary className="cursor-pointer text-muted">
                イベント ({result.events.length})
              </summary>
              <pre className="mt-1 max-h-48 overflow-auto rounded bg-surface-sunken p-2 text-content">
                {result.events.map((e) => JSON.stringify(e)).join("\n")}
              </pre>
            </details>
          )}
        </>
      ) : (
        <p className="text-xs text-muted">
          {entry.status === "RUNNING"
            ? "実行中です。結果は終了後に表示されます。"
            : "結果がありません(起動に失敗した可能性があります。ログを確認してください)。"}
        </p>
      )}

      <div className="space-y-1">
        <div className="flex items-center gap-3 text-xs">
          <span className="text-muted">ログ(末尾 {LOG_TAIL_LINES} 行)</span>
          {(["launch", "stack"] as const)
            .filter((name) => name === "launch" || hasStackLog)
            .map((name) => (
              <button
                key={name}
                type="button"
                onClick={() => setLogName(name)}
                className={
                  logName === name
                    ? "text-accent"
                    : "text-muted hover:text-content"
                }
              >
                {name}.log
              </button>
            ))}
        </div>
        <pre className="max-h-96 overflow-auto rounded bg-surface-sunken p-2 font-mono text-[11px] text-content whitespace-pre-wrap break-all">
          {log ?? "ログがありません"}
        </pre>
      </div>
    </div>
  );
}
