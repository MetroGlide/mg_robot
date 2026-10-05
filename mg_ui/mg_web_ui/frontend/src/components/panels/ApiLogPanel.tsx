import { ApiLog } from "../../hooks/useSystemManagerClient";

export default function ApiLogPanel({ logs }: { logs: ApiLog[] }) {
  if (logs.length === 0) return null;

  return (
    <section className="space-y-2 rounded-lg bg-surface-sunken p-4">
      <p className="text-xs text-muted">API Log</p>
      <ul className="space-y-1 max-h-48 overflow-y-auto font-mono text-xs">
        {logs.map((log) => (
          <li key={log.id} className="flex gap-2 items-start">
            <span className="shrink-0 text-muted">{log.timestamp}</span>
            <span
              className={`shrink-0 font-bold ${log.success ? "text-ok" : "text-error"}`}
            >
              {log.success ? "OK" : "ERR"}
            </span>
            <span className="shrink-0">{log.path}</span>
            {log.message && (
              <span className="break-all text-muted">{log.message}</span>
            )}
          </li>
        ))}
      </ul>
    </section>
  );
}
