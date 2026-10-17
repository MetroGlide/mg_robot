const STATUS_CLASS: Record<string, string> = {
  PENDING: "bg-muted text-surface-elevated",
  RUNNING: "bg-accent text-surface-elevated animate-pulse",
  PASSED: "bg-ok text-surface-elevated",
  FAILED: "bg-error text-surface-elevated",
  ERROR: "bg-warn text-surface-elevated",
  running: "bg-accent text-surface-elevated animate-pulse",
  finished: "bg-muted text-surface-elevated",
  aborted: "bg-warn text-surface-elevated",
};

/** シナリオの結果(PASSED など)や run の状態(running など)を表すラベル */
export default function EntryStatusBadge({ status }: { status: string }) {
  const cls = STATUS_CLASS[status] ?? "bg-muted text-surface-elevated";
  return (
    <span
      className={`inline-block rounded px-1.5 py-0.5 text-[10px] font-bold tracking-wide ${cls}`}
    >
      {status}
    </span>
  );
}
