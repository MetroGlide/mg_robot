const STATUS_CLASS: Record<string, string> = {
  PENDING: "bg-gray-600 text-gray-200",
  RUNNING: "bg-blue-600 text-white animate-pulse",
  PASSED: "bg-green-600 text-white",
  FAILED: "bg-red-600 text-white",
  ERROR: "bg-yellow-600 text-white",
  running: "bg-blue-600 text-white animate-pulse",
  finished: "bg-gray-600 text-gray-200",
  aborted: "bg-orange-700 text-white",
};

/** シナリオの結果(PASSED など)や run の状態(running など)を表すラベル */
export default function EntryStatusBadge({ status }: { status: string }) {
  const cls = STATUS_CLASS[status] ?? "bg-gray-600 text-gray-200";
  return (
    <span
      className={`inline-block rounded px-1.5 py-0.5 text-[10px] font-bold tracking-wide ${cls}`}
    >
      {status}
    </span>
  );
}
