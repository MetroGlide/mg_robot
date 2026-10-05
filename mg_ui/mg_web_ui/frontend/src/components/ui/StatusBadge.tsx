export type ContainerStatus = "running" | "exited" | "dead" | "unknown";

const DOT_CLASS: Record<string, string> = {
  running: "bg-ok",
  exited: "bg-error",
  dead: "bg-error",
  unknown: "bg-muted",
};

interface StatusBadgeProps {
  status: string;
}

export default function StatusBadge({ status }: StatusBadgeProps) {
  const dot = DOT_CLASS[status] ?? "bg-muted";
  return (
    <div className="flex items-center gap-2">
      <span className={`w-3 h-3 rounded-full flex-shrink-0 ${dot}`} />
      <span className="text-sm font-semibold capitalize">{status}</span>
    </div>
  );
}
