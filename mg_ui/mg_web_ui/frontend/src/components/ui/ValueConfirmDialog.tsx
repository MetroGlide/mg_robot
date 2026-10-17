interface ValueConfirmDialogProps {
  open: boolean;
  title: string;
  values: { label: string; value: string }[];
  onConfirm: () => void;
  onCancel: () => void;
}

export default function ValueConfirmDialog({
  open,
  title,
  values,
  onConfirm,
  onCancel,
}: ValueConfirmDialogProps) {
  if (!open) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center">
      <div
        className="absolute inset-0 bg-black/60 backdrop-blur-sm"
        onClick={onCancel}
      />
      <div className="relative z-10 w-full max-w-lg rounded-xl border border-line bg-surface-elevated text-content shadow-card">
        <div className="border-b border-line px-5 py-4">
          <span className="text-sm font-medium">{title}</span>
        </div>
        <div className="px-5 py-4 space-y-3">
          {values.map(({ label, value }) => (
            <div key={label}>
              <p className="mb-1 text-xs text-muted">{label}</p>
              <p className="break-all rounded bg-surface-sunken px-3 py-2 text-sm">
                {value || <span className="italic text-muted">（空）</span>}
              </p>
            </div>
          ))}
        </div>
        <div className="flex justify-end gap-2 border-t border-line px-5 py-3">
          <button
            onClick={onCancel}
            className="rounded bg-surface-sunken px-4 py-2 text-sm font-medium hover:bg-line"
          >
            Cancel
          </button>
          <button
            onClick={onConfirm}
            className="rounded bg-accent px-4 py-2 text-sm font-medium text-surface-elevated hover:opacity-90"
          >
            OK
          </button>
        </div>
      </div>
    </div>
  );
}
