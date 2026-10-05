import SettingPage from "../pages/SettingPage";

interface SettingModalProps {
  open: boolean;
  onClose: () => void;
}

export default function SettingModal({ open, onClose }: SettingModalProps) {
  if (!open) return null;

  return (
    <div
      className="fixed inset-0 z-50 flex items-center justify-center"
      onMouseDown={(e) => {
        if (e.target === e.currentTarget) onClose();
      }}
    >
      <div
        className="absolute inset-0 bg-black/60"
        onClick={onClose}
      />
      <div className="relative z-10 rounded-xl border border-line bg-surface-elevated text-content shadow-card w-full max-w-2xl max-h-[80vh] flex flex-col">
        <div className="flex items-center justify-between px-4 py-3 border-b border-line flex-shrink-0">
          <span className="text-sm font-semibold">Settings</span>
          <div className="flex items-center gap-1">
            <button
              onClick={onClose}
              className="p-1.5 text-muted hover:text-content rounded transition-colors"
              title="閉じる"
            >
              <svg
                className="w-4 h-4"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M6 18L18 6M6 6l12 12"
                />
              </svg>
            </button>
          </div>
        </div>
        <div className="flex-1 overflow-hidden">
          <SettingPage />
        </div>
      </div>
    </div>
  );
}
