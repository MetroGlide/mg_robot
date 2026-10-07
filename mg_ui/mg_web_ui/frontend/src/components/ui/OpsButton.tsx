import { ReactNode } from "react";

export type OpsButtonTone = "primary" | "danger" | "warn" | "neutral";

const TONE_CLASS: Record<OpsButtonTone, string> = {
  primary: "bg-accent text-surface-elevated hover:opacity-90",
  danger: "bg-error text-surface-elevated hover:opacity-90",
  warn: "bg-warn text-surface-elevated hover:opacity-90",
  neutral: "bg-surface-sunken text-content hover:bg-line",
};

interface OpsButtonProps {
  children: ReactNode;
  onClick: () => void;
  tone?: OpsButtonTone;
  disabled?: boolean;
  title?: string;
}

/** 新 UI の操作ボタン。タッチパッドで押しやすいよう、高さを確保する */
export default function OpsButton({
  children,
  onClick,
  tone = "neutral",
  disabled = false,
  title,
}: OpsButtonProps) {
  return (
    <button
      type="button"
      title={title}
      disabled={disabled}
      onClick={onClick}
      className={`h-10 rounded-lg px-4 text-sm font-semibold transition-colors disabled:cursor-not-allowed disabled:opacity-40 ${TONE_CLASS[tone]}`}
    >
      {children}
    </button>
  );
}
