import { ReactNode } from "react";

export type Tone = "ok" | "warn" | "error" | "neutral";

export const TONE_TEXT: Record<Tone, string> = {
  ok: "text-ok",
  warn: "text-warn",
  error: "text-error",
  neutral: "text-muted",
};

export const TONE_BG: Record<Tone, string> = {
  ok: "bg-ok",
  warn: "bg-warn",
  error: "bg-error",
  neutral: "bg-muted",
};

/** /diagnostics の level(0 OK / 1 WARN / 2 ERROR / 3 STALE)を色に対応させる */
export function toneFromDiagLevel(level: number): Tone {
  if (level === 0) return "ok";
  if (level === 1) return "warn";
  if (level === 2) return "error";
  return "neutral";
}

interface PillProps {
  tone?: Tone;
  children: ReactNode;
  title?: string;
}

export default function Pill({ tone = "neutral", children, title }: PillProps) {
  return (
    <span
      title={title}
      className={`inline-flex items-center gap-1.5 rounded-full border border-line bg-surface-elevated px-2.5 py-0.5 text-xs font-semibold ${TONE_TEXT[tone]}`}
    >
      <span className={`h-2 w-2 rounded-full ${TONE_BG[tone]}`} />
      {children}
    </span>
  );
}
