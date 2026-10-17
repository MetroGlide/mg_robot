import { Tone, TONE_BG } from "./Pill";

interface HealthDotProps {
  tone: Tone;
  title?: string;
}

export default function HealthDot({ tone, title }: HealthDotProps) {
  return (
    <span
      title={title}
      className={`inline-block h-2.5 w-2.5 flex-shrink-0 rounded-full ${TONE_BG[tone]}`}
    />
  );
}
