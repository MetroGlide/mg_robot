import { Tone, TONE_BG, TONE_TEXT } from "./Pill";

export interface Step {
  id: string;
  label: string;
}

interface StepperProps {
  steps: Step[];
  /** 現在の段。steps に含まれない場合はどの段も進行中にしない */
  currentId: string | null;
  /** 現在の段の色。SUSPENDED は warn、ERROR は error のように状態で変える */
  tone?: Tone;
}

/** 横一列の進捗表示。現在の段より前は完了、後は未到達として描く */
export default function Stepper({ steps, currentId, tone = "ok" }: StepperProps) {
  const currentIndex = steps.findIndex((s) => s.id === currentId);
  return (
    <ol className="flex items-start">
      {steps.map((step, i) => {
        const done = currentIndex >= 0 && i < currentIndex;
        const current = i === currentIndex;
        const dot = current
          ? TONE_BG[tone]
          : done
            ? "bg-accent"
            : "bg-surface-sunken border border-line";
        const label = current ? TONE_TEXT[tone] : done ? "text-content" : "text-muted";
        return (
          <li key={step.id} className="flex flex-1 flex-col items-center gap-1">
            <div className="flex w-full items-center">
              <span className={`h-0.5 flex-1 ${i === 0 ? "opacity-0" : done || current ? "bg-accent" : "bg-line"}`} />
              <span
                className={`h-3.5 w-3.5 rounded-full ${dot} ${current ? "ring-4 ring-accent/20" : ""}`}
              />
              <span className={`h-0.5 flex-1 ${i === steps.length - 1 ? "opacity-0" : done ? "bg-accent" : "bg-line"}`} />
            </div>
            <span className={`text-[11px] font-semibold ${label}`}>{step.label}</span>
          </li>
        );
      })}
    </ol>
  );
}
