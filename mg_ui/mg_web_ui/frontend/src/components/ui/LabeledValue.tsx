export type LabeledValueVariant =
  | "normal"
  | "success"
  | "warn"
  | "error"
  | "muted";

const VALUE_CLASS: Record<LabeledValueVariant, string> = {
  normal: "text-content",
  success: "text-ok",
  warn: "text-warn",
  error: "text-error",
  muted: "text-muted",
};

interface LabeledValueProps {
  label: string;
  value: string;
  variant?: LabeledValueVariant;
  bold?: boolean;
}

export default function LabeledValue({
  label,
  value,
  variant = "normal",
  bold = false,
}: LabeledValueProps) {
  return (
    <div>
      <span className="block text-xs text-muted">{label}</span>
      <p className={`${VALUE_CLASS[variant]} ${bold ? "font-semibold" : ""}`}>
        {value}
      </p>
    </div>
  );
}
