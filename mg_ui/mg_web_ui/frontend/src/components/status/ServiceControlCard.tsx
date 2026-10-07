import SectionCard from "../layout/SectionCard";

export interface ServiceButton {
  label: string;
  onClick: () => void;
  variant?: "green" | "red" | "blue" | "gray";
}

interface ServiceControlCardProps {
  title: string;
  buttons: ServiceButton[];
  loading?: boolean;
  error?: string | null;
}

const VARIANT_CLASS: Record<NonNullable<ServiceButton["variant"]>, string> = {
  green: "bg-ok text-surface-elevated hover:opacity-90",
  red: "bg-error text-surface-elevated hover:opacity-90",
  blue: "bg-accent text-surface-elevated hover:opacity-90",
  gray: "bg-surface-sunken text-content hover:bg-line",
};

export default function ServiceControlCard({
  title,
  buttons,
  loading = false,
  error,
}: ServiceControlCardProps) {
  return (
    <SectionCard title={title}>
      <div className="flex flex-wrap gap-3">
        {buttons.map((btn) => (
          <button
            key={btn.label}
            onClick={btn.onClick}
            disabled={loading}
            className={`${VARIANT_CLASS[btn.variant ?? "gray"]} disabled:opacity-50 px-4 py-2 rounded font-medium text-sm`}
          >
            {btn.label}
          </button>
        ))}
      </div>
      {error && <p className="mt-2 text-sm text-error">{error}</p>}
    </SectionCard>
  );
}
