export interface TabItem {
  id: string;
  label: string;
  /** ラベルの横に出す件数など("2/3")。省略可 */
  badge?: string;
}

interface TabsProps {
  items: TabItem[];
  activeId: string;
  onChange: (id: string) => void;
}

export default function Tabs({ items, activeId, onChange }: TabsProps) {
  return (
    <div role="tablist" className="flex gap-1 rounded-lg bg-surface-sunken p-1">
      {items.map((item) => {
        const active = item.id === activeId;
        return (
          <button
            key={item.id}
            type="button"
            role="tab"
            aria-selected={active}
            onClick={() => onChange(item.id)}
            className={`rounded-md px-2.5 py-1 text-xs font-semibold transition-colors ${
              active
                ? "bg-surface-elevated text-content shadow-card"
                : "text-muted hover:text-content"
            }`}
          >
            {item.label}
            {item.badge && (
              <span className="ml-1 tabular-nums text-muted">{item.badge}</span>
            )}
          </button>
        );
      })}
    </div>
  );
}
