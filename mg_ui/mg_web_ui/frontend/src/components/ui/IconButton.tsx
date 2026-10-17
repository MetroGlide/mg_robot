import { ReactNode } from "react";

interface IconButtonProps {
  icon: ReactNode;
  title: string;
  onClick: () => void;
  active?: boolean;
  disabled?: boolean;
}

export default function IconButton({
  icon,
  title,
  onClick,
  active = false,
  disabled = false,
}: IconButtonProps) {
  return (
    <button
      type="button"
      title={title}
      aria-label={title}
      aria-pressed={active}
      disabled={disabled}
      onClick={onClick}
      className={`flex h-9 w-9 items-center justify-center rounded-lg transition-colors disabled:cursor-not-allowed disabled:opacity-40 ${
        active
          ? "bg-accent text-surface-elevated"
          : "text-content hover:bg-surface-sunken"
      }`}
    >
      {icon}
    </button>
  );
}
