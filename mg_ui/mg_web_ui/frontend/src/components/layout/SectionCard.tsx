import { ReactNode } from "react";

interface SectionCardProps {
  title?: string;
  children: ReactNode;
  className?: string;
}

export default function SectionCard({
  title,
  children,
  className = "",
}: SectionCardProps) {
  return (
    <div
      className={`rounded-md border border-line bg-surface-sunken p-3 ${className}`}
    >
      {title && (
        <p className="mb-2 text-xs font-medium text-muted">{title}</p>
      )}
      {children}
    </div>
  );
}
