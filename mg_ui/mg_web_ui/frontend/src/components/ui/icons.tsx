import { ReactNode } from "react";

// アイコンはインライン SVG で持つ(外部のアイコン集に依存しない)。使うものだけを足していく。

function Icon({ children, size = 20 }: { children: ReactNode; size?: number }) {
  return (
    <svg
      width={size}
      height={size}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth={2}
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      {children}
    </svg>
  );
}

export const PlusIcon = () => (
  <Icon>
    <path d="M12 5v14M5 12h14" />
  </Icon>
);
export const MinusIcon = () => (
  <Icon>
    <path d="M5 12h14" />
  </Icon>
);
export const HomeIcon = () => (
  <Icon>
    <path d="M3 11l9-8 9 8M5 10v10h14V10" />
  </Icon>
);
export const RotateIcon = () => (
  <Icon>
    <path d="M21 12a9 9 0 1 1-3-6.7M21 3v6h-6" />
  </Icon>
);
export const CrosshairIcon = () => (
  <Icon>
    <circle cx="12" cy="12" r="7" />
    <path d="M12 2v4M12 18v4M2 12h4M18 12h4" />
  </Icon>
);
export const CloseIcon = () => (
  <Icon size={16}>
    <path d="M6 6l12 12M18 6L6 18" />
  </Icon>
);
export const SunIcon = () => (
  <Icon>
    <circle cx="12" cy="12" r="4" />
    <path d="M12 2v2M12 20v2M2 12h2M20 12h2M5 5l1.5 1.5M17.5 17.5L19 19M5 19l1.5-1.5M17.5 6.5L19 5" />
  </Icon>
);
export const MoonIcon = () => (
  <Icon>
    <path d="M21 13A9 9 0 1 1 11 3a7 7 0 0 0 10 10z" />
  </Icon>
);
export const CpuIcon = () => (
  <Icon>
    <rect x="6" y="6" width="12" height="12" rx="1" />
    <path d="M9 2v4M15 2v4M9 18v4M15 18v4M2 9h4M2 15h4M18 9h4M18 15h4" />
  </Icon>
);
export const SatelliteIcon = () => (
  <Icon>
    <circle cx="12" cy="12" r="3" />
    <path d="M12 3a9 9 0 0 1 9 9M12 7a5 5 0 0 1 5 5" />
  </Icon>
);
