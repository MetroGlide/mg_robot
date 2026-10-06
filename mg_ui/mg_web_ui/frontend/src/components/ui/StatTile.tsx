import { ReactNode } from "react";
import { Tone, TONE_TEXT } from "./Pill";

interface StatTileProps {
  icon?: ReactNode;
  label: string;
  value: string;
  unit?: string;
  tone?: Tone;
  /** 値が古いとき(鮮度切れ)に true。値を灰色にし、経過秒を添える */
  stale?: boolean;
  ageSec?: number | null;
  /** 幅(Tailwind の w-*)。値の桁が変わってもカードの大きさが変わらないよう、呼び出し側で固定する */
  widthClass: string;
}

export default function StatTile({
  icon,
  label,
  value,
  unit,
  tone = "neutral",
  stale = false,
  ageSec = null,
  widthClass,
}: StatTileProps) {
  const valueClass = stale ? "text-muted" : "text-content";
  return (
    <div className={`flex shrink-0 items-center gap-3 px-3 py-2 ${widthClass}`}>
      {icon && (
        <div className={`flex-shrink-0 ${stale ? "text-muted" : TONE_TEXT[tone]}`}>
          {icon}
        </div>
      )}
      <div className="min-w-0">
        <div className="truncate text-xs text-muted">{label}</div>
        <div className={`truncate text-lg font-bold tabular-nums leading-tight ${valueClass}`}>
          {stale ? "--" : value}
          {!stale && unit && (
            <span className="ml-1 text-xs font-normal text-muted">{unit}</span>
          )}
        </div>
        {stale && ageSec !== null && (
          <div className="truncate text-[10px] text-warn">{Math.floor(ageSec)}秒 更新なし</div>
        )}
      </div>
    </div>
  );
}
