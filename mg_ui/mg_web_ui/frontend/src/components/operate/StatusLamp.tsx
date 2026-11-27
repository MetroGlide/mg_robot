import { useEffect, useState } from "react";
import { LAMP_COLORS, LampSpec, lampLabel } from "./statusLamp";

/**
 * 状態ランプ。色と点滅は statusLamp.ts の表から渡す。
 * 点滅は情報 (止まっているか) なので、低負荷モードでも止めないよう CSS のアニメーションではなくタイマーで切り替える。
 */
export default function StatusLamp({
  lamp,
  size = 16,
  className = "",
}: {
  lamp: LampSpec;
  size?: number;
  className?: string;
}) {
  const [lit, setLit] = useState(true);
  const { blinkMs, color } = lamp;

  useEffect(() => {
    setLit(true);
    if (blinkMs === null || color === "off") return;
    const timer = setInterval(() => setLit((v) => !v), blinkMs / 2);
    return () => clearInterval(timer);
  }, [blinkMs, color]);

  const off = color === "off" || !lit;
  return (
    <span
      role="img"
      aria-label={lampLabel(lamp)}
      title={lampLabel(lamp)}
      className={`inline-block shrink-0 rounded-full border border-line ${className}`}
      style={{
        width: size,
        height: size,
        backgroundColor: off ? "transparent" : LAMP_COLORS[color],
        boxShadow: off ? "none" : `0 0 ${size / 2}px ${LAMP_COLORS[color]}`,
      }}
    />
  );
}
