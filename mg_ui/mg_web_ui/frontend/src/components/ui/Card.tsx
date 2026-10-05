import { ReactNode } from "react";

interface CardProps {
  children: ReactNode;
  className?: string;
}

/**
 * 地図の上に重ねるカード。
 * WebGL キャンバスの上では backdrop-filter(ぼかし)を使わない。キャンバスの更新のたびに再合成が走り負荷が上がるため、
 * 半透明の単色と影で質感を出す。
 */
export default function Card({ children, className = "" }: CardProps) {
  return (
    <div
      className={`rounded-xl border border-line bg-surface-elevated/95 text-content shadow-card ${className}`}
    >
      {children}
    </div>
  );
}
