import { ReactNode } from "react";

interface CardProps {
  children: ReactNode;
  className?: string;
  /** true なら透明度の設定に従わず、常に(ほぼ)不透明にする。操作の部品や警告のカード用 */
  solid?: boolean;
}

/**
 * 地図の上に重ねるカード。
 * WebGL キャンバスの上では backdrop-filter(ぼかし)を使わない。キャンバスの更新のたびに再合成が走り負荷が上がるため、
 * 半透明の単色と影で質感を出す。背景の不透明度は運用ビューの設定(--panel-alpha)に従う(index.css の .panel-surface)。
 */
export default function Card({ children, className = "", solid = false }: CardProps) {
  const background = solid ? "bg-surface-elevated/95" : "panel-surface";
  return (
    <div
      className={`rounded-xl border border-line ${background} text-content shadow-card ${className}`}
    >
      {children}
    </div>
  );
}
