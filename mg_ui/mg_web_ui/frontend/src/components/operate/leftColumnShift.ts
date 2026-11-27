/** 列や部品の矩形 (画面座標)。DOMRect の必要な成分だけ */
export interface ColumnRect {
  top: number;
  bottom: number;
  left: number;
  right: number;
}

/**
 * 左下の列を右へずらす量 (px)。
 * 左上の列の部品 (KPI の行・操作パネルなど) のうち、左下の列と縦に重なる (間隔が gap 未満の) ものの
 * 右端の gap 先まで動かす。重なる部品がなければ 0。横長の KPI の行が重ならなければ、その幅ではずらさない。
 * 左下の列は下端で固定しているので、ずらしても上端は変わらず、判定は振動しない。
 */
export function leftColumnShift(
  topLeftItems: ColumnRect[],
  columnLeft: number,
  bottomLeft: ColumnRect | null,
  gap: number,
): number {
  if (!bottomLeft || bottomLeft.bottom - bottomLeft.top <= 0) return 0;
  const overlapping = topLeftItems.filter(
    (r) =>
      r.bottom - r.top > 0 &&
      r.bottom + gap > bottomLeft.top &&
      r.top < bottomLeft.bottom,
  );
  if (overlapping.length === 0) return 0;
  const right = Math.max(...overlapping.map((r) => r.right));
  return Math.max(0, right + gap - columnLeft);
}
