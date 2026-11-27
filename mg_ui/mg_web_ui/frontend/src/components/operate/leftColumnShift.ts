/** 列の矩形 (画面座標)。DOMRect の必要な成分だけ */
export interface ColumnRect {
  top: number;
  bottom: number;
  left: number;
  right: number;
}

/**
 * 左下の列を右へずらす量 (px)。左上の列と縦に重なる (間隔が gap 未満) ときだけ、
 * 左上の列の右端の gap 先まで動かす。重ならなければ 0。
 * 左下の列は下端で固定しているので、ずらしても上端は変わらず、判定は振動しない。
 */
export function leftColumnShift(
  topLeft: ColumnRect | null,
  bottomLeft: ColumnRect | null,
  gap: number,
): number {
  if (!topLeft || !bottomLeft) return 0;
  if (topLeft.bottom - topLeft.top <= 0 || bottomLeft.bottom - bottomLeft.top <= 0) {
    return 0;
  }
  if (topLeft.bottom + gap <= bottomLeft.top) return 0;
  return Math.max(0, topLeft.right + gap - topLeft.left);
}
