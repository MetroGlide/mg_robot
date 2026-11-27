import { CSSProperties, ReactNode, useLayoutEffect, useRef, useState } from "react";
import { usePanelOpacity } from "../../hooks/usePanelOpacity";
import { leftColumnShift } from "./leftColumnShift";

/** 左の 2 つの列の間に空ける間隔 (px)。ずらすときの横の間隔にも使う */
const LEFT_COLUMN_GAP = 8;

/**
 * 左上と左下の列が縦に重なるとき、左下の列を左上の列の右へずらす量 (px) を返す。
 * パネルの開閉 (列の大きさ) と画面の大きさの変化で測り直す。
 */
function useLeftColumnShift() {
  const rootRef = useRef<HTMLDivElement>(null);
  const topLeftRef = useRef<HTMLDivElement>(null);
  const bottomLeftRef = useRef<HTMLDivElement>(null);
  const [shift, setShift] = useState(0);

  useLayoutEffect(() => {
    const root = rootRef.current;
    const top = topLeftRef.current;
    const bottom = bottomLeftRef.current;
    if (!root || !top || !bottom) return;
    const measure = () =>
      setShift(
        leftColumnShift(
          top.getBoundingClientRect(),
          bottom.getBoundingClientRect(),
          LEFT_COLUMN_GAP,
        ),
      );
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(root);
    observer.observe(top);
    observer.observe(bottom);
    return () => observer.disconnect();
  }, []);

  return { rootRef, topLeftRef, bottomLeftRef, shift };
}

interface Props {
  /** 全面に描く地図 */
  map: ReactNode;
  topLeft?: ReactNode;
  /** 右端の縦のツールバー(縦中央) */
  toolbar?: ReactNode;
  /** ツールバーの左に出す、右上のパネルの列(閉じたパネルは、アイコンになって同じ列に残る) */
  topRight?: ReactNode;
  /** 上中央に出す通知 */
  notice?: ReactNode;
  bottomLeft?: ReactNode;
  /** ツールバーの左に出す、右下のパネルの列 */
  bottomRight?: ReactNode;
}

/**
 * 運用ビューの共通レイアウト。地図を全面に置き、各スロットの部品を重ねる。
 * 重ねる部品は、操作できる部分だけがポインタを受け取る(地図の操作を妨げない)。
 * パネルの背景の不透明度(--panel-alpha)は、ここで与える。運用ビューの外のカードには影響しない。
 * 画面の高さが足りず左上と左下の列が重なるときは、左下の列を左上の列の右へずらして横に並べる。
 */
export default function OperateLayout({
  map,
  topLeft,
  toolbar,
  topRight,
  notice,
  bottomLeft,
  bottomRight,
}: Props) {
  const [opacity] = usePanelOpacity();
  const { rootRef, topLeftRef, bottomLeftRef, shift } = useLeftColumnShift();
  const style = { "--panel-alpha": opacity } as CSSProperties;
  return (
    <div
      ref={rootRef}
      className="ops-overlay relative h-full w-full overflow-hidden"
      style={style}
    >
      <div className="absolute inset-0">{map}</div>
      <div className="pointer-events-none absolute inset-0">
        <div ref={topLeftRef} className="pointer-events-auto absolute left-3 top-3">
          {topLeft}
        </div>
        <div className="pointer-events-auto absolute right-3 top-1/2 z-10 -translate-y-1/2">
          {toolbar}
        </div>
        <div className="pointer-events-auto absolute right-16 top-3">{topRight}</div>
        <div className="pointer-events-auto absolute left-1/2 top-3 -translate-x-1/2">
          {notice}
        </div>
        <div
          ref={bottomLeftRef}
          className="pointer-events-auto absolute bottom-3 left-3"
          style={shift > 0 ? { transform: `translateX(${shift}px)` } : undefined}
        >
          {bottomLeft}
        </div>
        <div className="pointer-events-auto absolute bottom-3 right-16">{bottomRight}</div>
      </div>
    </div>
  );
}
