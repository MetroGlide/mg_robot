import { CSSProperties, ReactNode, useLayoutEffect, useRef, useState } from "react";
import { usePanelOpacity } from "../../hooks/usePanelOpacity";
import { leftColumnShift } from "./leftColumnShift";

/** 左の 2 つの列の間に空ける間隔 (px)。ずらすときの横の間隔にも使う */
const LEFT_COLUMN_GAP = 8;

/**
 * 列を縦に積んだ部品 (KPI の行・操作パネルなど) を返す。
 * 子が 1 つだけの包み (flex の列など) は降りて、縦に並んだ子を部品とみなす。
 */
function columnItems(column: HTMLElement): Element[] {
  let el: Element = column;
  while (el.children.length === 1) el = el.children[0];
  return el.children.length > 0 ? Array.from(el.children) : [el];
}

/**
 * 左上と左下の列が縦に重なるとき、左下の列を、重なる部品 (操作パネル) のすぐ右へずらす量 (px) を返す。
 * パネルの開閉 (部品の大きさ) と画面の大きさの変化で測り直す。
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
    const observer = new ResizeObserver(() => measure());
    function measure() {
      const items = columnItems(top!);
      // 開閉で入れ替わった部品も、幅の変化を拾えるよう監視に足す (監視済みなら何もしない)
      items.forEach((item) => observer.observe(item));
      setShift(
        leftColumnShift(
          items.map((item) => item.getBoundingClientRect()),
          top!.getBoundingClientRect().left,
          bottom!.getBoundingClientRect(),
          LEFT_COLUMN_GAP,
        ),
      );
    }
    observer.observe(root);
    observer.observe(top);
    observer.observe(bottom);
    measure();
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
 * スロットの枠はポインタを受け取らず、カード(Card)などの部品の根だけが受け取る。
 * 枠の矩形は列の最も広い部品の幅 × 列の高さになるので、枠で受け取ると部品の脇の地図が操作できなくなる。
 * スロットに Card 以外を直接置くときは、その部品の根に pointer-events-auto を付ける。
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
        <div ref={topLeftRef} className="absolute left-3 top-3">
          {topLeft}
        </div>
        <div className="absolute right-3 top-1/2 z-10 -translate-y-1/2">
          {toolbar}
        </div>
        <div className="absolute right-16 top-3">{topRight}</div>
        <div className="absolute left-1/2 top-3 -translate-x-1/2">
          {notice}
        </div>
        <div
          ref={bottomLeftRef}
          className="absolute bottom-3 left-3"
          style={shift > 0 ? { transform: `translateX(${shift}px)` } : undefined}
        >
          {bottomLeft}
        </div>
        <div className="absolute bottom-3 right-16">{bottomRight}</div>
      </div>
    </div>
  );
}
