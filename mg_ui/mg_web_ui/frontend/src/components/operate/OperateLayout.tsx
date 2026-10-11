import { CSSProperties, ReactNode } from "react";
import { usePanelOpacity } from "../../hooks/usePanelOpacity";

interface Props {
  /** 全面に描く地図 */
  map: ReactNode;
  topLeft?: ReactNode;
  /** 右端の縦のツールバー(残りの高さの中央) */
  toolbar?: ReactNode;
  /** 右端の列の上に置く、パネルの表示切り替え */
  panelBar?: ReactNode;
  /** ツールバーの左に出す詳細カードなど */
  topRight?: ReactNode;
  /** 上中央に出す通知 */
  notice?: ReactNode;
  bottomLeft?: ReactNode;
  bottomRight?: ReactNode;
}

/**
 * 運用ビューの共通レイアウト。地図を全面に置き、各スロットの部品を重ねる。
 * 重ねる部品は、操作できる部分だけがポインタを受け取る(地図の操作を妨げない)。
 * パネルの背景の不透明度(--panel-alpha)は、ここで与える。運用ビューの外のカードには影響しない。
 */
export default function OperateLayout({
  map,
  topLeft,
  toolbar,
  panelBar,
  topRight,
  notice,
  bottomLeft,
  bottomRight,
}: Props) {
  const [opacity] = usePanelOpacity();
  const style = { "--panel-alpha": opacity } as CSSProperties;
  return (
    <div className="ops-overlay relative h-full w-full overflow-hidden" style={style}>
      <div className="absolute inset-0">{map}</div>
      <div className="pointer-events-none absolute inset-0">
        <div className="pointer-events-auto absolute left-3 top-3">{topLeft}</div>
        <div className="pointer-events-none absolute bottom-3 right-3 top-3 z-10 flex flex-col items-end">
          {panelBar && <div className="pointer-events-auto">{panelBar}</div>}
          <div className="flex flex-1 items-center">
            <div className="pointer-events-auto">{toolbar}</div>
          </div>
        </div>
        <div className="pointer-events-auto absolute right-16 top-3">{topRight}</div>
        <div className="pointer-events-auto absolute left-1/2 top-3 -translate-x-1/2">
          {notice}
        </div>
        <div className="pointer-events-auto absolute bottom-3 left-3">{bottomLeft}</div>
        <div className="pointer-events-auto absolute bottom-3 right-16">{bottomRight}</div>
      </div>
    </div>
  );
}
