import { CSSProperties, ReactNode } from "react";
import { usePanelOpacity } from "../../hooks/usePanelOpacity";

interface Props {
  /** 全面に描く地図 */
  map: ReactNode;
  topLeft?: ReactNode;
  /** 右下のアイコンの列の上に置くツールバー。右下のパネルは、この列の幅だけ左に寄る */
  toolbar?: ReactNode;
  /** 右上のパネルの列。パネルごとの切り替えアイコンは、パネルの右隣(右端)に TogglePanel が置く */
  topRight?: ReactNode;
  /** 上中央に出す通知 */
  notice?: ReactNode;
  bottomLeft?: ReactNode;
  /** 右下のパネルの列(アイコンは含めない)。下端は、アイコンの列とそろう */
  bottomRight?: ReactNode;
  /** 右下のパネルの切り替えアイコン。ツールバーの下、右下の隅に置く(PanelToggleButton) */
  bottomRightIcon?: ReactNode;
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
  topRight,
  notice,
  bottomLeft,
  bottomRight,
  bottomRightIcon,
}: Props) {
  const [opacity] = usePanelOpacity();
  const style = { "--panel-alpha": opacity } as CSSProperties;
  return (
    <div className="ops-overlay relative h-full w-full overflow-hidden" style={style}>
      <div className="absolute inset-0">{map}</div>
      <div className="pointer-events-none absolute inset-0">
        <div className="pointer-events-auto absolute left-3 top-3">{topLeft}</div>
        <div className="pointer-events-auto absolute right-3 top-3">{topRight}</div>
        <div className="pointer-events-auto absolute left-1/2 top-3 -translate-x-1/2">
          {notice}
        </div>
        <div className="pointer-events-auto absolute bottom-3 left-3">{bottomLeft}</div>
        <div className="absolute bottom-3 right-3 flex items-end gap-2">
          <div className="pointer-events-auto flex flex-col items-end gap-2">
            {bottomRight}
          </div>
          <div className="pointer-events-auto z-10 flex flex-col items-end gap-2">
            {toolbar}
            {bottomRightIcon}
          </div>
        </div>
      </div>
    </div>
  );
}
