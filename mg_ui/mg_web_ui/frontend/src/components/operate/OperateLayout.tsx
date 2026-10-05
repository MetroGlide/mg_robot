import { ReactNode } from "react";

interface Props {
  /** 全面に描く地図 */
  map: ReactNode;
  topLeft?: ReactNode;
  /** 右端の縦のツールバー */
  toolbar?: ReactNode;
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
  return (
    <div className="relative h-full w-full overflow-hidden">
      <div className="absolute inset-0">{map}</div>
      <div className="pointer-events-none absolute inset-0">
        <div className="pointer-events-auto absolute left-3 top-3">{topLeft}</div>
        <div className="pointer-events-auto absolute right-3 top-1/2 -translate-y-1/2">
          {toolbar}
        </div>
        <div className="pointer-events-auto absolute right-16 top-3">{topRight}</div>
        <div className="pointer-events-auto absolute left-1/2 top-3 -translate-x-1/2">
          {notice}
        </div>
        <div className="pointer-events-auto absolute bottom-3 left-3">{bottomLeft}</div>
        <div className="pointer-events-auto absolute bottom-3 right-3">{bottomRight}</div>
      </div>
    </div>
  );
}
