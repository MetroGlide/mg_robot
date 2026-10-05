import { ReactNode } from "react";

interface Props {
  title: string;
  /** 見出しの右に置く、ビュー全体に関わる操作や状態 */
  headerRight?: ReactNode;
  children: ReactNode;
}

/**
 * 作業ビューの共通レイアウト。地図を使わず、表やフォームを中心にしたビュー
 * (シナリオテスト、システム)に使う。ページの中身だけを縦にスクロールさせる。
 */
export default function WorkLayout({ title, headerRight, children }: Props) {
  return (
    <div className="h-full overflow-y-auto">
      <div className="mx-auto max-w-[1600px] space-y-3 p-4">
        <div className="flex flex-wrap items-center gap-3">
          <h1 className="text-lg font-bold">{title}</h1>
          <div className="ml-auto flex flex-wrap items-center gap-2">{headerRight}</div>
        </div>
        {children}
      </div>
    </div>
  );
}
