import { ReactNode } from "react";

interface Props {
  title: string;
  /** 見出しの右に置く操作(切り替え、戻るリンクなど) */
  actions?: ReactNode;
  children: ReactNode;
}

/** 作業ビューの面。見出しと中身を 1 枚にまとめる */
export default function Panel({ title, actions, children }: Props) {
  return (
    <section className="space-y-3 rounded-xl border border-line bg-surface-elevated p-4">
      <div className="flex items-center justify-between gap-2">
        <h2 className="text-sm font-semibold">{title}</h2>
        {actions}
      </div>
      {children}
    </section>
  );
}
