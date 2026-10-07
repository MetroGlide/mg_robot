import { ReactNode } from "react";
import { useOpsValue } from "../../hooks/useOpsValue";

interface DisclosureProps {
  title: string;
  /** 開閉の状態を保持するキー(画面を切り替えても、再読み込みしても残る) */
  storageKey: string;
  defaultOpen?: boolean;
  children: ReactNode;
}

/** 開閉できる節。開閉は端末ごとに保持する */
export default function Disclosure({
  title,
  storageKey,
  defaultOpen = false,
  children,
}: DisclosureProps) {
  const [open, setOpen] = useOpsValue<boolean>(storageKey, defaultOpen);
  return (
    <section className="rounded-lg bg-surface-sunken">
      <button
        type="button"
        aria-expanded={open}
        onClick={() => setOpen(!open)}
        className="flex w-full items-center justify-between px-3 py-2 text-xs font-bold"
      >
        {title}
        <span className="text-muted">{open ? "▲" : "▼"}</span>
      </button>
      {open && <div className="px-3 pb-3">{children}</div>}
    </section>
  );
}
