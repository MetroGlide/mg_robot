import { ReactNode } from "react";
import { PanelId, usePanelVisible } from "./operatePanels";

/** 表示中のときだけ子を描く(隠している間は、子の購読や描画もしない) */
export default function PanelSlot({ id, children }: { id: PanelId; children: ReactNode }) {
  const [visible] = usePanelVisible(id);
  return visible ? <>{children}</> : null;
}
