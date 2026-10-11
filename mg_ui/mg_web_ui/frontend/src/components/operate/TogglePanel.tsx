import { ReactNode } from "react";
import Card from "../ui/Card";
import IconButton from "../ui/IconButton";
import { PANELS, PanelId, usePanelVisible } from "./operatePanels";

/**
 * アイコンの列の幅(アイコンのカード + 隙間)。アイコンのないカードを同じ列に並べるときの余白に使い、
 * パネルの右端をそろえる。
 */
export const DOCK_INSET_CLASS = "mr-[3.375rem]";

/** パネルの表示を切り替えるアイコン(小さな四角)。表示中は強調する */
export function PanelToggleButton({ id }: { id: PanelId }) {
  const [visible, setVisible] = usePanelVisible(id);
  const { title, icon: PanelIcon } = PANELS[id];
  return (
    <Card solid className="p-1">
      <IconButton
        icon={<PanelIcon />}
        title={`${title}を${visible ? "隠す" : "表示"}`}
        active={visible}
        onClick={() => setVisible(!visible)}
      />
    </Card>
  );
}

/** 表示中のときだけ子を描く(隠している間は、子の描画も購読もしない) */
export function PanelBody({ id, children }: { id: PanelId; children: ReactNode }) {
  const [visible] = usePanelVisible(id);
  return visible ? <>{children}</> : null;
}

interface TogglePanelProps {
  id: PanelId;
  /** アイコンを置く高さ。start はパネルの上端、end は下端 */
  align?: "start" | "end";
  children: ReactNode;
}

/**
 * パネルと、その表示を切り替えるアイコンの 1 行。アイコンはパネルの右隣に常に残り、押すとパネルを出し入れする。
 * 縦に積んだ行は、隠すと下の行が詰める。
 * アイコンを別の列に置く必要がある場合(右下のように、アイコンの上にツールバーを重ねる列)は、
 * PanelBody と PanelToggleButton を別々に置く。
 */
export default function TogglePanel({ id, align = "start", children }: TogglePanelProps) {
  return (
    <div className={`flex gap-2 ${align === "end" ? "items-end" : "items-start"}`}>
      <PanelBody id={id}>{children}</PanelBody>
      <PanelToggleButton id={id} />
    </div>
  );
}
