import { ReactNode } from "react";
import Card from "../ui/Card";
import IconButton from "../ui/IconButton";
import { PANELS, PanelId, usePanelVisible } from "./operatePanels";

interface TogglePanelProps {
  id: PanelId;
  /** 表示中のパネル。渡される close で、パネルを閉じる(パネルの × に使う) */
  children: (close: () => void) => ReactNode;
}

/**
 * 表示を切り替えられるパネル。閉じている間は、パネルの代わりにアイコン(小さな四角)を同じ位置に出し、
 * 押すとパネルが開く(開いている間はアイコンを出さない)。閉じるには、パネルの × を使う。
 * 右側の列に縦に積んで使うと、閉じたパネルは下のパネルが詰め、アイコンはそのパネルが出る位置に残る。
 * 閉じている間は、子を描かない(描画も購読もしない)。
 */
export default function TogglePanel({ id, children }: TogglePanelProps) {
  const [visible, setVisible] = usePanelVisible(id);
  if (visible) return <>{children(() => setVisible(false))}</>;
  const { title, icon: PanelIcon } = PANELS[id];
  return (
    <Card solid className="p-1">
      <IconButton
        icon={<PanelIcon />}
        title={`${title}を表示`}
        onClick={() => setVisible(true)}
      />
    </Card>
  );
}
