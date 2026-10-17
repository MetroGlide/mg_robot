import { ReactElement } from "react";
import { useOpsValue } from "../../hooks/useOpsValue";
import { FlagIcon, PulseIcon, RobotIcon, SatelliteIcon, SlidersIcon } from "../ui/icons";

/** 運用ビューで、表示・非表示を切り替えられるパネル */
export type PanelId = "robotDetail"
  | "gnssDetail"
  | "diagnostics"
  | "actions"
  | "waypointProgress";

interface PanelDef {
  title: string;
  icon: () => ReactElement;
  /** 表示の状態を保持するキー(画面を切り替えても、再読み込みしても残る) */
  storageKey: string;
  defaultVisible: boolean;
}

/**
 * 切り替えられるパネルの定義。パネルを増やすときは、ここに 1 つ足し、ビューで PanelToggleBar の ids と PanelSlot に使う。
 * 表示の状態はビューをまたいで共有する(あるビューで隠したパネルは、ほかのビューでも隠れる)。
 */
export const PANELS: Record<PanelId, PanelDef> = {
  robotDetail: {
    title: "ロボットの詳細",
    icon: RobotIcon,
    storageKey: "panel.robotDetail",
    defaultVisible: true,
  },
  gnssDetail: {
    title: "GNSS の詳細",
    icon: SatelliteIcon,
    storageKey: "panel.gnssDetail",
    defaultVisible: false,
  },
  diagnostics: {
    title: "診断",
    icon: PulseIcon,
    storageKey: "panel.diagnostics",
    defaultVisible: true,
  },
  actions: {
    title: "操作",
    icon: SlidersIcon,
    storageKey: "panel.actions",
    defaultVisible: true,
  },
  waypointProgress: {
    title: "Waypoint 走行",
    icon: FlagIcon,
    storageKey: "panel.waypointProgress",
    defaultVisible: true,
  },
};

/** パネルの表示状態。KPI の GNSS タイルなど、切り替えバー以外からも同じ状態を操作できる */
export function usePanelVisible(id: PanelId): [boolean, (value: boolean) => void] {
  const { storageKey, defaultVisible } = PANELS[id];
  return useOpsValue<boolean>(storageKey, defaultVisible);
}
