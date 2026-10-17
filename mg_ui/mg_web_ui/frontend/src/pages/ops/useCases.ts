import { ComponentType, lazy } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { NavLinkItem } from "../../components/shell/TopBar";

export interface OperateProps {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}

export interface UseCase {
  /** URL の /ops/:id。センサビューの "sensors"、システムビューの "system" とは重ねない */
  id: string;
  label: string;
  /** 運用ビュー。開いたときに読み込む(three.js などを使うため) */
  Operate: ComponentType<OperateProps>;
}

/**
 * 運用ビューのユースケース一覧。順序が上部バーの並びになる。
 * ユースケースを足すには、pages/ops/ にコンポーネントを書いて、ここに 1 行足す。
 */
export const USE_CASES: UseCase[] = [
  {
    id: "waypoint",
    label: "Waypoint",
    Operate: lazy(() => import("./WaypointOperate")),
  },
  {
    id: "slam",
    label: "SLAM",
    Operate: lazy(() => import("./SlamOperate")),
  },
  {
    id: "slam-gnss-2d",
    label: "SLAM-GNSS-2D",
    Operate: lazy(() => import("./SlamGnssOperate")),
  },
  {
    id: "scenario-test",
    label: "Scenario Test",
    Operate: lazy(() => import("./ScenarioOperate")),
  },
];

export function findUseCase(id: string | undefined): UseCase | undefined {
  return USE_CASES.find((u) => u.id === id);
}

export const USE_CASE_LINKS: NavLinkItem[] = USE_CASES.map((u) => ({
  id: u.id,
  label: u.label,
  to: `/ops/${u.id}`,
}));
