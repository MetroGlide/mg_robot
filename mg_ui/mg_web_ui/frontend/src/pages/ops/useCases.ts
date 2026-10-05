import { ComponentType, lazy } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";

export interface UseCase {
  /** URL の /ops/:id */
  id: string;
  label: string;
  /** 運用ビュー。開いたときに読み込む(three.js などを使うため) */
  Operate: ComponentType<{ client: FoxgloveClientHandle }>;
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
];

export function findUseCase(id: string | undefined): UseCase | undefined {
  return USE_CASES.find((u) => u.id === id);
}
