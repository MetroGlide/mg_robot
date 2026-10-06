import { Marker, MARKER_TYPE } from "../../../types/ros-types";

export interface WaypointPin {
  index: number;
  x: number;
  y: number;
  /** 地図上の向き [rad] */
  yaw: number;
  /** 止まらずに通過する点か(シーケンサは通過点を緑、停止点を赤で配信する) */
  through: boolean;
}

// シーケンサがウェイポイントの矢印を配信するときの名前空間
const WAYPOINT_NS = "waypoints";

/** マーカーの配列から、ウェイポイントの矢印だけを取り出し、番号の順に並べる */
export function waypointPinsFromMarkers(markers: Marker[]): WaypointPin[] {
  return markers
    .filter((m) => m.ns === WAYPOINT_NS && m.type === MARKER_TYPE.ARROW)
    .map((m) => {
      const { x, y, z, w } = m.pose.orientation;
      return {
        index: m.id,
        x: m.pose.position.x,
        y: m.pose.position.y,
        yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
        through: m.color.g > m.color.r,
      };
    })
    .sort((a, b) => a.index - b.index);
}

export type PinState = "past" | "current" | "upcoming";

/**
 * 走行中のウェイポイントの状態。通過済みは薄く、いまの目標は強調する。
 * 走行していない(待機中や状態が分からない)ときは、すべて通常の表示にする。
 */
export function pinState(
  index: number,
  currentIndex: number | null,
  running: boolean,
): PinState {
  if (!running || currentIndex === null) return "upcoming";
  if (index < currentIndex) return "past";
  return index === currentIndex ? "current" : "upcoming";
}
