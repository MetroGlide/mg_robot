import type { ResolvedTheme } from "../../contexts/ThemeContext";

/** 地図の上に描く要素の色。地図と背景(gridColors の mapLight / mapDark)の上で見分けられるように選ぶ */
export interface SceneColors {
  lidar: string;
  /** 点やマーカーの縁取り。地図の明暗によらず、点が背景に埋もれないようにする */
  halo: string;
  plan: string;
  actualPath: string;
  robot: string;
  particle: string;
  collision: string;
  /** ウェイポイントのうち、止まらずに通過する点 */
  waypointThrough: string;
  /** ウェイポイントのうち、止まる点 */
  waypointStop: string;
  waypointLabel: string;
}

export const SCENE_COLORS: Record<ResolvedTheme, SceneColors> = {
  light: {
    lidar: "#ea580c",
    halo: "#ffffff",
    plan: "#dc2626",
    actualPath: "#7e22ce",
    robot: "#0369a1",
    particle: "#15803d",
    collision: "#be185d",
    waypointThrough: "#16a34a",
    waypointStop: "#dc2626",
    waypointLabel: "#ffffff",
  },
  dark: {
    lidar: "#22d3ee",
    halo: "#0b111e",
    plan: "#f87171",
    actualPath: "#c084fc",
    robot: "#38bdf8",
    particle: "#4ade80",
    collision: "#f472b6",
    waypointThrough: "#4ade80",
    waypointStop: "#f87171",
    waypointLabel: "#0b111e",
  },
};
