import { Tone } from "../ui/Pill";
import { Step } from "../ui/Stepper";

/** クォータニオンから、地図平面での向き(yaw)を度で返す */
export function quaternionToYawDeg(q: {
  x: number;
  y: number;
  z: number;
  w: number;
}): number {
  const yaw = Math.atan2(
    2 * (q.w * q.z + q.x * q.y),
    1 - 2 * (q.y * q.y + q.z * q.z),
  );
  return (yaw * 180) / Math.PI;
}

/** LocalizationStatus.state(0 NORMAL ... 4 DEGRADED)の色 */
export function localizationTone(state: number): Tone {
  if (state === 0) return "ok";
  if (state === 4) return "error";
  return "warn";
}

/** waypoint_sequencer の通常の流れ。SUSPENDED と ERROR は流れの外なので段にしない */
export const SEQUENCER_STEPS: Step[] = [
  { id: "IDLE", label: "待機" },
  { id: "ON_STARTING", label: "出発前" },
  { id: "NAVIGATING", label: "走行中" },
  { id: "ON_ARRIVING", label: "到着処理" },
  { id: "GOAL_REACHED", label: "到着" },
];

/**
 * ステッパーの現在の段と色を決める。
 * SUSPENDED(一時停止)は、止まる前の段を特定できないので走行中の段を warn で示す。ERROR は待機の段を error で示す。
 */
export function sequencerStep(state: string | null): {
  currentId: string | null;
  tone: Tone;
} {
  if (state === null) return { currentId: null, tone: "neutral" };
  if (state === "SUSPENDED") return { currentId: "NAVIGATING", tone: "warn" };
  if (state === "ERROR") return { currentId: "IDLE", tone: "error" };
  return { currentId: state, tone: "ok" };
}
