import { ACTIVITY_CODE, NavigationActivity } from "../../types";

// 補足 (detail) を並べる活動。ほかの活動では detail が内部の値 (アクションの type など) なので出さない
const SHOW_DETAIL = new Set<string>([
  ACTIVITY_CODE.PAUSED,
  ACTIVITY_CODE.LOCALIZATION_HOLD,
  ACTIVITY_CODE.WATCHDOG_PAUSE,
  ACTIVITY_CODE.LOCALIZATION_RECOVERING_DRIVE,
  ACTIVITY_CODE.LOCALIZATION_SLOWDOWN,
  ACTIVITY_CODE.NO_LIDAR_DRIVE,
  ACTIVITY_CODE.OBSTACLE_STOP,
  ACTIVITY_CODE.OBSTACLE_SLOWDOWN,
  ACTIVITY_CODE.SENSOR_TIMEOUT_STOP,
  ACTIVITY_CODE.DETOUR_REJECTED,
]);

// 続いている秒数を並べる活動
const SHOW_ELAPSED = new Set<string>([
  ACTIVITY_CODE.PAUSED,
  ACTIVITY_CODE.LOCALIZATION_HOLD,
  ACTIVITY_CODE.WATCHDOG_PAUSE,
  ACTIVITY_CODE.LOCALIZATION_RECOVERING_DRIVE,
  ACTIVITY_CODE.OBSTACLE_STOP,
  ACTIVITY_CODE.SENSOR_TIMEOUT_STOP,
  ACTIVITY_CODE.NO_PROGRESS,
  ACTIVITY_CODE.RECOVERY,
  ACTIVITY_CODE.DETOUR_REJECTED,
  ACTIVITY_CODE.TRIGGER_WAIT,
]);

/** 活動を 1 行の文にする。判定はせず、ロボット側が決めた値を並べるだけ */
export function formatActivity(activity: NavigationActivity): string {
  const parts = [activity.text];
  if (activity.index >= 0 && activity.total > 0) {
    parts.push(`地点 ${activity.index + 1} / ${activity.total}`);
  }
  if (SHOW_DETAIL.has(activity.code) && activity.detail !== "") {
    parts.push(activity.detail);
  }
  if (activity.remaining >= 0) {
    parts.push(
      activity.code === ACTIVITY_CODE.DEPARTURE_WAIT
        ? `${activity.remaining.toFixed(1)} 秒`
        : `残り ${activity.remaining.toFixed(1)} m`,
    );
  }
  if (SHOW_ELAPSED.has(activity.code) && activity.elapsed_sec >= 1) {
    parts.push(`${Math.floor(activity.elapsed_sec)} 秒`);
  }
  return parts.join(" ・ ");
}
