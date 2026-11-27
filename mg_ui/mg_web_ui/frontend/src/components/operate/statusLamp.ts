import { ACTIVITY_CODE } from "../../types";

/**
 * 状態ランプ (パトライトのような色の表示) の見た目の表。色・点灯/点滅・点滅の速さは、ここだけで決める。
 * 活動の判定はロボット側 (waypoint_sequencer_node の ~/activity) が行い、UI は code から引くだけにする。
 * 色の考え方: 赤 = 止まり自動では戻れない、黄の点滅 = 止まって自動で対処中、黄の点灯 = 走るが精度・速度を落としている、
 * 緑 = 正常、青 = 人の操作による停止、橙 = 開始待ち、消灯 = 走行していない。
 */

export type ActivityCode = (typeof ACTIVITY_CODE)[keyof typeof ACTIVITY_CODE];

/** ランプの色。値は画面に出す色 (物理のランプに合わせた固定の色で、テーマでは変えない) */
export const LAMP_COLORS = {
  red: "#e53935",
  yellow: "#fbc02d",
  orange: "#fb8c00",
  green: "#43a047",
  blue: "#1e88e5",
  gray: "#9e9e9e",
  off: "transparent",
} as const;

export type LampColor = keyof typeof LAMP_COLORS;

/** 点滅の周期 [ms] (点いている時間と消えている時間の合計) */
export const BLINK_PERIOD_MS = {
  fast: 500,
  slow: 1200,
} as const;

export interface LampSpec {
  color: LampColor;
  /** 点滅の周期 [ms]。null は点灯 (消灯の色なら消灯) */
  blinkMs: number | null;
}

const SOLID = null;

/** 活動の code ごとのランプ。code を足したら、ここに書かないと型の検査で落ちる */
export const ACTIVITY_LAMPS: Record<ActivityCode, LampSpec> = {
  // 止まり、自動では戻れない
  error: { color: "red", blinkMs: SOLID },
  sensor_timeout_stop: { color: "red", blinkMs: SOLID },
  no_progress: { color: "red", blinkMs: SOLID },
  // 止まって、自動で対処している
  localization_hold: { color: "yellow", blinkMs: BLINK_PERIOD_MS.fast },
  watchdog_pause: { color: "yellow", blinkMs: BLINK_PERIOD_MS.fast },
  recovery: { color: "yellow", blinkMs: BLINK_PERIOD_MS.fast },
  detour_rejected: { color: "yellow", blinkMs: BLINK_PERIOD_MS.fast },
  obstacle_stop: { color: "yellow", blinkMs: BLINK_PERIOD_MS.fast },
  // 人の操作で止めている
  paused: { color: "blue", blinkMs: SOLID },
  // 開始待ち。MMI の LED1 (trigger_acceptable で橙の点滅。mg_drivers/scripts/mmi_switch_node.py) に合わせる
  trigger_wait: { color: "orange", blinkMs: BLINK_PERIOD_MS.slow },
  ready_to_start: { color: "orange", blinkMs: BLINK_PERIOD_MS.slow },
  // 走るが、精度・速度を落としている
  localization_recovering_drive: { color: "yellow", blinkMs: SOLID },
  localization_slowdown: { color: "yellow", blinkMs: SOLID },
  no_lidar_drive: { color: "yellow", blinkMs: SOLID },
  localization_unknown: { color: "yellow", blinkMs: SOLID },
  // 正常
  navigating: { color: "green", blinkMs: SOLID },
  manual_goal: { color: "green", blinkMs: SOLID },
  external_goal: { color: "green", blinkMs: SOLID },
  obstacle_slowdown: { color: "green", blinkMs: SOLID },
  departure_wait: { color: "green", blinkMs: SOLID },
  action: { color: "green", blinkMs: SOLID },
  // 走行していない
  goal_reached: { color: "off", blinkMs: SOLID },
  idle: { color: "off", blinkMs: SOLID },
};

/** 活動が届かない・古いとき (通信の途絶、シーケンサの停止)。ロボットのエラー (赤) と見分けるため灰にする */
export const STALE_LAMP: LampSpec = { color: "gray", blinkMs: SOLID };
/** 表にない code (ロボット側が新しく、UI が古いとき) */
export const UNKNOWN_LAMP: LampSpec = { color: "gray", blinkMs: SOLID };

/** ランプの色の表示名 (一覧の絞り込みと書き出しに使う) */
export const LAMP_COLOR_LABELS: Record<LampColor, string> = {
  red: "赤",
  yellow: "黄",
  orange: "橙",
  green: "緑",
  blue: "青",
  gray: "灰",
  off: "消灯",
};

function isActivityCode(code: string): code is ActivityCode {
  return Object.prototype.hasOwnProperty.call(ACTIVITY_LAMPS, code);
}

/** code のランプ。null は活動が届かない・古いとき */
export function lampFor(code: string | null): LampSpec {
  if (code === null) return STALE_LAMP;
  return isActivityCode(code) ? ACTIVITY_LAMPS[code] : UNKNOWN_LAMP;
}

/** ランプの状態の表示名 (例: 黄・速い点滅) */
export function lampLabel(lamp: LampSpec): string {
  const color = LAMP_COLOR_LABELS[lamp.color];
  if (lamp.blinkMs === null || lamp.color === "off") return color;
  const speed = lamp.blinkMs <= BLINK_PERIOD_MS.fast ? "速い点滅" : "遅い点滅";
  return `${color}・${speed}`;
}
