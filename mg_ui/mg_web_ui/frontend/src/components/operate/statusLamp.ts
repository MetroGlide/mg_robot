import { ACTIVITY_CODE } from "../../types";

/**
 * 状態ランプ (パトライトのような色の表示) の見た目の表。色・点灯/点滅・点滅の速さは、ここだけで決める。
 * 活動の判定はロボット側 (waypoint_sequencer_node の ~/activity) が行い、UI は code から引くだけにする。
 *
 * ランプは 2 つの軸で決める。code ごとには 2 つの属性だけを書き、色と点滅は下の 2 つの表から引く。
 * - 色 = 人がどうすればよいか (RESPONSE_COLORS)
 * - 点灯 / 点滅 = 動いているか (MOTION_BLINK_MS)
 * 色や点滅を変えたいときは、表の 1 行を変える (同じ意味の活動がまとめて変わる)。
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

/**
 * 点滅の周期 [ms] (点いている時間と消えている時間の合計)。
 * MMI の LED の点滅の周期はマイコン側で決まる。実機で測り、橙の点滅 (遅い点滅) をそれに合わせる。
 */
export const BLINK_PERIOD_MS = {
  fast: 500,
  slow: 1200,
} as const;

export interface LampSpec {
  color: LampColor;
  /** 点滅の周期 [ms]。null は点灯 (消灯の色なら消灯) */
  blinkMs: number | null;
}

/** 人がどうすればよいか */
export type LampResponse =
  /** 何もしなくてよい (正常) */
  | "normal"
  /** 見守る (いつもと違うが、自動で対処・制限している) */
  | "watch"
  /** 人が対応する (自動では戻らない) */
  | "act"
  /** 人が止めている (人が解除する) */
  | "operator"
  /** ボタン (MMI など) で走行を始められる */
  | "start"
  /** 走行していない */
  | "none";

/** 動いているか */
export type LampMotion =
  /** 走っている (または、続いているだけの状態) */
  | "moving"
  /** 止まって待っている (予定どおり、または外の条件で) */
  | "waiting"
  /** 異常で止まっている */
  | "handling";

/** 色の表。橙は MMI の LED1 (trigger_acceptable で橙の点滅。mg_drivers/scripts/mmi_protocol.py) に合わせる */
export const RESPONSE_COLORS: Record<LampResponse, LampColor> = {
  normal: "green",
  watch: "yellow",
  act: "red",
  operator: "blue",
  start: "orange",
  none: "off",
};

/** 動きの表 (点滅の周期。null は点灯) */
export const MOTION_BLINK_MS: Record<LampMotion, number | null> = {
  moving: null,
  waiting: BLINK_PERIOD_MS.slow,
  handling: BLINK_PERIOD_MS.fast,
};

export interface ActivityLampMeaning {
  response: LampResponse;
  motion: LampMotion;
}

function meaning(
  response: LampResponse,
  motion: LampMotion,
): ActivityLampMeaning {
  return { response, motion };
}

/** 活動の code ごとの意味。code を足したら、ここに書かないと型の検査で落ちる */
export const ACTIVITY_MEANINGS: Record<ActivityCode, ActivityLampMeaning> = {
  // 走っている
  navigating: meaning("normal", "moving"),
  manual_goal: meaning("normal", "moving"),
  external_goal: meaning("normal", "moving"),
  obstacle_slowdown: meaning("normal", "moving"),
  localization_recovering_drive: meaning("watch", "moving"),
  localization_slowdown: meaning("watch", "moving"),
  no_lidar_drive: meaning("watch", "moving"),
  localization_unknown: meaning("watch", "moving"),
  // 止まって待っている
  departure_wait: meaning("normal", "waiting"),
  action: meaning("normal", "waiting"),
  goal_reached: meaning("normal", "waiting"),
  obstacle_stop: meaning("watch", "waiting"),
  paused: meaning("operator", "waiting"),
  trigger_wait: meaning("start", "waiting"),
  ready_to_start: meaning("start", "waiting"),
  // 異常で止まっている
  localization_hold: meaning("watch", "handling"),
  watchdog_pause: meaning("watch", "handling"),
  recovery: meaning("watch", "handling"),
  detour_rejected: meaning("watch", "handling"),
  error: meaning("act", "handling"),
  sensor_timeout_stop: meaning("act", "handling"),
  no_progress: meaning("act", "handling"),
  // 走行していない
  idle: meaning("none", "moving"),
};

/** 意味からランプを決める (消灯なら点滅しない) */
export function lampOf({ response, motion }: ActivityLampMeaning): LampSpec {
  const color = RESPONSE_COLORS[response];
  return { color, blinkMs: color === "off" ? null : MOTION_BLINK_MS[motion] };
}

/** 活動の code ごとのランプ (ACTIVITY_MEANINGS から作る) */
export const ACTIVITY_LAMPS = Object.fromEntries(
  Object.entries(ACTIVITY_MEANINGS).map(([code, m]) => [code, lampOf(m)]),
) as Record<ActivityCode, LampSpec>;

/** 活動が届かない・古いとき (通信の途絶、シーケンサの停止)。ロボットのエラー (赤) と見分けるため灰にする */
export const STALE_LAMP: LampSpec = { color: "gray", blinkMs: null };
/** 表にない code (ロボット側が新しく、UI が古いとき) */
export const UNKNOWN_LAMP: LampSpec = { color: "gray", blinkMs: null };

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
  return Object.prototype.hasOwnProperty.call(ACTIVITY_MEANINGS, code);
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
