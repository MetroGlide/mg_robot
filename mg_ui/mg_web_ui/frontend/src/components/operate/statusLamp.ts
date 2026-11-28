/**
 * 状態ランプ (パトライトのような色の表示) の見た目の表。色・点灯/点滅・点滅の速さは、ここだけで決める。
 * 活動の判定と、活動ごとの意味 (人がどうすればよいか・動いているか) はロボット側が決める
 * (mg_waypoint_navigation/config/activity_lamps.yaml。~/activity の lamp_response・lamp_motion)。
 * MMI の LED も同じ意味から色を決める。ここは意味から画面の見た目を決める 2 つの表だけを持つ。
 * - 色 = 人がどうすればよいか (RESPONSE_COLORS)
 * - 点灯 / 点滅 = 動いているか (MOTION_BLINK_MS)
 */

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

/** 色の軸の表示名 (ランプの見方のページ) */
export const RESPONSE_LABELS: Record<LampResponse, string> = {
  normal: "何もしなくてよい (正常)",
  watch: "見守る (いつもと違うが、自動で対処・制限している)",
  act: "人が対応する (自動では戻らない)",
  operator: "人が止めている (人が解除する)",
  start: "ボタンで走行を始められる (MMI の LED1 と同じ)",
  none: "走行していない",
};

/** 動きの軸の表示名 (ランプの見方のページ) */
export const MOTION_LABELS: Record<LampMotion, string> = {
  moving: "点灯: 走っている",
  waiting: "遅い点滅: 止まって待っている",
  handling: "速い点滅: 異常で止まっている",
};

/** 動きの表 (点滅の周期。null は点灯) */
export const MOTION_BLINK_MS: Record<LampMotion, number | null> = {
  moving: null,
  waiting: BLINK_PERIOD_MS.slow,
  handling: BLINK_PERIOD_MS.fast,
};

function isResponse(value: string): value is LampResponse {
  return Object.prototype.hasOwnProperty.call(RESPONSE_COLORS, value);
}

function isMotion(value: string): value is LampMotion {
  return Object.prototype.hasOwnProperty.call(MOTION_BLINK_MS, value);
}

/** 意味からランプを決める (消灯なら点滅しない)。意味が分からなければ UNKNOWN_LAMP */
export function lampOf(response: string, motion: string): LampSpec {
  if (!isResponse(response) || !isMotion(motion)) return UNKNOWN_LAMP;
  const color = RESPONSE_COLORS[response];
  return { color, blinkMs: color === "off" ? null : MOTION_BLINK_MS[motion] };
}

/** 活動が届かない・古いとき (通信の途絶、シーケンサの停止)。ロボットのエラー (赤) と見分けるため灰にする */
export const STALE_LAMP: LampSpec = { color: "gray", blinkMs: null };
/** 意味が分からないとき (ロボット側が新しく、UI が古いとき) */
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

/** いまの活動のランプ。null は活動が届かない・古いとき */
export function lampFor(
  activity: { lamp_response: string; lamp_motion: string } | null,
): LampSpec {
  if (activity === null) return STALE_LAMP;
  return lampOf(activity.lamp_response, activity.lamp_motion);
}

/** ランプの状態の表示名 (例: 黄・速い点滅) */
export function lampLabel(lamp: LampSpec): string {
  const color = LAMP_COLOR_LABELS[lamp.color];
  if (lamp.blinkMs === null || lamp.color === "off") return color;
  const speed = lamp.blinkMs <= BLINK_PERIOD_MS.fast ? "速い点滅" : "遅い点滅";
  return `${color}・${speed}`;
}
