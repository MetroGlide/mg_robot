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

export interface ActivityLampMeaning {
  response: LampResponse;
  motion: LampMotion;
  /** ランプの見方のページに出す短い説明 (画面の文言はロボット側の text を使う) */
  description: string;
}

function meaning(
  response: LampResponse,
  motion: LampMotion,
  description: string,
): ActivityLampMeaning {
  return { response, motion, description };
}

/** 活動の code ごとの意味。code を足したら、ここに書かないと型の検査で落ちる */
export const ACTIVITY_MEANINGS: Record<ActivityCode, ActivityLampMeaning> = {
  // 走っている
  navigating: meaning("normal", "moving", "地点へ走行中"),
  manual_goal: meaning("normal", "moving", "指定した地点へ走行中"),
  external_goal: meaning(
    "normal",
    "moving",
    "外部指定のゴール (RViz など) へ走行中",
  ),
  obstacle_slowdown: meaning("normal", "moving", "障害物で減速して走行中"),
  localization_recovering_drive: meaning(
    "watch",
    "moving",
    "自己位置を復旧しながら走行中",
  ),
  localization_slowdown: meaning(
    "watch",
    "moving",
    "自己位置の不確かさで減速して走行中",
  ),
  no_lidar_drive: meaning("watch", "moving", "LiDAR なしで走行中"),
  localization_unknown: meaning(
    "watch",
    "moving",
    "自己位置の状態が届かないまま走行中",
  ),
  // 止まって待っている
  departure_wait: meaning("normal", "waiting", "出発までのカウントダウン"),
  action: meaning("normal", "waiting", "到着後のアクションを実行中"),
  goal_reached: meaning("normal", "waiting", "全地点に到着"),
  obstacle_stop: meaning("watch", "waiting", "障害物で停止して待っている"),
  paused: meaning("operator", "waiting", "人などの要求で一時停止中"),
  trigger_wait: meaning(
    "start",
    "waiting",
    "wait_trigger で、ボタンでの開始を待っている",
  ),
  ready_to_start: meaning("start", "waiting", "ボタンで走行を始められる"),
  // 異常で止まっている
  localization_hold: meaning("watch", "handling", "自己位置の確認のため停止中"),
  watchdog_pause: meaning("watch", "handling", "センサ監視による一時停止中"),
  recovery: meaning("watch", "handling", "リカバリー動作中"),
  detour_rejected: meaning("watch", "handling", "遠回りの経路を棄却して待機中"),
  error: meaning("act", "handling", "エラーで停止中"),
  sensor_timeout_stop: meaning("act", "handling", "センサ途絶で停止中"),
  no_progress: meaning("act", "handling", "進めていない"),
  // 走行していない
  idle: meaning("none", "moving", "Waypoint 走行なし"),
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
