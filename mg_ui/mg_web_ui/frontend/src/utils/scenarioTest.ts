import type {
  EntryStatus,
  ProgressEntry,
  ScenarioInfo,
  ScenarioStatus,
} from "../types/scenarioTest";

export const SCENARIO_TEST_SERVICE = "scenario-test";
export const SCENARIO_ENV_SERVICE = "scenario-env";
export const GAZEBO_SERVICE = "gazebo-simulation";

/** 既知の問題で失敗するシナリオに付けるタグ(Makefile では既定で除外している) */
export const KNOWN_ISSUE_TAG = "known_issue";

export interface ScenarioFilter {
  /** "" ならすべてのグループ */
  group: string;
  /** "" ならタグで絞り込まない */
  tag: string;
  showKnownIssues: boolean;
}

export function filterScenarios(
  scenarios: readonly ScenarioInfo[],
  filter: ScenarioFilter,
): ScenarioInfo[] {
  return scenarios.filter(
    (s) =>
      (!filter.group || s.group === filter.group) &&
      (!filter.tag || s.tags.includes(filter.tag)) &&
      (filter.showKnownIssues || !s.tags.includes(KNOWN_ISSUE_TAG)),
  );
}

export function collectGroups(scenarios: readonly ScenarioInfo[]): string[] {
  return [...new Set(scenarios.map((s) => s.group))];
}

export function collectTags(scenarios: readonly ScenarioInfo[]): string[] {
  return [...new Set(scenarios.flatMap((s) => s.tags))].sort();
}

export interface ProgressCounts {
  total: number;
  /** 結果が出たもの(PASSED・FAILED・ERROR) */
  done: number;
  passed: number;
  failed: number;
  error: number;
}

export function countProgress(
  entries: readonly ProgressEntry[],
): ProgressCounts {
  const count = (status: EntryStatus) =>
    entries.filter((e) => e.status === status).length;
  const passed = count("PASSED");
  const failed = count("FAILED");
  const error = count("ERROR");
  return {
    total: entries.length,
    done: passed + failed + error,
    passed,
    failed,
    error,
  };
}

/** シナリオ名 → 説明。進捗の表などに説明を添えるために引く */
export function descriptionsByName(
  scenarios: readonly ScenarioInfo[],
): Map<string, string> {
  return new Map(scenarios.map((s) => [s.name, s.description]));
}

export interface CurrentEntry {
  entry: ProgressEntry;
  /** 0 始まりの位置 */
  index: number;
}

/** いま実行中のエントリ。なければ null */
export function currentEntry(
  entries: readonly ProgressEntry[],
): CurrentEntry | null {
  const index = entries.findIndex((e) => e.status === "RUNNING");
  return index < 0 ? null : { entry: entries[index], index };
}

/** 失敗した(FAILED・ERROR の)エントリ */
export function failedEntries(
  entries: readonly ProgressEntry[],
): ProgressEntry[] {
  return entries.filter((e) => e.status === "FAILED" || e.status === "ERROR");
}

/** 進捗バーの 1 区切り(シナリオ 1 本分) */
export interface Segment {
  status: EntryStatus;
  /** 区切りに添える名前(tooltip 用) */
  label: string;
}

export function segmentsFromEntries(
  entries: readonly ProgressEntry[],
): Segment[] {
  return entries.map((e) => ({ status: e.status, label: e.result_dir }));
}

const COUNT_ORDER: EntryStatus[] = [
  "PASSED",
  "FAILED",
  "ERROR",
  "RUNNING",
  "PENDING",
];

/** 件数だけが分かる run(一覧の要約)の区切り。結果の種類ごとにまとめて並べ、残りは未実行にする */
export function segmentsFromCounts(
  counts: Partial<Record<EntryStatus, number>>,
  total: number,
): Segment[] {
  const segments: Segment[] = [];
  for (const status of COUNT_ORDER) {
    const n = status === "PENDING" ? 0 : (counts[status] ?? 0);
    for (let i = 0; i < n; i++) segments.push({ status, label: status });
  }
  while (segments.length < total) {
    segments.push({ status: "PENDING", label: "PENDING" });
  }
  return segments;
}

/**
 * 開始時刻(タイムゾーン付きの ISO 8601)からの経過秒。
 * 読めない値は null。コンテナとブラウザの時計が少しずれても、負にはしない。
 */
export function elapsedSinceSec(
  startedAt: string | undefined,
  nowMs: number,
): number | null {
  if (!startedAt) return null;
  const startMs = Date.parse(startedAt);
  if (Number.isNaN(startMs)) return null;
  return Math.max(0, (nowMs - startMs) / 1000);
}

/** 経過時間の表示。60 秒以上は分と秒にする */
export function formatElapsed(sec: number): string {
  const total = Math.max(0, Math.round(sec));
  if (total < 60) return `${total}s`;
  const m = Math.floor(total / 60);
  const s = total % 60;
  return `${m}m ${String(s).padStart(2, "0")}s`;
}

export type RunPhase = "idle" | "starting" | "running";

/**
 * 実行の段階。コンテナは動いているが、まだ新しい run の progress.json が
 * 書かれていない間(起動直後)は starting とする。
 */
export function runPhase(status: ScenarioStatus | null): RunPhase {
  if (!status || status.services[SCENARIO_TEST_SERVICE] !== "running") {
    return "idle";
  }
  return status.run?.state === "running" ? "running" : "starting";
}

const RUN_ID_RE = /^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})$/;
const JST_FORMAT = new Intl.DateTimeFormat("ja-JP", {
  timeZone: "Asia/Tokyo",
  year: "numeric",
  month: "2-digit",
  day: "2-digit",
  hour: "2-digit",
  minute: "2-digit",
  second: "2-digit",
  hourCycle: "h23",
});

/**
 * run_id(system_manager が UTC で付ける YYYYMMDD_HHMMSS)を JST の「YYYY/MM/DD HH:MM:SS」にする。
 * 日時の形でない id(手で付けた名前)はそのまま返す。
 */
export function formatRunTime(runId: string): string {
  const m = RUN_ID_RE.exec(runId);
  if (!m) return runId;
  const [, y, mo, d, h, mi, s] = m.map(Number);
  const ms = Date.UTC(y, mo - 1, d, h, mi, s);
  return JST_FORMAT.format(new Date(ms)).replace(/-/g, "/");
}

/** 実行したシナリオの要約。1 本ならその名前、複数なら「先頭の名前 ほか n 本」 */
export function summarizeScenarioNames(names: readonly string[]): string {
  if (names.length === 0) return "";
  if (names.length === 1) return names[0];
  return `${names[0]} ほか ${names.length - 1} 本`;
}

const HOST_RE =/^[a-zA-Z0-9][a-zA-Z0-9.-]*$/;

/** 実機PCのアドレスとして使えるか(空は「指定しない」として有効) */
export function isValidRobotAddress(value: string): boolean {
  return value === "" || HOST_RE.test(value);
}
