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

const HOST_RE = /^[a-zA-Z0-9][a-zA-Z0-9.-]*$/;

/** 実機PCのアドレスとして使えるか(空は「指定しない」として有効) */
export function isValidRobotAddress(value: string): boolean {
  return value === "" || HOST_RE.test(value);
}
