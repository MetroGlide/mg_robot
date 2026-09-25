import type {
  RunProgress,
  RunSummary,
  ScenarioInfo,
  ScenarioLogName,
  ScenarioResultData,
  ScenarioStatus,
} from "../types/scenarioTest";
import { getSysManagerUrl } from "./systemManagerConfig";

// 操作(POST)は useSystemManagerClient の callApi を使う。ここは参照(GET)だけ。

async function getJson<T>(path: string): Promise<T | null> {
  const r = await fetch(`${getSysManagerUrl()}${path}`);
  if (!r.ok) return null;
  return (await r.json()) as T;
}

function entryPath(runId: string, entry: string): string {
  return `/scenario/runs/${encodeURIComponent(runId)}/${encodeURIComponent(entry)}`;
}

export async function fetchScenarios(): Promise<ScenarioInfo[]> {
  const data = await getJson<{ scenarios: ScenarioInfo[] }>("/scenario/list");
  return data?.scenarios ?? [];
}

export function fetchScenarioStatus(): Promise<ScenarioStatus | null> {
  return getJson<ScenarioStatus>("/scenario/status");
}

export async function fetchRuns(limit = 30): Promise<RunSummary[]> {
  const data = await getJson<{ runs: RunSummary[] }>(
    `/scenario/runs?limit=${limit}`,
  );
  return data?.runs ?? [];
}

export function fetchRun(runId: string): Promise<RunProgress | null> {
  return getJson<RunProgress>(`/scenario/runs/${encodeURIComponent(runId)}`);
}

export function fetchEntryResult(
  runId: string,
  entry: string,
): Promise<ScenarioResultData | null> {
  return getJson<ScenarioResultData>(`${entryPath(runId, entry)}/result`);
}

export async function fetchEntryLog(
  runId: string,
  entry: string,
  name: ScenarioLogName,
  tail: number,
): Promise<string | null> {
  const data = await getJson<{ log: string }>(
    `${entryPath(runId, entry)}/log?name=${name}&tail=${tail}`,
  );
  return data?.log ?? null;
}
