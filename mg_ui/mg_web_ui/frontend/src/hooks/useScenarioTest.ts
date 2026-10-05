import { useCallback, useEffect, useState } from "react";
import type {
  RunSummary,
  ScenarioInfo,
  ScenarioStatus,
} from "../types/scenarioTest";
import {
  fetchRuns,
  fetchScenarios,
  fetchScenarioStatus,
} from "../utils/scenarioTestApi";

const STATUS_POLL_INTERVAL_MS = 2000;

export interface ScenarioTestHandle {
  scenarios: ScenarioInfo[];
  status: ScenarioStatus | null;
  runs: RunSummary[];
  reloadScenarios: () => Promise<void>;
  /** 状態と run の一覧を今すぐ取得し直す */
  refresh: () => Promise<void>;
}

/**
 * シナリオテストの状態を system_manager から取得する。
 * 状態は一定間隔で確認し(非表示のタブでは止める)、run が変わったら一覧も取得し直す。
 */
export function useScenarioTest(): ScenarioTestHandle {
  const [scenarios, setScenarios] = useState<ScenarioInfo[]>([]);
  const [status, setStatus] = useState<ScenarioStatus | null>(null);
  const [runs, setRuns] = useState<RunSummary[]>([]);

  const reloadScenarios = useCallback(async () => {
    try {
      setScenarios(await fetchScenarios());
    } catch {
      // system_manager が停止中でも画面は表示し、再読み込みを待つ
    }
  }, []);

  const refreshStatus = useCallback(async () => {
    try {
      setStatus(await fetchScenarioStatus());
    } catch {
      // system_manager が停止中は次の定期確認で回復を待つ
    }
  }, []);

  const refreshRuns = useCallback(async () => {
    try {
      setRuns(await fetchRuns());
    } catch {
      // 同上
    }
  }, []);

  const refresh = useCallback(async () => {
    await Promise.all([refreshStatus(), refreshRuns()]);
  }, [refreshStatus, refreshRuns]);

  useEffect(() => {
    void reloadScenarios();
  }, [reloadScenarios]);

  useEffect(() => {
    const tick = () => {
      if (!document.hidden) void refreshStatus();
    };
    tick();
    const id = setInterval(tick, STATUS_POLL_INTERVAL_MS);
    document.addEventListener("visibilitychange", tick);
    return () => {
      clearInterval(id);
      document.removeEventListener("visibilitychange", tick);
    };
  }, [refreshStatus]);

  // 新しい run が始まった・終わったときだけ一覧を取り直す(毎回の確認では取らない)
  const latestRunId = status?.run?.run_id;
  const latestRunState = status?.run?.state;
  useEffect(() => {
    void refreshRuns();
  }, [latestRunId, latestRunState, refreshRuns]);

  return { scenarios, status, runs, reloadScenarios, refresh };
}
