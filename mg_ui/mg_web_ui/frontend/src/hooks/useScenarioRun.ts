import { useCallback, useEffect, useMemo, useState } from "react";
import { SystemManagerHandle } from "./useSystemManagerClient";
import { useOpsValue } from "./useOpsValue";
import { useScenarioTest } from "./useScenarioTest";
import {
  DEFAULT_RUN_OPTIONS,
  RunOptionsValue,
} from "../components/scenario/RunOptionsForm";
import type { RunProgress, RunRequest } from "../types/scenarioTest";
import { fetchRun } from "../utils/scenarioTestApi";
import { loadSettings, saveSettings } from "../utils/settingsApi";
import {
  GAZEBO_SERVICE,
  isValidRobotAddress,
  runPhase,
  SCENARIO_ENV_SERVICE,
} from "../utils/scenarioTest";

const SETTINGS_KEY = "scenario_test";
const NO_SELECTION: string[] = [];

function parseOptions(raw: unknown): RunOptionsValue {
  if (typeof raw !== "object" || raw === null) return DEFAULT_RUN_OPTIONS;
  const v = raw as Partial<Record<keyof RunOptionsValue, unknown>>;
  return {
    repeat:
      typeof v.repeat === "number" ? v.repeat : DEFAULT_RUN_OPTIONS.repeat,
    gui: typeof v.gui === "boolean" ? v.gui : DEFAULT_RUN_OPTIONS.gui,
    attach:
      typeof v.attach === "boolean" ? v.attach : DEFAULT_RUN_OPTIONS.attach,
    robot: typeof v.robot === "string" ? v.robot : DEFAULT_RUN_OPTIONS.robot,
  };
}

/**
 * シナリオテストの選択・オプション・実行・停止・環境の操作と、表示する実行(最新または過去)の状態。
 * 選択は画面の切り替えをまたいで保持する。オプションは system_manager の設定に保存する。
 */
export function useScenarioRun(sysManager: SystemManagerHandle) {
  const { scenarios, status, runs, reloadScenarios, refresh } =
    useScenarioTest();
  const { callApi } = sysManager;

  const [selectedNames, setSelectedNames] = useOpsValue<string[]>(
    "scenario.selected",
    NO_SELECTION,
    false,
  );
  const selected = useMemo(() => new Set(selectedNames), [selectedNames]);
  const [options, setOptions] = useState<RunOptionsValue>(DEFAULT_RUN_OPTIONS);
  // null のときは最新の run を表示し続ける
  const [viewRunId, setViewRunId] = useState<string | null>(null);
  const [pastRun, setPastRun] = useState<RunProgress | null>(null);
  const [selectedEntry, setSelectedEntry] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    loadSettings().then((data) => setOptions(parseOptions(data[SETTINGS_KEY])));
  }, []);

  const updateOptions = (next: RunOptionsValue) => {
    setOptions(next);
    saveSettings(SETTINGS_KEY, next);
  };

  const latestRun = status?.run ?? null;
  const showingLatest = viewRunId === null || viewRunId === latestRun?.run_id;
  const run = showingLatest ? latestRun : pastRun;

  useEffect(() => {
    if (viewRunId === null || viewRunId === latestRun?.run_id) return;
    let cancelled = false;
    setPastRun(null);
    fetchRun(viewRunId)
      .then((r) => {
        if (!cancelled) setPastRun(r);
      })
      .catch(() => {
        if (!cancelled) setPastRun(null);
      });
    return () => {
      cancelled = true;
    };
  }, [viewRunId, latestRun?.run_id]);

  const selectedProgressEntry = useMemo(
    () => run?.entries.find((e) => e.result_dir === selectedEntry) ?? null,
    [run, selectedEntry],
  );

  const phase = runPhase(status);
  const envStatus = status?.services[SCENARIO_ENV_SERVICE] ?? null;
  const gazeboStatus = status?.services[GAZEBO_SERVICE] ?? null;
  const attachReady = envStatus === "running" || gazeboStatus === "running";
  const robotValid = isValidRobotAddress(options.robot);

  const startBlockedReason =
    phase !== "idle"
      ? "実行中です"
      : selected.size === 0
        ? "シナリオを選択してください"
        : !robotValid
          ? "実機PCのアドレスが不正です"
          : options.attach && !attachReady
            ? "attach には起動済みの環境が必要です"
            : null;

  const withBusy = async (action: () => Promise<void>) => {
    setBusy(true);
    setError(null);
    try {
      await action();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
      void refresh();
    }
  };

  const call = useCallback(
    async (path: string, body?: unknown) => {
      const result = await callApi(path, body);
      if (!result.success) throw new Error(result.message);
    },
    [callApi],
  );

  const start = () =>
    withBusy(async () => {
      const body: RunRequest = {
        // 一覧の並び順で実行する
        scenarios: [
          ...new Set(
            scenarios.filter((s) => selected.has(s.name)).map((s) => s.name),
          ),
        ],
        repeat: options.repeat,
        gui: options.gui && !options.attach,
        attach: options.attach,
        robot: options.attach ? "" : options.robot,
      };
      await call("/scenario/run/start", body);
      setViewRunId(null);
      setSelectedEntry(null);
    });

  const stop = () => withBusy(() => call("/scenario/run/stop"));
  const startEnv = (gui: boolean) =>
    withBusy(() => call("/scenario/env/start", { gui }));
  const stopEnv = () => withBusy(() => call("/scenario/env/stop"));

  const showLatest = () => {
    setViewRunId(null);
    setSelectedEntry(null);
  };
  const showRun = (runId: string) => {
    setViewRunId(runId);
    setSelectedEntry(null);
  };

  const phaseLabel =
    phase === "starting"
      ? "起動中…"
      : phase === "running"
        ? "実行中"
        : "待機中";

  return {
    scenarios,
    runs,
    reloadScenarios,
    selected,
    setSelected: (next: Set<string>) => setSelectedNames([...next]),
    options,
    updateOptions,
    run,
    latestRun,
    viewRunId,
    showingLatest,
    showLatest,
    showRun,
    selectedEntry,
    setSelectedEntry,
    selectedProgressEntry,
    phase,
    phaseLabel,
    envStatus,
    gazeboStatus,
    startBlockedReason,
    busy,
    error,
    start,
    stop,
    startEnv,
    stopEnv,
  };
}
