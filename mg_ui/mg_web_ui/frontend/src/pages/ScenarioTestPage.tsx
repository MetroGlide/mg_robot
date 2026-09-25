import { ReactNode, useEffect, useMemo, useState } from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import { useDockerLogStream } from "../hooks/useDockerLogStream";
import { useScenarioTest } from "../hooks/useScenarioTest";
import ServiceLogPanel from "../components/panels/ServiceLogPanel";
import RosViewer from "../components/ros-viewer/RosViewer";
import ActionButton from "../components/ui/ActionButton";
import Toggle from "../components/ui/Toggle";
import ScenarioSelector from "../components/scenario/ScenarioSelector";
import RunOptionsForm, {
  DEFAULT_RUN_OPTIONS,
  RunOptionsValue,
} from "../components/scenario/RunOptionsForm";
import ScenarioEnvCard from "../components/scenario/ScenarioEnvCard";
import RunProgressTable from "../components/scenario/RunProgressTable";
import ScenarioResultDetail from "../components/scenario/ScenarioResultDetail";
import RunHistoryList from "../components/scenario/RunHistoryList";
import type { RunProgress, RunRequest } from "../types/scenarioTest";
import { fetchRun } from "../utils/scenarioTestApi";
import { loadSettings, saveSettings } from "../utils/settingsApi";
import {
  GAZEBO_SERVICE,
  isValidRobotAddress,
  runPhase,
  SCENARIO_ENV_SERVICE,
  SCENARIO_TEST_SERVICE,
} from "../utils/scenarioTest";

const SETTINGS_KEY = "scenario_test";
const LOG_SERVICES = [SCENARIO_TEST_SERVICE, SCENARIO_ENV_SERVICE];
const LOG_SERVICE_SET = new Set(LOG_SERVICES);

function Panel({
  title,
  actions,
  children,
}: {
  title: string;
  actions?: ReactNode;
  children: ReactNode;
}) {
  return (
    <section className="bg-gray-800 rounded-lg p-4 space-y-3">
      <div className="flex items-center justify-between gap-2">
        <h2 className="text-sm font-medium text-gray-300">{title}</h2>
        {actions}
      </div>
      {children}
    </section>
  );
}

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

export default function ScenarioTestPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const { scenarios, status, runs, reloadScenarios, refresh } =
    useScenarioTest();
  const { callApi } = sysManager;
  const { entries, connected, clear } = useDockerLogStream(LOG_SERVICES);

  const [selected, setSelected] = useState<Set<string>>(new Set());
  const [options, setOptions] = useState<RunOptionsValue>(DEFAULT_RUN_OPTIONS);
  // null のときは最新の run を表示し続ける
  const [viewRunId, setViewRunId] = useState<string | null>(null);
  const [pastRun, setPastRun] = useState<RunProgress | null>(null);
  const [selectedEntry, setSelectedEntry] = useState<string | null>(null);
  const [liveView, setLiveView] = useState(false);
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

  const call = async (path: string, body?: unknown) => {
    const result = await callApi(path, body);
    if (!result.success) throw new Error(result.message);
  };

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

  const phaseLabel =
    phase === "starting"
      ? "起動中…"
      : phase === "running"
        ? "実行中"
        : "待機中";

  return (
    <div className="grid grid-cols-1 gap-4 xl:grid-cols-3">
      <div className="space-y-4">
        <Panel title="シナリオ">
          <ScenarioSelector
            scenarios={scenarios}
            selected={selected}
            onChange={setSelected}
            onReload={() => void reloadScenarios()}
            disabled={phase !== "idle"}
          />
        </Panel>

        <Panel title="実行">
          <RunOptionsForm
            value={options}
            onChange={updateOptions}
            disabled={phase !== "idle"}
          />
          <div className="flex items-center gap-2">
            {phase === "idle" ? (
              <ActionButton
                label="実行"
                variant="green"
                onClick={() => void start()}
                disabled={startBlockedReason !== null}
                loading={busy}
              />
            ) : (
              <ActionButton
                label="停止"
                variant="red"
                onClick={() => void stop()}
                loading={busy}
              />
            )}
            <span className="text-xs text-gray-400">
              {startBlockedReason && phase === "idle"
                ? startBlockedReason
                : phaseLabel}
            </span>
          </div>
          {error && <p className="text-xs text-red-400 break-all">{error}</p>}
        </Panel>

        <Panel title="起動済み環境(attach 用)">
          <ScenarioEnvCard
            envStatus={envStatus}
            gazeboStatus={gazeboStatus}
            onStart={(gui) => void startEnv(gui)}
            onStop={() => void stopEnv()}
            busy={busy}
          />
        </Panel>
      </div>

      <div className="space-y-4 xl:col-span-2">
        <Panel
          title="実行状況"
          actions={
            !showingLatest && (
              <button
                type="button"
                onClick={() => {
                  setViewRunId(null);
                  setSelectedEntry(null);
                }}
                className="text-xs text-blue-300 hover:text-blue-200"
              >
                最新の実行に戻る
              </button>
            )
          }
        >
          {phase === "starting" && showingLatest && (
            <p className="text-xs text-blue-300">テストを起動しています…</p>
          )}
          {run ? (
            <RunProgressTable
              run={run}
              selectedEntry={selectedEntry}
              onSelectEntry={setSelectedEntry}
            />
          ) : (
            <p className="text-xs text-gray-500">実行結果はまだありません</p>
          )}
        </Panel>

        {run && selectedProgressEntry && (
          <Panel title="シナリオの結果">
            <ScenarioResultDetail
              runId={run.run_id}
              entry={selectedProgressEntry}
              hasStackLog={Boolean(run.options.remote_stack)}
            />
          </Panel>
        )}

        <Panel
          title="ライブ表示"
          actions={
            <Toggle value={liveView} onChange={() => setLiveView((v) => !v)} />
          }
        >
          {liveView ? (
            <div className="h-[28rem] overflow-hidden rounded-lg border border-gray-700">
              <RosViewer client={client} className="w-full h-full" />
            </div>
          ) : (
            <p className="text-xs text-gray-500">
              オンにすると、接続中の foxglove_bridge
              から地図・ロボット・経路を表示する(シミュレータと同じ PC の bridge
              に接続しているときに使える)。
            </p>
          )}
        </Panel>

        <ServiceLogPanel
          entries={entries}
          displayServices={LOG_SERVICE_SET}
          connected={connected}
          receivingServices={LOG_SERVICE_SET}
          onClear={clear}
        />

        <Panel title="実行履歴">
          <RunHistoryList
            runs={runs}
            selectedRunId={viewRunId ?? latestRun?.run_id ?? null}
            onSelect={(runId) => {
              setViewRunId(runId);
              setSelectedEntry(null);
            }}
          />
        </Panel>
      </div>
    </div>
  );
}
