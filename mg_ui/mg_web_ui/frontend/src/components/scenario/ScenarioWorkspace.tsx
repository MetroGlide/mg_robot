import { useMemo, useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useDockerLogStream } from "../../hooks/useDockerLogStream";
import { useScenarioRun } from "../../hooks/useScenarioRun";
import ServiceLogPanel from "../panels/ServiceLogPanel";
import ActionButton from "../ui/ActionButton";
import Toggle from "../ui/Toggle";
import Panel from "../work/Panel";
import ScenarioSelector from "./ScenarioSelector";
import RunOptionsForm from "./RunOptionsForm";
import ScenarioEnvCard from "./ScenarioEnvCard";
import RunProgressTable from "./RunProgressTable";
import PlannedRunList from "./PlannedRunList";
import ScenarioLiveView from "./ScenarioLiveView";
import ScenarioResultDetail from "./ScenarioResultDetail";
import RunHistoryList from "./RunHistoryList";
import {
  descriptionsByName,
  SCENARIO_ENV_SERVICE,
  SCENARIO_TEST_SERVICE,
} from "../../utils/scenarioTest";

const LOG_SERVICES = [SCENARIO_TEST_SERVICE, SCENARIO_ENV_SERVICE];
const LOG_SERVICE_SET = new Set(LOG_SERVICES);

/** シナリオテストの作業画面の本体。選択・オプション・実行状況・結果・ログ・履歴を並べる。旧ページと新 UI の作業ビューで共有する */
export default function ScenarioWorkspace({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const s = useScenarioRun(sysManager);
  const { entries, connected, clear } = useDockerLogStream(LOG_SERVICES);
  const [liveView, setLiveView] = useState(false);
  const { run, phase } = s;
  const descriptions = useMemo(
    () => descriptionsByName(s.scenarios),
    [s.scenarios],
  );

  return (
    <div className="grid grid-cols-1 gap-4 xl:grid-cols-3">
      <div className="space-y-4">
        <Panel title="シナリオ">
          <ScenarioSelector
            scenarios={s.scenarios}
            selected={s.selected}
            onChange={s.setSelected}
            onReload={() => void s.reloadScenarios()}
            disabled={phase !== "idle"}
          />
        </Panel>

        <Panel title="実行">
          <RunOptionsForm
            value={s.options}
            onChange={s.updateOptions}
            disabled={phase !== "idle"}
          />
          <div className="flex items-center gap-2">
            {phase === "idle" ? (
              <ActionButton
                label="実行"
                variant="green"
                onClick={() => void s.start()}
                disabled={s.startBlockedReason !== null}
                loading={s.busy}
              />
            ) : (
              <ActionButton
                label="停止"
                variant="red"
                onClick={() => void s.stop()}
                loading={s.busy}
              />
            )}
            <span className="text-xs text-muted">
              {s.startBlockedReason && phase === "idle"
                ? s.startBlockedReason
                : s.phaseLabel}
            </span>
          </div>
          {s.error && <p className="break-all text-xs text-error">{s.error}</p>}
        </Panel>

        <Panel title="起動済み環境(attach 用)">
          <ScenarioEnvCard
            envStatus={s.envStatus}
            gazeboStatus={s.gazeboStatus}
            onStart={(gui) => void s.startEnv(gui)}
            onStop={() => void s.stopEnv()}
            busy={s.busy}
          />
        </Panel>
      </div>

      <div className="space-y-4 xl:col-span-2">
        {phase === "running" && !s.showingLatest && s.latestRun && (
          <Panel
            title="実行中のテスト"
            actions={
              <button
                type="button"
                onClick={s.showLatest}
                className="text-xs text-accent hover:opacity-80"
              >
                この実行を表示
              </button>
            }
          >
            <RunProgressTable
              run={s.latestRun}
              selectedEntry={null}
              onSelectEntry={(dir) => {
                s.showLatest();
                s.setSelectedEntry(dir);
              }}
              descriptions={descriptions}
            />
          </Panel>
        )}

        <Panel
          title={s.showingLatest ? "実行状況" : "過去の実行"}
          actions={
            !s.showingLatest && (
              <button
                type="button"
                onClick={s.showLatest}
                className="text-xs text-accent hover:opacity-80"
              >
                最新の実行に戻る
              </button>
            )
          }
        >
          {phase === "starting" && s.showingLatest ? (
            s.plannedNames.length > 0 ? (
              <PlannedRunList
                names={s.plannedNames}
                repeat={s.options.repeat}
                descriptions={descriptions}
              />
            ) : (
              <p className="text-xs text-accent">テストを起動しています…</p>
            )
          ) : run ? (
            <RunProgressTable
              run={run}
              selectedEntry={s.selectedEntry}
              onSelectEntry={s.setSelectedEntry}
              descriptions={descriptions}
            />
          ) : (
            <p className="text-xs text-muted">実行結果はまだありません</p>
          )}
        </Panel>

        {run && s.selectedProgressEntry && (
          <Panel title="シナリオの結果">
            <ScenarioResultDetail
              runId={run.run_id}
              entry={s.selectedProgressEntry}
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
            <ScenarioLiveView client={client} />
          ) : (
            <p className="text-xs text-muted">
              オンにすると、接続中の foxglove_bridge
              から地図・ロボット・経路を表示する(シミュレータと同じ PC の bridge
              に接続しているときに使える)。ロボットの状態や診断は Waypoint などのタブで確認する。
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
            runs={s.runs}
            selectedRunId={s.viewRunId ?? s.latestRun?.run_id ?? null}
            onSelect={s.showRun}
          />
        </Panel>
      </div>
    </div>
  );
}
