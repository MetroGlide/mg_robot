import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useSimulation } from "../../contexts/SimulationContext";
import { useRosbagReplay } from "../../contexts/RosbagReplayContext";
import { useOpsValue } from "../../hooks/useOpsValue";
import { useWaypointControl } from "../../hooks/useWaypointControl";
import {
  GOAL_BT_OPTIONS,
  GoalBtMode,
  isGoalBtMode,
} from "../../utils/waypointActions";
import ActionButton from "../ui/ActionButton";
import Card from "../ui/Card";
import Disclosure from "../ui/Disclosure";
import ActionResultText from "../waypoint-actions/ActionResultText";
import GenericServiceCaller from "../waypoint-actions/GenericServiceCaller";
import GenericTopicPublisher from "../waypoint-actions/GenericTopicPublisher";
import {
  LocalizationActions,
  MapActions,
} from "../waypoint-actions/WaypointActionsSection";
import RosbagReplaySection from "../sections/RosbagReplaySection";
import ContainerControl from "./ContainerControl";
import SimulationActions from "./SimulationActions";

const INPUT_CLASS =
  "rounded-md border border-line bg-surface-elevated px-2 py-1 text-xs text-content";

interface Props {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
  control: ReturnType<typeof useWaypointControl>;
  goalBt: GoalBtMode;
  onGoalBtChange: (mode: GoalBtMode) => void;
  connected: boolean;
}

/** 走行の補助操作(番号の指定、再読込、待ちなしの開始、手動ゴールの BT) */
function DriveTools({
  control,
  goalBt,
  onGoalBtChange,
  connected,
}: Omit<Props, "client" | "sysManager">) {
  const disabled = control.loading || !connected;
  return (
    <div className="space-y-3">
      <div className="flex flex-wrap items-end gap-2">
        <label className="text-xs text-muted">
          次の地点の番号
          <input
            type="number"
            min={0}
            value={control.jumpIndex}
            onChange={(e) => control.setJumpIndex(Number(e.target.value))}
            className={`mt-1 block w-20 ${INPUT_CLASS}`}
          />
        </label>
        <ActionButton label="Jump" size="sm" disabled={!connected} onClick={control.jump} />
        <ActionButton label="Reload WPs" size="sm" disabled={disabled} onClick={control.reload} />
        <ActionButton
          label="START IMMEDIATE"
          size="sm"
          variant="green"
          disabled={disabled}
          onClick={control.startImmediate}
        />
      </div>
      <label className="block text-xs text-muted">
        手動ゴールの BT
        <select
          value={goalBt}
          onChange={(e) => {
            if (isGoalBtMode(e.target.value)) onGoalBtChange(e.target.value);
          }}
          className={`mt-1 block w-full ${INPUT_CLASS}`}
        >
          {GOAL_BT_OPTIONS.map((option) => (
            <option key={option.value} value={option.value}>
              {option.label}
            </option>
          ))}
        </select>
      </label>
      <p className="text-xs text-muted">
        Queue wait は global_costmap のセンサ障害物レイヤーを無効にします。戻すときは Normal
        でゴールを送るか、Nav2 を再起動してください。シーケンスの走行中は拒否されます。
      </p>
      <ActionResultText result={control.goalRunner.result} />
    </div>
  );
}

/**
 * 走行の補助操作をまとめたカード。節ごとに開閉でき、開閉は画面を切り替えても保持する。
 * 走行中は操作しない運用なので、全部を閉じて、状態の表示だけを見られる。
 * シミュレーションと Rosbag 再生の節は、設定でそれぞれを有効にしたときだけ出す。
 */
export default function ActionsCard({ client, sysManager, ...drive }: Props) {
  const [open, setOpen] = useOpsValue<boolean>("actions.card", true);
  const { isSimulation } = useSimulation();
  const { isRosbagReplayVisible } = useRosbagReplay();

  return (
    <Card className="w-80 p-3">
      <button
        type="button"
        aria-expanded={open}
        onClick={() => setOpen(!open)}
        className="flex w-full items-center justify-between text-sm font-bold"
      >
        操作
        <span className="text-xs text-muted">{open ? "閉じる" : "開く"}</span>
      </button>
      {open && (
        <div className="mt-2 max-h-[calc(100vh-27rem)] space-y-2 overflow-y-auto">
          <Disclosure title="走行" storageKey="actions.drive">
            <DriveTools {...drive} />
          </Disclosure>
          <Disclosure title="自己位置 (AMCL / GNSS)" storageKey="actions.localization" defaultOpen>
            <LocalizationActions client={client} />
          </Disclosure>
          <Disclosure title="地図の切り替え" storageKey="actions.map">
            <MapActions client={client} />
          </Disclosure>
          <Disclosure title="サービスの呼び出し" storageKey="actions.service">
            <GenericServiceCaller client={client} />
          </Disclosure>
          <Disclosure title="トピックの publish" storageKey="actions.topic">
            <GenericTopicPublisher client={client} />
          </Disclosure>
          <Disclosure title="コンテナ" storageKey="actions.container">
            <ContainerControl
              sysManager={sysManager}
              container="navigation"
              title="Navigation"
            />
          </Disclosure>
          {isSimulation && (
            <Disclosure title="シミュレーション" storageKey="actions.simulation">
              <SimulationActions
                sysManager={sysManager}
                onResetAmcl={drive.control.publishInitialPose}
                showScenarioTest
              />
            </Disclosure>
          )}
          {isRosbagReplayVisible && (
            <Disclosure title="Rosbag 再生" storageKey="actions.rosbag">
              <RosbagReplaySection client={client} sysManager={sysManager} />
            </Disclosure>
          )}
        </div>
      )}
    </Card>
  );
}
