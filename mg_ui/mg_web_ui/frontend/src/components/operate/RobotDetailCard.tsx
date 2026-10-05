import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useLocalizationGates } from "../../hooks/useLocalizationGates";
import { useThrottledTopic } from "../../hooks/useThrottledTopic";
import { useTopicSubscriber } from "../../hooks/useTopicSubscriber";
import {
  Nav2LifecycleResult,
  useNav2LifecycleCheck,
} from "../../hooks/useNav2LifecycleCheck";
import {
  CollisionDetectorState,
  GOAL_STATUS,
  GoalStatusArray,
  StringMsg,
} from "../../types";
import { TOPICS } from "../../ros/interfaces";
import { parseNavigationMode } from "../../utils/waypointActions";
import Card from "../ui/Card";
import IconButton from "../ui/IconButton";
import { CloseIcon } from "../ui/icons";
import { quaternionToYawDeg } from "./displayState";

interface AmclPose {
  pose: {
    pose: {
      position: { x: number; y: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
}

const AMCL_HZ = 1;
// 衝突検知の状態は collision_detector の周期で届く。表示は 2Hz で足りる
const COLLISION_HZ = 2;

function Row({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex justify-between gap-4 py-1 text-xs">
      <span className="whitespace-nowrap text-muted">{label}</span>
      <span className="text-right font-semibold tabular-nums">{value}</span>
    </div>
  );
}

function onOff(value: boolean | null): string {
  if (value === null) return "--";
  return value ? "ON" : "OFF";
}

function activeText(value: boolean | null): string {
  if (value === null) return "応答なし";
  return value ? "Active" : "Inactive";
}

function lifecycleText(result: Nav2LifecycleResult | null): string {
  if (!result) return "未確認";
  return `nav ${activeText(result.navigation)} / loc ${activeText(result.localization)}`;
}

/** 衝突検知のポリゴンごとの検知状態。検知中のポリゴンを赤で示す */
function CollisionRow({ state }: { state: CollisionDetectorState | null }) {
  return (
    <div className="flex justify-between gap-4 py-1 text-xs">
      <span className="whitespace-nowrap text-muted">衝突検知</span>
      <span className="flex flex-wrap justify-end gap-1">
        {!state && <span className="font-semibold">--</span>}
        {state?.polygons.map((name, i) => (
          <span
            key={name}
            className={`rounded px-1.5 py-0.5 font-semibold ${
              state.detections[i]
                ? "bg-error text-surface-elevated"
                : "bg-surface-sunken text-muted"
            }`}
          >
            {name}
          </span>
        ))}
      </span>
    </div>
  );
}

/**
 * ロボットの姿勢と、自己位置推定の入力(AMCL・GNSS)の入切、走行モード、Nav2 と衝突検知の状態。
 * Nav2 のライフサイクルは、ボタンを押したときだけ調べる(定期的なサービス呼び出しをしない)。
 */
export default function RobotDetailCard({
  client,
  onClose,
}: {
  client: FoxgloveClientHandle;
  onClose: () => void;
}) {
  const amcl = useThrottledTopic<AmclPose>(
    client,
    TOPICS.AMCL_POSE,
    "geometry_msgs/msg/PoseWithCovarianceStamped",
    AMCL_HZ,
  );
  const gates = useLocalizationGates(client);
  const modeMsg = useTopicSubscriber<StringMsg>(
    client,
    TOPICS.WAYPOINT_NAVIGATION_MODE,
    "std_msgs/msg/String",
  );
  const mode = modeMsg ? parseNavigationMode(modeMsg.data) : null;
  // ゴールの状態が変わったときだけ届くので、間引かない
  const actionStatusMsg = useTopicSubscriber<GoalStatusArray>(
    client,
    TOPICS.NAV_ACTION_STATUS,
    "action_msgs/msg/GoalStatusArray",
  );
  const statusList = actionStatusMsg?.status_list ?? [];
  const actionStatus =
    statusList.length > 0 ? statusList[statusList.length - 1].status : null;
  const collision = useThrottledTopic<CollisionDetectorState>(
    client,
    TOPICS.COLLISION_STATE,
    "nav2_msgs/msg/CollisionDetectorState",
    COLLISION_HZ,
  );
  const lifecycle = useNav2LifecycleCheck(client);
  const pose = amcl?.pose.pose;

  return (
    <Card className="w-72 p-3">
      <div className="mb-1 flex items-center justify-between">
        <h2 className="text-sm font-bold">MG-01</h2>
        <IconButton icon={<CloseIcon />} title="閉じる" onClick={onClose} />
      </div>
      <Row label="X" value={pose ? `${pose.position.x.toFixed(2)} m` : "--"} />
      <Row label="Y" value={pose ? `${pose.position.y.toFixed(2)} m` : "--"} />
      <Row
        label="向き"
        value={pose ? `${quaternionToYawDeg(pose.orientation).toFixed(0)}°` : "--"}
      />
      <div className="my-1 h-px bg-line" />
      <Row label="AMCL" value={onOff(gates.amcl ? gates.amcl.applied : null)} />
      <Row label="GNSS" value={onOff(gates.gnssPublishing)} />
      <Row
        label="走行モード"
        value={mode ? `${mode.mode} (${mode.behavior_tree})` : "--"}
      />
      <div className="my-1 h-px bg-line" />
      <Row
        label="Nav2 のゴール"
        value={
          actionStatus !== null ? (GOAL_STATUS[actionStatus] ?? String(actionStatus)) : "--"
        }
      />
      <div className="flex items-center justify-between gap-2 py-1 text-xs">
        <span className="whitespace-nowrap text-muted">ライフサイクル</span>
        <span
          className="truncate text-right font-semibold"
          title={lifecycle.result ? lifecycle.result.checkedAt.toLocaleTimeString() : undefined}
        >
          {lifecycleText(lifecycle.result)}
        </span>
        <button
          type="button"
          disabled={lifecycle.checking || client.status !== "connected"}
          onClick={() => void lifecycle.check()}
          className="shrink-0 rounded-md bg-surface-sunken px-2 py-0.5 font-semibold hover:bg-line disabled:opacity-40"
        >
          確認
        </button>
      </div>
      <CollisionRow state={collision} />
    </Card>
  );
}
