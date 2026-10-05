import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useLocalizationGates } from "../../hooks/useLocalizationGates";
import { useThrottledTopic } from "../../hooks/useThrottledTopic";
import { useTopicSubscriber } from "../../hooks/useTopicSubscriber";
import { StringMsg } from "../../types";
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

/** ロボットの姿勢と、自己位置推定の入力(AMCL・GNSS)の入切、走行モード */
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
    </Card>
  );
}
