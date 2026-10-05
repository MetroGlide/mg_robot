import { useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useWaypointControl } from "../../hooks/useWaypointControl";
import Card from "../../components/ui/Card";
import OperateLayout from "../../components/operate/OperateLayout";
import OperateMap from "../../components/operate/OperateMap";
import KpiRow from "../../components/operate/KpiRow";
import MapToolbar from "../../components/operate/MapToolbar";
import HealthTabsCard from "../../components/operate/HealthTabsCard";
import RobotDetailCard from "../../components/operate/RobotDetailCard";
import WaypointProgress from "../../components/operate/WaypointProgress";
import { MapCommand } from "../../components/operate/MapCameraControls";

/** Waypoint 走行の運用ビュー */
export default function WaypointOperate({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const [command, setCommand] = useState<MapCommand | null>(null);
  const [follow, setFollow] = useState(false);
  const [detailOpen, setDetailOpen] = useState(true);
  const [mapError, setMapError] = useState<string | null>(null);
  const control = useWaypointControl(client, "default", setMapError);
  const connected = client.status === "connected";

  const issueCommand = (kind: MapCommand["kind"]) =>
    setCommand((prev) => ({ kind, n: (prev?.n ?? 0) + 1 }));

  const goalResult = control.goalRunner.result;
  const notice = mapError ?? goalResult?.text ?? null;
  const noticeIsError = mapError !== null || goalResult?.ok === false;

  return (
    <OperateLayout
      map={
        <OperateMap
          client={client}
          command={command}
          follow={follow}
          onUserPan={() => setFollow(false)}
          interactionMode={control.interactionMode}
          onPoseSet={control.handleMapPoseSet}
        />
      }
      topLeft={<KpiRow client={client} />}
      toolbar={
        <MapToolbar
          onCommand={issueCommand}
          follow={follow}
          onToggleFollow={() => setFollow((v) => !v)}
          interactionMode={control.interactionMode}
          onInteractionModeChange={control.setInteractionMode}
          disabled={!connected}
        />
      }
      topRight={
        detailOpen ? (
          <RobotDetailCard client={client} onClose={() => setDetailOpen(false)} />
        ) : (
          <button
            type="button"
            onClick={() => setDetailOpen(true)}
            className="rounded-lg border border-line bg-surface-elevated px-3 py-1.5 text-xs font-semibold shadow-card"
          >
            ロボットの詳細
          </button>
        )
      }
      notice={
        notice && (
          <Card className="px-3 py-1.5">
            <p className={`text-xs ${noticeIsError ? "text-error" : "text-ok"}`}>
              {notice}
            </p>
          </Card>
        )
      }
      bottomLeft={
        <WaypointProgress client={client} control={control} connected={connected} />
      }
      bottomRight={<HealthTabsCard client={client} />}
    />
  );
}
