import { useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useOpsValue } from "../../hooks/useOpsValue";
import OperateLayout from "../../components/operate/OperateLayout";
import OperateMap from "../../components/operate/OperateMap";
import KpiRow from "../../components/operate/KpiRow";
import MapToolbar from "../../components/operate/MapToolbar";
import HealthTabsCard from "../../components/operate/HealthTabsCard";
import SlamControl from "../../components/operate/SlamControl";
import { MapCommand } from "../../components/operate/MapCameraControls";
import {
  OverlayCards,
  OverlayToggleButtons,
} from "../../components/operate/MapOverlays";

const VIEW_KEY = "slam";

/** SLAM(slam_toolbox)の運用ビュー。地図づくりの様子を見ながら、SLAM の起動・停止と地図の保存を行う */
export default function SlamOperate({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [command, setCommand] = useState<MapCommand | null>(null);
  const [follow, setFollow] = useOpsValue<boolean>(`${VIEW_KEY}.follow`, true);

  const issueCommand = (kind: MapCommand["kind"]) =>
    setCommand((prev) => ({ kind, n: (prev?.n ?? 0) + 1 }));

  return (
    <OperateLayout
      map={
        <OperateMap
          viewKey={VIEW_KEY}
          client={client}
          command={command}
          follow={follow}
          onUserPan={() => setFollow(false)}
          interactionMode="none"
          onPoseSet={() => {}}
          showNavLayers={false}
        />
      }
      topLeft={<KpiRow client={client} />}
      toolbar={
        <MapToolbar
          onCommand={issueCommand}
          follow={follow}
          onToggleFollow={() => setFollow(!follow)}
          disabled={client.status !== "connected"}
          extra={<OverlayToggleButtons disabled={client.status !== "connected"} />}
        />
      }
      bottomLeft={
        <div className="flex flex-col items-start gap-2">
          <OverlayCards client={client} />
          <SlamControl client={client} sysManager={sysManager} />
        </div>
      }
      bottomRight={<HealthTabsCard client={client} />}
    />
  );
}
