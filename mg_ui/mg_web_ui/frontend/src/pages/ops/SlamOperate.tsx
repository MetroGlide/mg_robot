import { useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import OperateLayout from "../../components/operate/OperateLayout";
import OperateMap from "../../components/operate/OperateMap";
import KpiRow from "../../components/operate/KpiRow";
import MapToolbar from "../../components/operate/MapToolbar";
import HealthTabsCard from "../../components/operate/HealthTabsCard";
import SlamControl from "../../components/operate/SlamControl";
import { MapCommand } from "../../components/operate/MapCameraControls";

/** SLAM(slam_toolbox)の運用ビュー。地図づくりの様子を見ながら、SLAM の起動・停止と地図の保存を行う */
export default function SlamOperate({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [command, setCommand] = useState<MapCommand | null>(null);
  const [follow, setFollow] = useState(true);

  const issueCommand = (kind: MapCommand["kind"]) =>
    setCommand((prev) => ({ kind, n: (prev?.n ?? 0) + 1 }));

  return (
    <OperateLayout
      map={
        <OperateMap
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
          onToggleFollow={() => setFollow((v) => !v)}
        />
      }
      bottomLeft={<SlamControl sysManager={sysManager} />}
      bottomRight={<HealthTabsCard client={client} />}
    />
  );
}
