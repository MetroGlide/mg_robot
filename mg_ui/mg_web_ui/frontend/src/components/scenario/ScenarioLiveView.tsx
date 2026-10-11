import { useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useOpsValue } from "../../hooks/useOpsValue";
import { useSensorsVisible } from "../../hooks/useSensorsVisible";
import OperateLayout from "../operate/OperateLayout";
import OperateMap from "../operate/OperateMap";
import MapToolbar from "../operate/MapToolbar";
import IconButton from "../ui/IconButton";
import { LayersIcon } from "../ui/icons";
import { MapCommand } from "../operate/MapCameraControls";

const VIEW_KEY = "scenarioLive";

/**
 * シナリオテスト中のライブ表示。運用ビューと同じ地図と地図ツールバー(センサ表示の切り替えを含む)だけを出す。
 * ロボットの状態や診断などの詳細は、Waypoint などの運用ビューで確認する。
 */
export default function ScenarioLiveView({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const [follow, setFollow] = useOpsValue<boolean>(`${VIEW_KEY}.follow`, true);
  const [sensorsVisible, setSensorsVisible] = useSensorsVisible();
  const [command, setCommand] = useState<MapCommand | null>(null);
  const issueCommand = (kind: MapCommand["kind"]) =>
    setCommand((prev) => ({ kind, n: (prev?.n ?? 0) + 1 }));

  return (
    <div className="h-[30rem] overflow-hidden rounded-lg border border-line">
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
            showSensors={sensorsVisible}
          />
        }
        toolbar={
          <MapToolbar
            onCommand={issueCommand}
            follow={follow}
            onToggleFollow={() => setFollow(!follow)}
            disabled={client.status !== "connected"}
            extra={
              <IconButton
                icon={<LayersIcon />}
                title="センサ表示をまとめて切り替え(内容はセンサビューのレイヤーで選ぶ)"
                active={sensorsVisible}
                onClick={() => setSensorsVisible(!sensorsVisible)}
              />
            }
          />
        }
      />
    </div>
  );
}
