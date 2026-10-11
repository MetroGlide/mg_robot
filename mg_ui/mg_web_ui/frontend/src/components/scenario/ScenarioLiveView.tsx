import { useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useOpsValue } from "../../hooks/useOpsValue";
import OperateLayout from "../operate/OperateLayout";
import OperateMap from "../operate/OperateMap";
import MapToolbar from "../operate/MapToolbar";
import { MapCommand } from "../operate/MapCameraControls";

const VIEW_KEY = "scenarioLive";

/**
 * シナリオテスト中のライブ表示。運用ビューと同じ地図と地図ツールバーだけを出す。
 * ロボットの状態や診断などの詳細は、Waypoint などの運用ビューで確認する。
 */
export default function ScenarioLiveView({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const [follow, setFollow] = useOpsValue<boolean>(`${VIEW_KEY}.follow`, true);
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
          />
        }
        toolbar={
          <MapToolbar
            onCommand={issueCommand}
            follow={follow}
            onToggleFollow={() => setFollow(!follow)}
            disabled={client.status !== "connected"}
          />
        }
      />
    </div>
  );
}
