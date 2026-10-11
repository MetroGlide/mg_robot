import { useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useOpsValue } from "../../hooks/useOpsValue";
import OperateLayout from "../operate/OperateLayout";
import OperateMap from "../operate/OperateMap";
import KpiRow from "../operate/KpiRow";
import DiagnosticsBoard from "../operate/DiagnosticsBoard";
import RobotDetailCard from "../operate/RobotDetailCard";
import TogglePanel from "../operate/TogglePanel";
import { MapCommand } from "../operate/MapCameraControls";

const VIEW_KEY = "scenarioLive";

/**
 * シナリオテスト中のライブ表示。運用ビューと同じ地図とパネルを使い、操作のパネル
 * (ツールバー・走行の操作・ジョイスティックなど)は出さずに、状態の表示だけにする。
 */
export default function ScenarioLiveView({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const [follow, setFollow] = useOpsValue<boolean>(`${VIEW_KEY}.follow`, true);
  // 地図の操作がないので、コマンドは送らない
  const [command] = useState<MapCommand | null>(null);

  return (
    <div className="h-[36rem] overflow-hidden rounded-lg border border-line">
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
        topLeft={<KpiRow client={client} />}
        topRight={
          <TogglePanel id="robotDetail">
            {(close) => <RobotDetailCard client={client} onClose={close} />}
          </TogglePanel>
        }
        bottomRight={
          <TogglePanel id="diagnostics">
            {(close) => <DiagnosticsBoard client={client} onClose={close} />}
          </TogglePanel>
        }
      />
    </div>
  );
}
