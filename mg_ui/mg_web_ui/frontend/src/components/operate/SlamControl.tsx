import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useSimulation } from "../../contexts/SimulationContext";
import { useRosbagReplay } from "../../contexts/RosbagReplayContext";
import { useOpsValue } from "../../hooks/useOpsValue";
import { useSystemManagerAction } from "../../hooks/useSystemManagerAction";
import Card from "../ui/Card";
import OpsButton from "../ui/OpsButton";
import RosbagReplaySection from "../sections/RosbagReplaySection";
import Disclosure from "../ui/Disclosure";
import ContainerControl from "./ContainerControl";
import SimulationActions from "./SimulationActions";

const INPUT_CLASS =
  "w-full rounded-md border border-line bg-surface-sunken px-2 py-1 text-xs text-content";

/**
 * SLAM コンテナの起動・停止と、地図の保存。
 * シミュレーションと Rosbag 再生の節は、設定でそれぞれを有効にしたときだけ出す。
 */
export default function SlamControl({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const { isSimulation } = useSimulation();
  const { isRosbagReplayVisible } = useRosbagReplay();
  const { call, loading, error } = useSystemManagerAction(sysManager);
  const [mapDir, setMapDir] = useOpsValue("slam.mapDir", "/root/ros2_data");
  const [mapName, setMapName] = useOpsValue("slam.mapName", "map");

  const running = sysManager.containers["slam"] === "running";

  return (
    <Card className="w-[34rem] max-w-full p-4">
      <ContainerControl sysManager={sysManager} container="slam" title="SLAM" />
      <div className="mt-3 grid grid-cols-[1fr_8rem_auto] items-end gap-2">
        <label className="text-xs text-muted">
          保存先
          <input
            type="text"
            value={mapDir}
            onChange={(e) => setMapDir(e.target.value)}
            className={INPUT_CLASS}
          />
        </label>
        <label className="text-xs text-muted">
          地図名
          <input
            type="text"
            value={mapName}
            onChange={(e) => setMapName(e.target.value)}
            className={INPUT_CLASS}
          />
        </label>
        <OpsButton
          tone="primary"
          disabled={loading || !running}
          onClick={() => call("/map/common/save", { map_dir: mapDir, map_name: mapName })}
        >
          地図を保存
        </OpsButton>
      </div>
      {error && <p className="mt-2 text-xs text-error">{error}</p>}
      {(isSimulation || isRosbagReplayVisible) && (
        <div className="mt-3 max-h-64 space-y-2 overflow-y-auto">
          {isSimulation && (
            <Disclosure title="シミュレーション" storageKey="slam.simulation">
              <SimulationActions sysManager={sysManager} />
            </Disclosure>
          )}
          {isRosbagReplayVisible && (
            <Disclosure title="Rosbag 再生" storageKey="slam.rosbag">
              <RosbagReplaySection client={client} sysManager={sysManager} />
            </Disclosure>
          )}
        </div>
      )}
    </Card>
  );
}
