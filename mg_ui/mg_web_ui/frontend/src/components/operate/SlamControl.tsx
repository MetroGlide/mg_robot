import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useOpsValue } from "../../hooks/useOpsValue";
import { useSystemManagerAction } from "../../hooks/useSystemManagerAction";
import Card from "../ui/Card";
import OpsButton from "../ui/OpsButton";
import ContainerControl from "./ContainerControl";

const INPUT_CLASS =
  "w-full rounded-md border border-line bg-surface-sunken px-2 py-1 text-xs text-content";

/** SLAM コンテナの起動・停止と、地図の保存 */
export default function SlamControl({
  sysManager,
}: {
  sysManager: SystemManagerHandle;
}) {
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
          onClick={() => call("/map/save", { map_dir: mapDir, map_name: mapName })}
        >
          地図を保存
        </OpsButton>
      </div>
      {error && <p className="mt-2 text-xs text-error">{error}</p>}
    </Card>
  );
}
