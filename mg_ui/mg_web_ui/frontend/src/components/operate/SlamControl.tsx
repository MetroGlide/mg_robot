import { useState } from "react";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useSystemManagerAction } from "../../hooks/useSystemManagerAction";
import Card from "../ui/Card";
import OpsButton from "../ui/OpsButton";
import Pill, { Tone } from "../ui/Pill";

const STATE_DISPLAY: Record<string, { label: string; tone: Tone }> = {
  running: { label: "起動中", tone: "ok" },
  exited: { label: "停止", tone: "neutral" },
  dead: { label: "異常終了", tone: "error" },
};

const INPUT_CLASS =
  "w-full rounded-md border border-line bg-surface-sunken px-2 py-1 text-xs text-content";

/** SLAM コンテナの起動・停止と、地図の保存 */
export default function SlamControl({
  sysManager,
}: {
  sysManager: SystemManagerHandle;
}) {
  const { call, loading, error } = useSystemManagerAction(sysManager);
  const [mapDir, setMapDir] = useState("/root/ros2_data");
  const [mapName, setMapName] = useState("map");

  const state = sysManager.containers["slam"];
  const display = state
    ? (STATE_DISPLAY[state] ?? { label: state, tone: "neutral" as Tone })
    : { label: "不明(system_manager に届いていません)", tone: "warn" as Tone };
  const running = state === "running";

  return (
    <Card className="w-[34rem] max-w-full p-4">
      <div className="mb-3 flex items-center justify-between">
        <h2 className="text-sm font-bold">SLAM</h2>
        <Pill tone={display.tone}>{display.label}</Pill>
      </div>
      <div className="flex flex-wrap gap-2">
        <OpsButton
          tone={running ? "neutral" : "primary"}
          disabled={loading}
          onClick={() => call(running ? "/slam/restart" : "/slam/start")}
        >
          {running ? "再起動" : "起動"}
        </OpsButton>
        <OpsButton tone="danger" disabled={loading || !running} onClick={() => call("/slam/stop")}>
          停止
        </OpsButton>
      </div>
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
