import { useState } from "react";
import ActionButton from "../ui/ActionButton";
import StatusBadge from "../ui/StatusBadge";

interface ScenarioEnvCardProps {
  envStatus: string | null;
  gazeboStatus: string | null;
  onStart: (gui: boolean) => void;
  onStop: () => void;
  busy: boolean;
}

/** attach モード用の環境(scenario-env: シミュレータとナビゲーション)の起動・停止 */
export default function ScenarioEnvCard({
  envStatus,
  gazeboStatus,
  onStart,
  onStop,
  busy,
}: ScenarioEnvCardProps) {
  const [gui, setGui] = useState(true);
  const running = envStatus === "running";

  return (
    <div className="space-y-2 text-sm">
      <div className="flex items-center justify-between">
        <StatusBadge status={envStatus ?? "unknown"} />
        <div className="flex gap-2">
          {running ? (
            <>
              <ActionButton
                label="再起動"
                size="sm"
                variant="yellow"
                onClick={() => onStart(gui)}
                loading={busy}
              />
              <ActionButton
                label="停止"
                size="sm"
                variant="red"
                onClick={onStop}
                loading={busy}
              />
            </>
          ) : (
            <ActionButton
              label="起動"
              size="sm"
              variant="green"
              onClick={() => onStart(gui)}
              loading={busy}
            />
          )}
        </div>
      </div>
      <label className="flex items-center gap-2 text-xs text-gray-300 select-none">
        <input
          type="checkbox"
          checked={gui}
          onChange={(e) => setGui(e.target.checked)}
        />
        GUI を表示して起動する
      </label>
      <p className="text-xs text-gray-400">
        プロファイルのワールドと地図でシミュレータとナビゲーションを起動したままにする。
        起動完了まで 1〜2
        分かかる。途中で停止したシナリオの障害物が残った場合は再起動する。
      </p>
      {gazeboStatus === "running" && !running && (
        <p className="text-xs text-yellow-300">
          gazebo-simulation が動いています。attach
          はそのまま使えますが、この環境を起動するには先に停止してください。
        </p>
      )}
    </div>
  );
}
