import { useState } from "react";
import { PoseInput } from "../../hooks/useWaypointControl";
import OpsButton from "../ui/OpsButton";

interface SimulationPoseSectionProps {
  onResetRobot: (pose: PoseInput) => void;
  onResetAmcl?: (pose: PoseInput) => void;
  loading?: boolean;
  error?: string | null;
}

const INPUT_CLASS =
  "mt-1 block w-full rounded-md border border-line bg-surface-elevated px-2 py-1 text-xs text-content";

/** シミュレータ上のロボットの姿勢と、AMCL の姿勢を指定した値に戻す。枠は呼び出し側で付ける */
export default function SimulationPoseSection({
  onResetRobot,
  onResetAmcl,
  loading = false,
  error,
}: SimulationPoseSectionProps) {
  const [pose, setPose] = useState<PoseInput>({
    x: 0.0,
    y: 0.0,
    z: 0.05,
    yaw: 0.0,
  });

  const update = (key: keyof PoseInput, value: number) =>
    setPose((prev) => ({ ...prev, [key]: value }));

  return (
    <div className="space-y-2">
      <div className="grid grid-cols-4 gap-2">
        {(["x", "y", "z", "yaw"] as const).map((k) => (
          <label key={k} className="text-xs text-muted">
            {k === "yaw" ? "yaw (rad)" : `${k} (m)`}
            <input
              type="number"
              step={k === "z" ? "0.01" : "0.1"}
              value={pose[k]}
              onChange={(e) => update(k, Number(e.target.value))}
              className={INPUT_CLASS}
            />
          </label>
        ))}
      </div>
      <div className="flex flex-wrap gap-2">
        <OpsButton tone="primary" disabled={loading} onClick={() => onResetRobot(pose)}>
          ロボットの姿勢をリセット
        </OpsButton>
        {onResetAmcl && (
          <OpsButton disabled={loading} onClick={() => onResetAmcl(pose)}>
            AMCL の姿勢をリセット
          </OpsButton>
        )}
      </div>
      {error && <p className="text-xs text-error">{error}</p>}
    </div>
  );
}
