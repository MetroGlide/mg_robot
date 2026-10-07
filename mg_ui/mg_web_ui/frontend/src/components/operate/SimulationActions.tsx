import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useSystemManagerAction } from "../../hooks/useSystemManagerAction";
import { PoseInput } from "../../hooks/useWaypointControl";
import SimulationPoseSection from "../status/SimulationPoseSection";
import ContainerControl from "./ContainerControl";

interface Props {
  sysManager: SystemManagerHandle;
  /** AMCL の姿勢も合わせて戻す場合に渡す(Waypoint のように AMCL を使うビュー) */
  onResetAmcl?: (pose: PoseInput) => void;
  /** Scenario Test のコンテナの操作を出すか */
  showScenarioTest?: boolean;
}

/** シミュレーション用の操作。シミュレータ上の姿勢のリセットと、Scenario Test コンテナの起動・停止 */
export default function SimulationActions({
  sysManager,
  onResetAmcl,
  showScenarioTest = false,
}: Props) {
  const { call, loading, error } = useSystemManagerAction(sysManager);

  return (
    <div className="space-y-3">
      <SimulationPoseSection
        onResetRobot={(pose) => void call("/simulation/reset-pose", pose)}
        onResetAmcl={onResetAmcl}
        loading={loading}
        error={error}
      />
      {showScenarioTest && (
        <ContainerControl
          sysManager={sysManager}
          container="scenario-test"
          title="Scenario Test"
          restartable={false}
        />
      )}
    </div>
  );
}
