import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import ScenarioWorkspace from "../../components/scenario/ScenarioWorkspace";
import WorkLayout from "../../components/work/WorkLayout";

/** シナリオテストの作業ビュー。地図を使わず、選択・実行・結果・履歴を並べる */
export default function ScenarioOperate({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  return (
    <WorkLayout title="Scenario Test">
      <ScenarioWorkspace client={client} sysManager={sysManager} />
    </WorkLayout>
  );
}
