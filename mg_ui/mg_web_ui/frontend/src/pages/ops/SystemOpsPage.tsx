import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import SystemPage from "../SystemPage";
import WorkLayout from "../../components/work/WorkLayout";

/** システムビュー。サービスの操作とログの設定、診断の全件、サービスのログ、API のログ */
export default function SystemOpsPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  return (
    <WorkLayout title="システム">
      <SystemPage client={client} sysManager={sysManager} />
    </WorkLayout>
  );
}
