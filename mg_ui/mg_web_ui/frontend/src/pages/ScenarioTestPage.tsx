import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import ScenarioWorkspace from "../components/scenario/ScenarioWorkspace";

export default function ScenarioTestPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  return <ScenarioWorkspace client={client} sysManager={sysManager} />;
}
