import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import SystemPage from "../SystemPage";

/**
 * システムビュー。コンテナの操作・診断の全件・ログは、旧 SystemPage をそのまま表示する。
 * 旧ページの部品は濃色の配色を直接書いているので、テーマに関わらず濃色の面に置く
 * (新しいテーマへの置き換えは doc/ui_migration_todo.md)。
 */
export default function SystemOpsPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  return (
    <div className="dark h-full overflow-y-auto bg-gray-900 p-4 text-white">
      <SystemPage client={client} sysManager={sysManager} />
    </div>
  );
}
