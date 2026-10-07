import { Suspense } from "react";
import { Navigate, useParams } from "react-router-dom";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { findUseCase, USE_CASES } from "./useCases";

/** /ops/:useCase の運用ビュー。ユースケースごとのコンポーネントに任せる */
export default function OperatePage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const { useCase } = useParams();
  const found = findUseCase(useCase);
  if (!found) return <Navigate to={`/ops/${USE_CASES[0].id}`} replace />;

  const { Operate } = found;
  return (
    <Suspense fallback={<p className="p-4 text-sm text-muted">読み込み中…</p>}>
      <Operate client={client} sysManager={sysManager} />
    </Suspense>
  );
}
