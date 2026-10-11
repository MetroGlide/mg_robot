import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useDiagnosticsAgg } from "../../hooks/useDiagnosticsAgg";
import { useFreshness } from "../../hooks/useFreshness";
import { TOPICS } from "../../ros/interfaces";
import { DiagNode } from "../../utils/diagnosticsTree";
import HealthDot from "../ui/HealthDot";
import { toneFromDiagLevel } from "../ui/Pill";

// 集約した診断は 1Hz で配信される。これを超えて届かなければ、集約(または監視)が止まっている
const STALE_AFTER_SEC = 5;

function NodeRow({ node, depth }: { node: DiagNode; depth: number }) {
  const heading = (
    <span className="flex items-center gap-2">
      <HealthDot tone={toneFromDiagLevel(node.level)} />
      <span className="font-medium">{node.name}</span>
      {node.message && <span className="text-muted">{node.message}</span>}
    </span>
  );

  // 元の診断(葉)は、補足の値(values)を開いて見られる
  if (node.isLeaf) {
    return (
      <li className="ml-4 text-xs">
        <details>
          <summary className="cursor-pointer py-0.5">{heading}</summary>
          <dl className="ml-6 grid grid-cols-[max-content_1fr] gap-x-3 pb-1 text-muted">
            {node.hardwareId && (
              <>
                <dt>hardware_id</dt>
                <dd>{node.hardwareId}</dd>
              </>
            )}
            {node.values.map((v) => (
              <div key={v.key} className="contents">
                <dt>{v.key}</dt>
                <dd className="break-all">{v.value}</dd>
              </div>
            ))}
          </dl>
        </details>
      </li>
    );
  }

  // 正常でないものと、上の 2 階層は開いておく
  return (
    <li className={depth === 0 ? "text-sm" : "ml-4 text-xs"}>
      <details open={node.level !== 0 || depth < 2}>
        <summary className="cursor-pointer py-0.5">{heading}</summary>
        <ul>
          {node.children.map((child) => (
            <NodeRow key={child.path} node={child} depth={depth + 1} />
          ))}
        </ul>
      </details>
    </li>
  );
}

/**
 * 集約した診断(/diagnostics_agg)の全項目を、木で表示する。
 * レベルは ROS 側の判定で、グループのレベルは、中の項目のうち最も悪いもの。
 */
export default function DiagnosticsTreePanel({ client }: { client: FoxgloveClientHandle }) {
  const root = useDiagnosticsAgg(client);
  const freshness = useFreshness(client, TOPICS.DIAGNOSTICS_AGG, STALE_AFTER_SEC);

  if (root === null) {
    return <p className="text-sm text-muted">診断(/diagnostics_agg)を待っています…</p>;
  }
  return (
    <div>
      {freshness.stale && (
        <p className="mb-2 text-xs text-warn">
          診断が更新されていません。表示は最後に受信した状態です。
        </p>
      )}
      <ul>
        <NodeRow node={root} depth={0} />
      </ul>
    </div>
  );
}
