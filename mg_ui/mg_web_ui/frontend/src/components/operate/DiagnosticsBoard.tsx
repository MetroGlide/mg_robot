import { useMemo, useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useDiagnosticsAgg } from "../../hooks/useDiagnosticsAgg";
import { useFreshness } from "../../hooks/useFreshness";
import { useOpsValue } from "../../hooks/useOpsValue";
import {
  countRegion,
  HealthGroup,
  HealthMode,
  HealthRegion,
  layoutGroup,
  summarizeHealth,
} from "../../hooks/healthSummary";
import { TOPICS } from "../../ros/interfaces";
import Card from "../ui/Card";
import HealthDot from "../ui/HealthDot";
import { toneFromDiagLevel } from "../ui/Pill";
import Tabs, { TabItem } from "../ui/Tabs";
import DiagnosticsTreePanel from "./DiagnosticsTreePanel";

// 集約した診断は 1Hz で配信される。これを超えて届かなければ、集約(または監視)が止まっている
const STALE_AFTER_SEC = 5;

const MODE_TABS: TabItem[] = [
  { id: "full", label: "全表示" },
  { id: "compact", label: "簡易" },
];

function Row({
  level,
  title,
  summary,
  tip,
}: {
  level: number;
  title: string;
  summary: string;
  tip?: string;
}) {
  return (
    <li
      title={tip ?? `${title} ${summary}`}
      className="flex items-baseline gap-1.5 text-xs leading-tight"
    >
      <span className="self-center">
        <HealthDot tone={toneFromDiagLevel(level)} />
      </span>
      <span className="font-medium">{title}</span>
      {summary && <span className="break-words text-muted">{summary}</span>}
    </li>
  );
}

function GroupRows({ group, mode }: { group: HealthGroup; mode: HealthMode }) {
  const layout = layoutGroup(group, mode);
  return (
    <>
      <Row level={layout.level} title={layout.title} summary={layout.summary} />
      {layout.rows.length > 0 && (
        <li className="ml-4">
          <ul className="flex flex-wrap gap-x-3 gap-y-0.5">
            {layout.rows.map((item) => (
              <Row
                key={item.name}
                level={item.level}
                title={item.label}
                summary={item.message}
                tip={`${item.name}: ${item.message}`}
              />
            ))}
          </ul>
        </li>
      )}
    </>
  );
}

function RegionColumn({ region, mode }: { region: HealthRegion; mode: HealthMode }) {
  const { ok, total } = countRegion(region);
  return (
    <section className="min-w-[8rem] max-w-[14rem]">
      <h3 className="mb-1 flex items-center gap-1.5 text-[11px] font-semibold text-muted">
        <HealthDot tone={toneFromDiagLevel(region.level)} />
        {region.label}
        <span className="tabular-nums">
          {ok}/{total}
        </span>
      </h3>
      <ul className="space-y-1">
        {region.groups.map((group) => (
          <GroupRows key={group.name} group={group} mode={mode} />
        ))}
      </ul>
    </section>
  );
}

/**
 * 集約した診断(/diagnostics_agg、1Hz)の一覧。区分ごとに並べ、スクロールなしで全体を見渡せる。
 * 全表示は全項目、簡易は OK の項目を件数にまとめて異常な項目だけを出す(選択は画面を切り替えても保持する)。
 * レベルは ROS 側の判定をそのまま表示する。「詳細」で、値まで見られる木を開く。
 */
export default function DiagnosticsBoard({ client }: { client: FoxgloveClientHandle }) {
  const [mode, setMode] = useOpsValue<HealthMode>("health.mode", "full");
  const [detailOpen, setDetailOpen] = useState(false);
  const root = useDiagnosticsAgg(client);
  const freshness = useFreshness(client, TOPICS.DIAGNOSTICS_AGG, STALE_AFTER_SEC);
  const regions = useMemo(() => summarizeHealth(root), [root]);

  return (
    <div className="flex flex-col items-end gap-2">
      {detailOpen && (
        <Card className="max-h-[70vh] w-[28rem] overflow-y-auto p-3">
          <DiagnosticsTreePanel client={client} />
        </Card>
      )}
      <Card className="max-w-[calc(100vw-37rem)] p-3">
        <div className="mb-2 flex items-center gap-2">
          <HealthDot tone={root === null ? "neutral" : toneFromDiagLevel(root.level)} />
          <h2 className="text-xs font-bold">診断</h2>
          <div className="ml-auto w-36">
            <Tabs
              items={MODE_TABS}
              activeId={mode}
              onChange={(id) => setMode(id as HealthMode)}
            />
          </div>
          <button
            type="button"
            onClick={() => setDetailOpen(!detailOpen)}
            aria-pressed={detailOpen}
            className="shrink-0 rounded-md bg-surface-sunken px-2 py-1 text-xs font-semibold hover:bg-line"
          >
            詳細
          </button>
        </div>
        {root === null && <p className="text-xs text-muted">診断を待っています…</p>}
        {root !== null && freshness.stale && (
          <p className="mb-2 text-xs text-warn">
            診断が更新されていません。表示は最後に受信した状態です。
          </p>
        )}
        <div className="flex flex-wrap items-start gap-x-5 gap-y-3">
          {regions.map((region) => (
            <RegionColumn key={region.id} region={region} mode={mode} />
          ))}
        </div>
      </Card>
    </div>
  );
}
