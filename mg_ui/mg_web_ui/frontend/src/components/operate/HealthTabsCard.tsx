import { useMemo, useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useDiagnosticsMap } from "../../hooks/useDiagnosticsMap";
import { useFreshness } from "../../hooks/useFreshness";
import {
  countOk,
  HealthGroup,
  summarizeHealth,
} from "../../hooks/healthSummary";
import { TOPICS } from "../../ros/interfaces";
import Card from "../ui/Card";
import HealthDot from "../ui/HealthDot";
import { toneFromDiagLevel } from "../ui/Pill";
import Tabs, { TabItem } from "../ui/Tabs";

// 診断は 1Hz で配信される。これを超えて届かなければ、診断ノード自体が止まっている
const DIAGNOSTICS_STALE_AFTER_SEC = 5;

const GROUP_LABEL: Record<HealthGroup, string> = {
  sensors: "センサ",
  topics: "トピック",
  nodes: "ノード",
};

/** センサ・トピック・ノードの状態。/diagnostics(1Hz)だけを使い、高レートのトピックは購読しない */
export default function HealthTabsCard({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const [group, setGroup] = useState<HealthGroup>("sensors");
  const diagnostics = useDiagnosticsMap(client);
  const freshness = useFreshness(
    client,
    TOPICS.DIAGNOSTICS,
    DIAGNOSTICS_STALE_AFTER_SEC,
  );
  const summary = useMemo(() => summarizeHealth(diagnostics), [diagnostics]);

  const tabs: TabItem[] = (Object.keys(GROUP_LABEL) as HealthGroup[]).map(
    (id) => {
      const { ok, total } = countOk(summary[id]);
      return { id, label: GROUP_LABEL[id], badge: `${ok}/${total}` };
    },
  );
  const items = summary[group];

  return (
    <Card className="w-72 p-3">
      <Tabs
        items={tabs}
        activeId={group}
        onChange={(id) => setGroup(id as HealthGroup)}
      />
      {freshness.stale && (
        <p className="mt-2 text-xs text-warn">
          診断が更新されていません。表示は最後に受信した状態です。
        </p>
      )}
      <ul className="mt-2 max-h-48 space-y-1.5 overflow-y-auto">
        {items.length === 0 && (
          <li className="text-xs text-muted">データなし</li>
        )}
        {items.map((item) => (
          <li key={item.name} className="flex items-center gap-2 text-xs">
            <HealthDot tone={toneFromDiagLevel(item.level)} />
            <span className="min-w-0 flex-1 truncate font-medium">
              {item.name}
            </span>
            <span className="text-muted">{item.message}</span>
          </li>
        ))}
      </ul>
    </Card>
  );
}
