import { useMemo } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useDiagnosticsMap } from "../../hooks/useDiagnosticsMap";
import { useFreshness } from "../../hooks/useFreshness";
import { useOpsValue } from "../../hooks/useOpsValue";
import {
  countOk,
  HealthGroup,
  HealthItem,
  summarizeHealth,
} from "../../hooks/healthSummary";
import { TOPICS } from "../../ros/interfaces";
import Card from "../ui/Card";
import HealthDot from "../ui/HealthDot";
import { toneFromDiagLevel } from "../ui/Pill";
import Tabs, { TabItem } from "../ui/Tabs";

// 診断は 1Hz で配信される。これを超えて届かなければ、診断ノード自体が止まっている
const DIAGNOSTICS_STALE_AFTER_SEC = 5;

const GROUPS: HealthGroup[] = ["sensors", "topics", "nodes"];

const GROUP_LABEL: Record<HealthGroup, string> = {
  sensors: "センサ",
  topics: "トピック",
  nodes: "ノード",
};

/** 1 つにしぼる(group)か、全部を並べる("all") */
type View = HealthGroup | "all";

/** compact なら、狭い列に収まるよう、メッセージを名前の下の行に置く */
function ItemList({
  items,
  compact = false,
}: {
  items: HealthItem[];
  compact?: boolean;
}) {
  return (
    <ul className="space-y-1.5">
      {items.length === 0 && <li className="text-xs text-muted">データなし</li>}
      {items.map((item) => (
        <li
          key={item.name}
          title={`${item.name}: ${item.message}`}
          className="flex items-start gap-2 text-xs"
        >
          <span className="mt-1">
            <HealthDot tone={toneFromDiagLevel(item.level)} />
          </span>
          <div
            className={`min-w-0 flex-1 ${compact ? "" : "flex items-center gap-2"}`}
          >
            <div className="min-w-0 flex-1 truncate font-medium">{item.name}</div>
            <div className="text-muted">{item.message}</div>
          </div>
        </li>
      ))}
    </ul>
  );
}

/**
 * センサ・トピック・ノードの状態。/diagnostics(1Hz)だけを使い、高レートのトピックは購読しない。
 * 「全て」を選ぶと、タブを切り替えずに 3 つを並べて見られる。選択は画面を切り替えても保持する。
 */
export default function HealthTabsCard({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const [view, setView] = useOpsValue<View>("health.view", "sensors");
  const diagnostics = useDiagnosticsMap(client);
  const freshness = useFreshness(
    client,
    TOPICS.DIAGNOSTICS,
    DIAGNOSTICS_STALE_AFTER_SEC,
  );
  const summary = useMemo(() => summarizeHealth(diagnostics), [diagnostics]);

  const tabs: TabItem[] = [
    ...GROUPS.map((id) => {
      const { ok, total } = countOk(summary[id]);
      return { id, label: GROUP_LABEL[id], badge: `${ok}/${total}` };
    }),
    { id: "all", label: "全て" },
  ];

  return (
    <Card className={`p-3 ${view === "all" ? "w-[42rem]" : "w-[24rem]"}`}>
      <Tabs items={tabs} activeId={view} onChange={(id) => setView(id as View)} />
      {freshness.stale && (
        <p className="mt-2 text-xs text-warn">
          診断が更新されていません。表示は最後に受信した状態です。
        </p>
      )}
      {view === "all" ? (
        <div className="mt-2 grid max-h-56 grid-cols-3 gap-4 overflow-y-auto">
          {GROUPS.map((group) => {
            const { ok, total } = countOk(summary[group]);
            return (
              <section key={group}>
                <h3 className="mb-1 text-[11px] font-semibold text-muted">
                  {GROUP_LABEL[group]} {ok}/{total}
                </h3>
                <ItemList items={summary[group]} compact />
              </section>
            );
          })}
        </div>
      ) : (
        <div className="mt-2 max-h-56 overflow-y-auto">
          <ItemList items={summary[view]} />
        </div>
      )}
    </Card>
  );
}
