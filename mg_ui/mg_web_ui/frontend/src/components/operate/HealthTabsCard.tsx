import { useMemo } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useDiagnosticsAgg } from "../../hooks/useDiagnosticsAgg";
import { useFreshness } from "../../hooks/useFreshness";
import { useOpsValue } from "../../hooks/useOpsValue";
import { countOk, HealthItem, summarizeHealth } from "../../hooks/healthSummary";
import { TOPICS } from "../../ros/interfaces";
import Card from "../ui/Card";
import HealthDot from "../ui/HealthDot";
import { toneFromDiagLevel } from "../ui/Pill";
import Tabs, { TabItem } from "../ui/Tabs";

// 集約した診断は 1Hz で配信される。これを超えて届かなければ、集約(または監視)が止まっている
const DIAGNOSTICS_STALE_AFTER_SEC = 5;

/** 1 つにしぼる(区分の id)か、全部を並べる */
const ALL = "all";

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
 * ドライバ・自己位置・ナビ・システムなどの区分ごとの状態。集約した診断(/diagnostics_agg、1Hz)だけを使い、
 * 高レートのトピックは購読しない。レベルは ROS 側の判定をそのまま表示する。
 * 「全て」を選ぶと、タブを切り替えずに並べて見られる。選択は画面を切り替えても保持する。
 */
export default function HealthTabsCard({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const [view, setView] = useOpsValue<string>("health.view", ALL);
  const root = useDiagnosticsAgg(client);
  const freshness = useFreshness(
    client,
    TOPICS.DIAGNOSTICS_AGG,
    DIAGNOSTICS_STALE_AFTER_SEC,
  );
  const regions = useMemo(() => summarizeHealth(root), [root]);

  const tabs: TabItem[] = [
    ...regions.map((region) => {
      const { ok, total } = countOk(region.items);
      return { id: region.id, label: region.label, badge: `${ok}/${total}` };
    }),
    { id: ALL, label: "全て" },
  ];
  // 保存された選択が今の区分になければ(シミュレーションにドライバがない、など)全てにする
  const activeId = tabs.some((t) => t.id === view) ? view : ALL;
  const selected = regions.find((r) => r.id === activeId);

  return (
    <Card className={`p-3 ${activeId === ALL ? "w-[42rem]" : "w-[24rem]"}`}>
      <Tabs items={tabs} activeId={activeId} onChange={setView} />
      {root === null && (
        <p className="mt-2 text-xs text-muted">診断を待っています…</p>
      )}
      {root !== null && freshness.stale && (
        <p className="mt-2 text-xs text-warn">
          診断が更新されていません。表示は最後に受信した状態です。
        </p>
      )}
      {activeId === ALL ? (
        <div className="mt-2 grid max-h-56 grid-cols-3 gap-4 overflow-y-auto">
          {regions.map((region) => {
            const { ok, total } = countOk(region.items);
            return (
              <section key={region.id}>
                <h3 className="mb-1 text-[11px] font-semibold text-muted">
                  {region.label} {ok}/{total}
                </h3>
                <ItemList items={region.items} compact />
              </section>
            );
          })}
        </div>
      ) : (
        <div className="mt-2 max-h-56 overflow-y-auto">
          <ItemList items={selected?.items ?? []} />
        </div>
      )}
    </Card>
  );
}
