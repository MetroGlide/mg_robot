import { DiagnosticStatus } from "../types";

export type HealthGroup = "sensors" | "topics" | "nodes";

export interface HealthItem {
  /** "topic/" や "node/" を除いた名前 */
  name: string;
  level: number;
  message: string;
}

export interface HealthSummary {
  sensors: HealthItem[];
  topics: HealthItem[];
  nodes: HealthItem[];
}

// mg_diagnostics に明示的なセンサ接続の確認はないので、センサが出すトピックの配信状態で代用する。
// どのトピックをセンサとするかは、mg_diagnostics の params(監視するトピックの group)で決まる。
const GROUP_KEY = "group";
const SENSOR_GROUP = "sensor";

const TOPIC_PREFIX = "topic/";
const NODE_PREFIX = "node/";

/** /diagnostics の status を、センサ・トピック・ノードに分類する。他の診断(localization など)は含めない。 */
export function summarizeHealth(statuses: DiagnosticStatus[]): HealthSummary {
  const summary: HealthSummary = { sensors: [], topics: [], nodes: [] };
  for (const s of statuses) {
    if (s.name.startsWith(TOPIC_PREFIX)) {
      const name = s.name.slice(TOPIC_PREFIX.length);
      const item = { name, level: s.level, message: s.message };
      const isSensor = s.values.some(
        (v) => v.key === GROUP_KEY && v.value === SENSOR_GROUP,
      );
      (isSensor ? summary.sensors : summary.topics).push(item);
    } else if (s.name.startsWith(NODE_PREFIX)) {
      summary.nodes.push({
        name: s.name.slice(NODE_PREFIX.length),
        level: s.level,
        message: s.message,
      });
    }
  }
  return summary;
}

/** OK(level 0)の数と全体の数 */
export function countOk(items: HealthItem[]): { ok: number; total: number } {
  return {
    ok: items.filter((i) => i.level === 0).length,
    total: items.length,
  };
}
