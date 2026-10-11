import { DIAG_REGIONS } from "../ros/diagnosticPaths";
import { DiagNode, leaves } from "../utils/diagnosticsTree";

export interface HealthItem {
  /** 元の診断の名前(`<ノード名>: <タスク名>`)。空の区分は、その区分の名前 */
  name: string;
  level: number;
  message: string;
}

export interface HealthRegion {
  id: string;
  label: string;
  items: HealthItem[];
}

/** 区分(Drivers など)の中の項目。診断が 1 つも届いていない区分(STALE)は、区分そのものを 1 項目にする */
function itemsOf(region: DiagNode): HealthItem[] {
  const items: HealthItem[] = [];
  const children = region.isLeaf ? [region] : region.children;
  for (const child of children) {
    const found = leaves(child);
    if (found.length > 0) {
      items.push(...found.map((l) => ({ name: l.name, level: l.level, message: l.message })));
    } else {
      items.push({ name: child.name, level: child.level, message: child.message });
    }
  }
  return items;
}

/**
 * 集約した診断の木を、区分(ドライバ・自己位置・ナビ・システムなど)ごとの項目にする。
 * レベルは診断(ROS 側)が決めたものをそのまま使い、UI では判定しない。
 * 木に無い区分は出さない(シミュレーションにはドライバがない、など)。
 */
export function summarizeHealth(root: DiagNode | null): HealthRegion[] {
  if (root === null) return [];
  const regions: HealthRegion[] = [];
  for (const { id, label } of DIAG_REGIONS) {
    const node = root.children.find((c) => c.name === id);
    if (node) regions.push({ id, label, items: itemsOf(node) });
  }
  return regions;
}

/** OK(level 0)の数と全体の数 */
export function countOk(items: HealthItem[]): { ok: number; total: number } {
  return {
    ok: items.filter((i) => i.level === 0).length,
    total: items.length,
  };
}
