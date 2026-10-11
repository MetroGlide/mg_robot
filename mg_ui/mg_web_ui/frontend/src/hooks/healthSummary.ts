import { DIAG_REGIONS } from "../ros/diagnosticPaths";
import { DiagNode, leaves } from "../utils/diagnosticsTree";

/** 表示のモード。full は全項目、compact は OK の項目を件数にまとめる */
export type HealthMode = "full" | "compact";

export interface HealthItem {
  /** 元の診断の名前(`<ノード名>: <タスク名>`) */
  name: string;
  /** 表示名。タスク名。グループ内で重なるときだけ、元の名前 */
  label: string;
  level: number;
  /** 表示するメッセージ。意味のない(空、OK など)ものは空にする */
  message: string;
}

/** 集約のグループ(Nodes、Nav2 など)。単独の診断が直接区分に入るときは、その診断が 1 項目のグループになる */
export interface HealthGroup {
  name: string;
  level: number;
  items: HealthItem[];
}

export interface HealthRegion {
  id: string;
  label: string;
  level: number;
  groups: HealthGroup[];
}

/** 1 グループの表示内容。title の行(summary が右に付く)と、その下に並べる項目 */
export interface GroupLayout {
  title: string;
  level: number;
  /** 項目が 1 つならそのメッセージ、複数なら「OK の数/全体」 */
  summary: string;
  rows: HealthItem[];
}

const TRIVIAL_MESSAGES = new Set(["", "ok", "OK", "''"]);

function taskName(fullName: string): string {
  const i = fullName.indexOf(": ");
  return i < 0 ? fullName : fullName.slice(i + 2);
}

/** 空・OK・名前の繰り返しは、画面を占めるだけなので出さない */
function meaningfulMessage(message: string, ...names: string[]): string {
  if (TRIVIAL_MESSAGES.has(message) || names.includes(message)) return "";
  return message;
}

function itemsOf(group: DiagNode): HealthItem[] {
  // 診断が 1 つも届いていないグループ(STALE)は、グループそのものを項目にする
  const found = leaves(group);
  const nodes = found.length > 0 ? found : [group];
  const tasks = nodes.map((n) => taskName(n.name));
  return nodes.map((n, i) => {
    const label = tasks.filter((t) => t === tasks[i]).length > 1 ? n.name : tasks[i];
    return { name: n.name, label, level: n.level, message: meaningfulMessage(n.message, label) };
  });
}

function groupOf(node: DiagNode): HealthGroup {
  return {
    name: node.isLeaf ? taskName(node.name) : node.name,
    level: node.level,
    items: itemsOf(node),
  };
}

/**
 * 集約した診断の木を、区分(ドライバ・自己位置・ナビ・システムなど)ごとのグループと項目にする。
 * レベルは診断(ROS 側)が決めたものをそのまま使い、UI では判定しない。
 * 木に無い区分は出さない(シミュレーションにはドライバがない、など)。
 */
export function summarizeHealth(root: DiagNode | null): HealthRegion[] {
  if (root === null) return [];
  const regions: HealthRegion[] = [];
  for (const { id, label } of DIAG_REGIONS) {
    const node = root.children.find((c) => c.name === id);
    if (!node) continue;
    const children = node.isLeaf ? [node] : node.children;
    regions.push({ id, label, level: node.level, groups: children.map(groupOf) });
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

/** 区分に含まれる項目の OK の数と全体の数 */
export function countRegion(region: HealthRegion): { ok: number; total: number } {
  return countOk(region.groups.flatMap((g) => g.items));
}

/**
 * グループの表示内容。項目が 1 つなら 1 行(グループ名とメッセージ)。
 * 複数なら見出し(OK の数/全体)と項目で、compact では OK の項目を並べない。
 */
export function layoutGroup(group: HealthGroup, mode: HealthMode): GroupLayout {
  if (group.items.length === 1) {
    const [item] = group.items;
    return {
      title: group.name,
      level: group.level,
      summary: meaningfulMessage(item.message, group.name),
      rows: [],
    };
  }
  const { ok, total } = countOk(group.items);
  return {
    title: group.name,
    level: group.level,
    summary: `${ok}/${total}`,
    rows: mode === "compact" ? group.items.filter((i) => i.level !== 0) : group.items,
  };
}
