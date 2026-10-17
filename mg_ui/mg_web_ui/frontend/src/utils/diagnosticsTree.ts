import { DiagnosticStatus } from "../types";

export interface DiagNode {
  /** "/Robot/Drivers/LiDAR" のような、根からのパス */
  path: string;
  /** パスの最後の要素 */
  name: string;
  level: number;
  message: string;
  hardwareId: string;
  values: { key: string; value: string }[];
  children: DiagNode[];
  /** 元の診断(`<ノード名>: <タスク名>`)。グループ(集約の区分)ではない */
  isLeaf: boolean;
}

/** 元の診断の名前は `<ノード名>: <タスク名>`。集約のグループの名前には ": " がない */
function isLeafName(name: string): boolean {
  return name.includes(": ");
}

function lastSegment(path: string): string {
  return path.slice(path.lastIndexOf("/") + 1);
}

function parentPath(path: string): string {
  return path.slice(0, path.lastIndexOf("/"));
}

/** 重いほうのレベル。STALE(3)は ERROR(2)より重いものとして扱う */
export function worstLevel(levels: number[]): number {
  return levels.reduce((worst, level) => Math.max(worst, level), 0);
}

/**
 * 集約した診断(/diagnostics_agg の status の一覧)を、根(rootPath)からの木にする。
 * 集約が出さなかった途中のパスは、子のうち最も重いレベルで補う。根がなければ null。
 */
export function buildDiagnosticsTree(
  statuses: DiagnosticStatus[],
  rootPath: string,
): DiagNode | null {
  const nodes = new Map<string, DiagNode>();
  const make = (path: string, status?: DiagnosticStatus): DiagNode => ({
    path,
    name: lastSegment(path),
    level: status?.level ?? 0,
    message: status?.message ?? "",
    hardwareId: status?.hardware_id ?? "",
    values: status?.values ?? [],
    children: [],
    isLeaf: isLeafName(lastSegment(path)),
  });

  const inTree = (path: string) => path === rootPath || path.startsWith(`${rootPath}/`);
  for (const status of statuses) {
    if (inTree(status.name)) nodes.set(status.name, make(status.name, status));
  }

  const synthetic = new Set<string>();
  for (const path of [...nodes.keys()]) {
    for (let p = parentPath(path); inTree(p) && !nodes.has(p); p = parentPath(p)) {
      nodes.set(p, make(p));
      synthetic.add(p);
    }
  }

  for (const [path, node] of nodes) {
    if (path === rootPath) continue;
    nodes.get(parentPath(path))?.children.push(node);
  }

  const finish = (node: DiagNode): void => {
    node.children.sort((a, b) => a.name.localeCompare(b.name));
    node.children.forEach(finish);
    if (synthetic.has(node.path)) {
      node.level = worstLevel(node.children.map((c) => c.level));
    }
  };
  const root = nodes.get(rootPath) ?? null;
  if (root) finish(root);
  return root;
}

export function findNode(root: DiagNode | null, path: string): DiagNode | null {
  if (root === null) return null;
  if (root.path === path) return root;
  for (const child of root.children) {
    const found = findNode(child, path);
    if (found) return found;
  }
  return null;
}

/** 元の診断(葉)の一覧 */
export function leaves(node: DiagNode): DiagNode[] {
  if (node.isLeaf || node.children.length === 0) return node.isLeaf ? [node] : [];
  return node.children.flatMap(leaves);
}
