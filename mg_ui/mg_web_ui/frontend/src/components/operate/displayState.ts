/** クォータニオンから、地図平面での向き(yaw)を度で返す */
export function quaternionToYawDeg(q: {
  x: number;
  y: number;
  z: number;
  w: number;
}): number {
  const yaw = Math.atan2(
    2 * (q.w * q.z + q.x * q.y),
    1 - 2 * (q.y * q.y + q.z * q.z),
  );
  return (yaw * 180) / Math.PI;
}

/** 地点の区切りのバーの 1 区画の状態 */
export type WaypointSegment = "passed" | "current" | "ahead";

/**
 * 地点の区切りのバーの区画。シーケンサが出す番号と総数を並べるだけで、状態は判定しない
 * (いま何をしているかはランプが表す)。全地点に到着したら全て通過済み。
 */
export function waypointSegments(
  currentIndex: number,
  total: number,
  goalReached: boolean,
): WaypointSegment[] {
  return Array.from({ length: Math.max(0, total) }, (_, i) => {
    if (goalReached || i < currentIndex) return "passed";
    return i === currentIndex ? "current" : "ahead";
  });
}

/** 区切りで見せる地点の範囲 [start, end)。地点が多いときは、いまの地点の前後だけにする */
export function waypointWindow(
  currentIndex: number,
  total: number,
  before: number,
  after: number,
): { start: number; end: number } {
  const size = before + after + 1;
  if (total <= size) return { start: 0, end: Math.max(0, total) };
  const start = Math.min(Math.max(0, currentIndex - before), total - size);
  return { start, end: start + size };
}

/** いまの地点から先で、最初に到着後アクションがある地点の番号。なければ null */
export function nextActionIndex(
  actionTypes: readonly (readonly string[])[],
  currentIndex: number,
): number | null {
  for (let i = Math.max(0, currentIndex); i < actionTypes.length; i++) {
    if (actionTypes[i].length > 0) return i;
  }
  return null;
}
