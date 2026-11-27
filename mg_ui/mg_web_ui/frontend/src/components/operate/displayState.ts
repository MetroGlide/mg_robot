
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
