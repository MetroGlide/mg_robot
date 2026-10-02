import type { NavigationMapsResponse } from "../types/api";
import { getSysManagerUrl } from "./systemManagerConfig";

/** MAP_PATH/map_list.txt の地図の一覧を取得する。通信や応答の異常は例外にする。 */
export async function fetchNavigationMaps(): Promise<NavigationMapsResponse> {
  const r = await fetch(`${getSysManagerUrl()}/navigation/maps`);
  if (!r.ok) throw new Error(`GET /navigation/maps failed: HTTP ${r.status}`);
  return (await r.json()) as NavigationMapsResponse;
}
