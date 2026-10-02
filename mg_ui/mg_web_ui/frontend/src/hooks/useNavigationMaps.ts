import { useCallback, useEffect, useState } from "react";
import { LoadedMaps } from "../types";
import type { NavigationMapEntry } from "../types/api";
import { TOPICS } from "../ros/topics";
import { fetchNavigationMaps } from "../utils/navigationMapsApi";
import { FoxgloveClientHandle } from "./useFoxgloveClient";
import { useTopicSubscriber } from "./useTopicSubscriber";

export interface NavigationMaps {
  /** map_list.txt の地図。まだ取得できていなければ空 */
  maps: NavigationMapEntry[];
  /** map_list.txt に書かれていたが、使えない項目 */
  skipped: string[];
  loading: boolean;
  /** 一覧を取得できなかった理由 (map_list.txt が無い、通信失敗など) */
  error: string | null;
  reload: () => Promise<void>;
  /** sequencer が map_server に読み込ませた地図。状態トピックが届いていなければ null */
  loaded: LoadedMaps | null;
}

export function useNavigationMaps(
  client: FoxgloveClientHandle,
): NavigationMaps {
  const [maps, setMaps] = useState<NavigationMapEntry[]>([]);
  const [skipped, setSkipped] = useState<string[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const loaded = useTopicSubscriber<LoadedMaps>(
    client,
    TOPICS.WAYPOINT_LOADED_MAPS,
    "mg_msgs/msg/LoadedMaps",
  );

  const reload = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const response = await fetchNavigationMaps();
      if (response.success) {
        setMaps(response.maps);
        setSkipped(response.skipped);
      } else {
        setMaps([]);
        setSkipped([]);
        setError(response.message);
      }
    } catch (e) {
      setMaps([]);
      setSkipped([]);
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    void reload();
  }, [reload]);

  return { maps, skipped, loading, error, reload, loaded };
}
