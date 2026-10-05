import { useEffect, useMemo, useState } from "react";
import { getSysManagerUrl } from "../utils/systemManagerConfig";

export interface ServiceItem {
  key: string;
  label: string;
}

export interface ServiceLayer {
  id: string;
  label: string;
  services: readonly ServiceItem[];
}

interface ServicesResponse {
  layers: { id: string; label: string }[];
  services: { key: string; label: string; layer: string | null }[];
}

const RETRY_INTERVAL_MS = 3000;

/** system_manager が管理するサービスの一覧を、表示グループごとにまとめて返す。 */
export function useServiceRegistry(): {
  layers: ServiceLayer[];
  services: readonly ServiceItem[];
  loaded: boolean;
} {
  const [registry, setRegistry] = useState<ServicesResponse | null>(null);

  useEffect(() => {
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout> | null = null;

    const load = async () => {
      try {
        const r = await fetch(`${getSysManagerUrl()}/services`);
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        const data = (await r.json()) as ServicesResponse;
        if (!cancelled) setRegistry(data);
      } catch {
        // system_manager の起動待ちの間は、間隔をあけて取得し直す
        if (!cancelled) timer = setTimeout(load, RETRY_INTERVAL_MS);
      }
    };
    void load();

    return () => {
      cancelled = true;
      if (timer) clearTimeout(timer);
    };
  }, []);

  return useMemo(() => {
    if (!registry) return { layers: [], services: [], loaded: false };
    const layers = registry.layers
      .map((layer) => ({
        ...layer,
        services: registry.services
          .filter((s) => s.layer === layer.id)
          .map(({ key, label }) => ({ key, label })),
      }))
      .filter((layer) => layer.services.length > 0);
    return {
      layers,
      services: layers.flatMap((layer) => layer.services),
      loaded: true,
    };
  }, [registry]);
}
