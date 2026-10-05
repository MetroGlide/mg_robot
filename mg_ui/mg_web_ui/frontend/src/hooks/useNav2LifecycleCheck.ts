import { useCallback, useState } from "react";
import { FoxgloveClientHandle } from "./useFoxgloveClient";
import { SERVICES } from "../ros/services";

export interface Nav2LifecycleResult {
  /** null はサービスの呼び出しに失敗したこと(lifecycle_manager がいない、など) */
  navigation: boolean | null;
  localization: boolean | null;
  checkedAt: Date;
}

/**
 * Nav2 のライフサイクル(navigation / localization)が active かを、呼んだときだけ 1 回調べる。
 * 負荷を増やさないため、定期的には呼ばない。普段の監視は /diagnostics のノードの状態で行う。
 */
export function useNav2LifecycleCheck(client: FoxgloveClientHandle) {
  const [result, setResult] = useState<Nav2LifecycleResult | null>(null);
  const [checking, setChecking] = useState(false);
  const { callService } = client;

  const isActive = useCallback(
    async (service: string): Promise<boolean | null> => {
      try {
        const r = await callService(service, {});
        return (r as { success?: boolean }).success === true;
      } catch {
        return null;
      }
    },
    [callService],
  );

  const check = useCallback(async () => {
    setChecking(true);
    const [navigation, localization] = await Promise.all([
      isActive(SERVICES.LIFECYCLE_NAV_IS_ACTIVE),
      isActive(SERVICES.LIFECYCLE_LOC_IS_ACTIVE),
    ]);
    setResult({ navigation, localization, checkedAt: new Date() });
    setChecking(false);
  }, [isActive]);

  return { result, checking, check };
}
