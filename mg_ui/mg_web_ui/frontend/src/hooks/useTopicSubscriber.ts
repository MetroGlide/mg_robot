import { useEffect, useState } from "react";
import { FoxgloveClientHandle } from "./useFoxgloveClient";

/** トピックの最新メッセージを返す。購読は接続状態によらず維持される。 */
export function useTopicSubscriber<T>(
  client: FoxgloveClientHandle,
  topic: string,
  schemaName: string,
): T | null {
  const [data, setData] = useState<T | null>(null);
  const { subscribe } = client;

  useEffect(
    () => subscribe(topic, schemaName, (msg) => setData(msg as T)),
    [subscribe, topic, schemaName],
  );

  return data;
}
