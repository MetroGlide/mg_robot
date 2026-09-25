import { useEffect, useMemo, useSyncExternalStore } from "react";
import {
  ConnectionStatus,
  FoxgloveConnection,
} from "../ros/foxgloveConnection";

export type { ConnectionStatus };

export interface FoxgloveClientHandle {
  status: ConnectionStatus;
  /** 接続前や再接続中でも呼べる。接続後にサーバ側の購読が自動で張られる。 */
  subscribe: (
    topic: string,
    schemaName: string,
    onMessage: (data: unknown) => void,
  ) => () => void;
  callService: (service: string, payload: unknown) => Promise<unknown>;
  publish: (topic: string, schemaName: string, data: unknown) => void;
  /** publish の前に呼んでおくと、最初のメッセージが取りこぼされにくくなる */
  advertise: (topic: string, schemaName: string) => () => void;
  /** トピックの最後のメッセージを受信した時刻(ms)。未受信なら null */
  getLastMessageAt: (topic: string) => number | null;
}

function getWsUrl(): string {
  return `ws://${window.location.hostname}:8765/`;
}

export function useFoxgloveClient(): FoxgloveClientHandle {
  const connection = useMemo(
    () => new FoxgloveConnection({ url: getWsUrl() }),
    [],
  );

  useEffect(() => {
    connection.start();
    return () => connection.stop();
  }, [connection]);

  const status = useSyncExternalStore(
    (onChange) => connection.onStatusChange(onChange),
    () => connection.status,
  );

  // status 以外は接続が変わらない限り同一の関数にして、依存に入れた側の再実行を避ける
  const methods = useMemo<Omit<FoxgloveClientHandle, "status">>(
    () => ({
      subscribe: (topic, _schemaName, onMessage) =>
        connection.subscribe(topic, onMessage),
      callService: (service, payload) =>
        connection.callService(service, payload),
      publish: (topic, schemaName, data) =>
        connection.publish(topic, schemaName, data),
      advertise: (topic, schemaName) => connection.advertise(topic, schemaName),
      getLastMessageAt: (topic) => connection.getLastMessageAt(topic),
    }),
    [connection],
  );

  return useMemo(() => ({ status, ...methods }), [status, methods]);
}
