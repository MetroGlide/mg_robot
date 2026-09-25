import { useEffect, useMemo, useRef, useState } from "react";
import type { LogEntry } from "../types/api";
import {
  getSysManagerUrl,
  SYS_MANAGER_URL_CHANGED_EVENT,
} from "../utils/systemManagerConfig";

const MAX_LOG_ENTRIES = 500;
const FLUSH_INTERVAL_MS = 100;
const RECONNECT_DELAY_MS = 3000;

interface StreamEntry {
  service?: unknown;
  line?: unknown;
  error?: unknown;
}

interface StreamBatch {
  entries?: StreamEntry[];
  dropped?: number;
}

export function useDockerLogStream(subscribedServices: string[]) {
  const [entries, setEntries] = useState<LogEntry[]>([]);
  const [connected, setConnected] = useState(false);
  const [baseUrl, setBaseUrl] = useState(() => getSysManagerUrl());

  useEffect(() => {
    const handler = () => setBaseUrl(getSysManagerUrl());
    window.addEventListener(SYS_MANAGER_URL_CHANGED_EVENT, handler);
    return () => window.removeEventListener(SYS_MANAGER_URL_CHANGED_EVENT, handler);
  }, []);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const flushTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const pendingRef = useRef<LogEntry[]>([]);
  const mountedRef = useRef(false);
  const lastServicesRef = useRef<string[]>([]);
  const entryIdRef = useRef(0);

  const servicePayload = useMemo(
    () => [...subscribedServices].sort(),
    [subscribedServices],
  );

  const clear = () => {
    pendingRef.current = [];
    setEntries([]);
  };

  useEffect(() => {
    mountedRef.current = true;

    const flushPending = () => {
      if (pendingRef.current.length === 0) return;
      const next = pendingRef.current;
      pendingRef.current = [];
      setEntries((prev) => [...prev, ...next].slice(-MAX_LOG_ENTRIES));
    };

    const sendServices = () => {
      const ws = wsRef.current;
      if (!ws || ws.readyState !== WebSocket.OPEN) return;
      ws.send(JSON.stringify({ services: lastServicesRef.current }));
    };

    const scheduleReconnect = () => {
      if (!mountedRef.current || reconnectTimerRef.current) return;
      reconnectTimerRef.current = setTimeout(() => {
        reconnectTimerRef.current = null;
        connect();
      }, RECONNECT_DELAY_MS);
    };

    const connect = () => {
      if (!mountedRef.current) return;
      if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) return;

      const ws = new WebSocket(
        baseUrl.replace(/^http/, "ws") + "/logs/stream",
      );
      wsRef.current = ws;

      ws.onopen = () => {
        if (!mountedRef.current) {
          ws.close();
          return;
        }
        setConnected(true);
        sendServices();
      };

      ws.onmessage = (event) => {
        let batch: StreamBatch;
        try {
          batch = JSON.parse(event.data);
        } catch {
          return;
        }
        if (batch.dropped) {
          console.warn(`log stream dropped ${batch.dropped} lines`);
        }
        for (const entry of batch.entries ?? []) {
          if (entry.error) {
            // バックエンドのエラーは情報として扱う
            console.warn("log stream error", entry);
            continue;
          }
          if (typeof entry.service !== "string" || typeof entry.line !== "string") {
            continue;
          }
          pendingRef.current.push({
            id: ++entryIdRef.current,
            service: entry.service,
            line: entry.line,
          });
        }
      };

      ws.onclose = () => {
        if (wsRef.current === ws) {
          wsRef.current = null;
        }
        setConnected(false);
        scheduleReconnect();
      };

      ws.onerror = () => {
        setConnected(false);
      };
    };

    connect();
    flushTimerRef.current = setInterval(flushPending, FLUSH_INTERVAL_MS);

    return () => {
      mountedRef.current = false;

      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current);
        reconnectTimerRef.current = null;
      }

      if (flushTimerRef.current) {
        clearInterval(flushTimerRef.current);
        flushTimerRef.current = null;
      }

      const ws = wsRef.current;
      wsRef.current = null;
      if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
        ws.close();
      }

      setConnected(false);
    };
  }, [baseUrl]);

  useEffect(() => {
    lastServicesRef.current = servicePayload;
    const ws = wsRef.current;
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ services: servicePayload }));
    }
  }, [servicePayload]);

  return { entries, connected, clear };
}
