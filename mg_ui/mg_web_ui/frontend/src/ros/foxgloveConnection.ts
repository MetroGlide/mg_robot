import { AdvertisedSchema, decodePayload, encodePayload } from "./codec";
import { SCHEMAS } from "./schemas";

export type ConnectionStatus =
  | "connecting"
  | "connected"
  | "disconnected"
  | "error";

interface ServerChannel extends AdvertisedSchema {
  id: number;
  topic: string;
}

interface ServiceInfo {
  id: number;
  request?: AdvertisedSchema;
  response?: AdvertisedSchema;
}

interface PendingCall {
  resolve: (value: unknown) => void;
  reject: (reason: Error) => void;
  response?: AdvertisedSchema;
  timer: ReturnType<typeof setTimeout>;
}

interface ClientChannel {
  id: number;
  schema: AdvertisedSchema | undefined;
  /** この時刻より前に送ったメッセージは、DDS のマッチング前で捨てられるおそれがある */
  readyAt: number;
  queue: Uint8Array[];
  flushTimer: ReturnType<typeof setTimeout> | null;
}

export interface ConnectionOptions {
  url: string;
  createSocket?: (url: string, protocols: string[]) => WebSocket;
  reconnectIntervalMs?: number;
  serviceTimeoutMs?: number;
  publisherWarmupMs?: number;
  clientSchemas?: Record<string, AdvertisedSchema>;
}

const SUBPROTOCOLS = ["foxglove.websocket.v1", "foxglove.sdk.v1"];

const OPCODE_CLIENT_MESSAGE = 0x01;
const OPCODE_SERVICE_CALL_REQUEST = 0x02;
const OPCODE_MESSAGE_DATA = 0x01;
const OPCODE_SERVICE_CALL_RESPONSE = 0x03;

const MESSAGE_DATA_HEADER_BYTES = 1 + 4 + 8;

/** トピック名・サービス名の先頭の "/" の有無を揃える */
export function normalizeName(name: string): string {
  return name.startsWith("/") ? name.slice(1) : name;
}

type Listener = (data: unknown) => void;

/**
 * foxglove_bridge への接続。
 *
 * 購読はトピックごとのリスナーの登録として接続とは独立に保持し、
 * advertise / unadvertise / 再接続のたびに、登録内容と現在のチャネルを突き合わせて
 * サーバ側の購読を張り直す。そのため subscribe はいつ呼んでもよく、
 * ブリッジ側のノードが再起動してチャネルの id が変わっても購読が復帰する。
 */
export class FoxgloveConnection {
  private readonly options: Required<Omit<ConnectionOptions, "url">> & {
    url: string;
  };
  private ws: WebSocket | null = null;
  private started = false;
  private statusValue: ConnectionStatus = "connecting";
  private readonly statusListeners = new Set<(s: ConnectionStatus) => void>();
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;

  private readonly channels = new Map<string, ServerChannel>();
  private readonly listeners = new Map<string, Map<number, Listener>>();
  private readonly active = new Map<
    string,
    { subId: number; channel: ServerChannel }
  >();
  private readonly topicBySubId = new Map<number, string>();
  private readonly lastMessageAt = new Map<string, number>();
  private listenerCounter = 0;
  private subCounter = 0;

  private readonly services = new Map<string, ServiceInfo>();
  private readonly pendingCalls = new Map<number, PendingCall>();
  private callCounter = 0;

  private readonly clientChannels = new Map<string, ClientChannel>();
  private readonly explicitPublishers = new Map<
    string,
    { schemaName: string; count: number }
  >();
  private clientChannelCounter = 0;

  constructor(options: ConnectionOptions) {
    this.options = {
      createSocket: (url, protocols) => new WebSocket(url, protocols),
      reconnectIntervalMs: 3000,
      serviceTimeoutMs: 10000,
      publisherWarmupMs: 400,
      clientSchemas: SCHEMAS,
      ...options,
    };
  }

  get status(): ConnectionStatus {
    return this.statusValue;
  }

  onStatusChange(listener: (status: ConnectionStatus) => void): () => void {
    this.statusListeners.add(listener);
    return () => this.statusListeners.delete(listener);
  }

  start(): void {
    if (this.started) return;
    this.started = true;
    this.connect();
  }

  stop(): void {
    this.started = false;
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    const ws = this.ws;
    this.ws = null;
    if (ws) {
      ws.onopen = null;
      ws.onerror = null;
      ws.onclose = null;
      ws.onmessage = null;
      if (ws.readyState === WebSocket.CONNECTING) {
        ws.addEventListener("open", () => ws.close());
      } else {
        ws.close();
      }
    }
    this.resetConnectionState();
  }

  /** トピックのメッセージを購読する。戻り値で購読を解除する。 */
  subscribe(topic: string, onMessage: Listener): () => void {
    const key = normalizeName(topic);
    const id = ++this.listenerCounter;
    let topicListeners = this.listeners.get(key);
    if (!topicListeners) {
      topicListeners = new Map();
      this.listeners.set(key, topicListeners);
    }
    topicListeners.set(id, onMessage);
    this.reconcile();

    return () => {
      const current = this.listeners.get(key);
      if (!current || !current.delete(id)) return;
      if (current.size === 0) this.listeners.delete(key);
      this.reconcile();
    };
  }

  /** トピックの最後のメッセージを受信した時刻(ms)。未受信なら null。 */
  getLastMessageAt(topic: string): number | null {
    return this.lastMessageAt.get(normalizeName(topic)) ?? null;
  }

  callService(service: string, payload: unknown): Promise<unknown> {
    return new Promise((resolve, reject) => {
      if (!this.isOpen()) {
        reject(new Error(`not connected (status: ${this.statusValue})`));
        return;
      }
      const info = this.services.get(normalizeName(service));
      if (info === undefined) {
        reject(
          new Error(
            `service not found: ${service} (status: ${this.statusValue})`,
          ),
        );
        return;
      }

      let payloadBytes: Uint8Array;
      try {
        payloadBytes = encodePayload(payload, info.request);
      } catch (e) {
        reject(e instanceof Error ? e : new Error(String(e)));
        return;
      }

      const callId = ++this.callCounter;
      const timer = setTimeout(() => {
        if (this.pendingCalls.delete(callId)) {
          reject(new Error("service call timed out"));
        }
      }, this.options.serviceTimeoutMs);
      this.pendingCalls.set(callId, {
        resolve,
        reject,
        response: info.response,
        timer,
      });

      const encodingBytes = new TextEncoder().encode(
        info.request?.encoding ?? "json",
      );
      const buf = new ArrayBuffer(
        1 + 4 + 4 + 4 + encodingBytes.byteLength + payloadBytes.byteLength,
      );
      const view = new DataView(buf);
      let offset = 0;
      view.setUint8(offset++, OPCODE_SERVICE_CALL_REQUEST);
      view.setUint32(offset, info.id, true);
      offset += 4;
      view.setUint32(offset, callId, true);
      offset += 4;
      view.setUint32(offset, encodingBytes.byteLength, true);
      offset += 4;
      new Uint8Array(buf).set(encodingBytes, offset);
      offset += encodingBytes.byteLength;
      new Uint8Array(buf).set(payloadBytes, offset);
      this.ws!.send(buf);
    });
  }

  /**
   * トピックへの publish を事前に準備する。
   * 最初の publish がマッチング前に捨てられないよう、使う前に呼んでおく。
   */
  advertise(topic: string, schemaName: string): () => void {
    const entry = this.explicitPublishers.get(topic);
    if (entry) {
      entry.count++;
    } else {
      this.explicitPublishers.set(topic, { schemaName, count: 1 });
      if (this.isOpen() && !this.clientChannels.has(topic)) {
        this.openClientChannel(topic, schemaName);
      }
    }
    return () => {
      const current = this.explicitPublishers.get(topic);
      if (!current) return;
      if (--current.count > 0) return;
      this.explicitPublishers.delete(topic);
      this.closeClientChannel(topic);
    };
  }

  /**
   * メッセージを publish する。未接続の場合は何もしない。
   * 初めて publish するトピックは、マッチングが済むまで送信を遅らせる。
   */
  publish(topic: string, schemaName: string, data: unknown): void {
    if (!this.isOpen()) return;
    const channel =
      this.clientChannels.get(topic) ?? this.openClientChannel(topic, schemaName);
    const bytes = encodePayload(data, channel.schema);

    if (channel.queue.length > 0 || Date.now() < channel.readyAt) {
      channel.queue.push(bytes);
      this.scheduleFlush(topic, channel);
      return;
    }
    this.sendClientMessage(channel, bytes);
  }

  private isOpen(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN;
  }

  private setStatus(status: ConnectionStatus): void {
    if (this.statusValue === status) return;
    this.statusValue = status;
    for (const listener of this.statusListeners) listener(status);
  }

  private connect(): void {
    if (!this.started) return;
    const ws = this.options.createSocket(this.options.url, SUBPROTOCOLS);
    ws.binaryType = "arraybuffer";
    this.ws = ws;

    ws.onopen = () => {
      if (this.ws !== ws) return;
      this.setStatus("connected");
      for (const [topic, { schemaName }] of this.explicitPublishers) {
        this.openClientChannel(topic, schemaName);
      }
    };
    ws.onerror = () => {
      if (this.ws === ws) this.setStatus("error");
    };
    ws.onclose = () => {
      if (this.ws !== ws) return;
      this.ws = null;
      this.resetConnectionState();
      this.setStatus("disconnected");
      if (this.started) {
        this.reconnectTimer = setTimeout(() => {
          this.reconnectTimer = null;
          this.connect();
        }, this.options.reconnectIntervalMs);
      }
    };
    ws.onmessage = (event: MessageEvent) => {
      if (this.ws !== ws) return;
      if (typeof event.data === "string") {
        this.handleText(event.data);
      } else if (event.data instanceof ArrayBuffer) {
        this.handleBinary(event.data);
      }
    };
  }

  /** 接続ごとの状態を捨てる。購読のリスナー登録と、明示した publisher は残す。 */
  private resetConnectionState(): void {
    this.channels.clear();
    this.active.clear();
    this.topicBySubId.clear();
    this.services.clear();
    for (const call of this.pendingCalls.values()) {
      clearTimeout(call.timer);
      call.reject(new Error("connection closed"));
    }
    this.pendingCalls.clear();
    for (const channel of this.clientChannels.values()) {
      if (channel.flushTimer) clearTimeout(channel.flushTimer);
    }
    this.clientChannels.clear();
  }

  private handleText(text: string): void {
    let msg: Record<string, unknown>;
    try {
      msg = JSON.parse(text) as Record<string, unknown>;
    } catch {
      return;
    }
    switch (msg["op"]) {
      case "advertise":
        for (const ch of msg["channels"] as ServerChannel[]) {
          this.channels.set(normalizeName(ch.topic), ch);
        }
        this.reconcile();
        break;
      case "unadvertise":
        this.removeChannels(msg["channelIds"] as number[]);
        break;
      case "advertiseServices":
        for (const svc of msg["services"] as Array<
          ServiceInfo & { name: string }
        >) {
          this.services.set(normalizeName(svc.name), {
            id: svc.id,
            request: svc.request,
            response: svc.response,
          });
        }
        break;
      case "unadvertiseServices":
        for (const id of msg["serviceIds"] as number[]) {
          for (const [key, info] of this.services) {
            if (info.id === id) this.services.delete(key);
          }
        }
        break;
      case "serviceCallFailure": {
        const call = this.takePending(msg["callId"] as number);
        call?.reject(new Error(String(msg["message"])));
        break;
      }
    }
  }

  private handleBinary(buffer: ArrayBuffer): void {
    const view = new DataView(buffer);
    const opcode = view.getUint8(0);

    if (opcode === OPCODE_MESSAGE_DATA) {
      const topic = this.topicBySubId.get(view.getUint32(1, true));
      if (topic === undefined) return;
      const listeners = this.listeners.get(topic);
      const channel = this.active.get(topic)?.channel;
      if (!listeners || listeners.size === 0 || !channel) return;

      this.lastMessageAt.set(topic, Date.now());
      const decoded = decodePayload(
        buffer.slice(MESSAGE_DATA_HEADER_BYTES),
        channel,
      );
      for (const listener of [...listeners.values()]) {
        try {
          listener(decoded);
        } catch (e) {
          console.error(`listener for ${topic} failed`, e);
        }
      }
    } else if (opcode === OPCODE_SERVICE_CALL_RESPONSE) {
      const call = this.takePending(view.getUint32(1 + 4, true));
      if (!call) return;
      const encodingLength = view.getUint32(1 + 4 + 4, true);
      const headerEnd = 1 + 4 + 4 + 4 + encodingLength;
      const encoding = new TextDecoder().decode(
        buffer.slice(1 + 4 + 4 + 4, headerEnd),
      );
      call.resolve(
        decodePayload(
          buffer.slice(headerEnd),
          call.response ? { ...call.response, encoding } : undefined,
        ),
      );
    }
  }

  private takePending(callId: number): PendingCall | undefined {
    const call = this.pendingCalls.get(callId);
    if (call) {
      clearTimeout(call.timer);
      this.pendingCalls.delete(callId);
    }
    return call;
  }

  private removeChannels(channelIds: number[]): void {
    const removed = new Set(channelIds);
    for (const [key, channel] of this.channels) {
      if (removed.has(channel.id)) this.channels.delete(key);
    }
    // サーバ側でチャネルごと消えた購読は、再 advertise 後に新しい id で張り直す
    for (const [key, sub] of this.active) {
      if (removed.has(sub.channel.id)) {
        this.active.delete(key);
        this.topicBySubId.delete(sub.subId);
      }
    }
  }

  /** リスナーの登録内容と現在のチャネルを突き合わせ、サーバ側の購読を過不足なく張る。 */
  private reconcile(): void {
    if (!this.isOpen()) return;

    for (const [key, listeners] of this.listeners) {
      if (listeners.size === 0) continue;
      const channel = this.channels.get(key);
      if (!channel) continue;
      const current = this.active.get(key);
      if (current && current.channel.id === channel.id) continue;
      if (current) this.unsubscribeServer(key);
      this.subscribeServer(key, channel);
    }
    for (const key of [...this.active.keys()]) {
      if ((this.listeners.get(key)?.size ?? 0) === 0) {
        this.unsubscribeServer(key);
      }
    }
  }

  private subscribeServer(key: string, channel: ServerChannel): void {
    const subId = ++this.subCounter;
    this.active.set(key, { subId, channel });
    this.topicBySubId.set(subId, key);
    this.sendJson({
      op: "subscribe",
      subscriptions: [{ id: subId, channelId: channel.id }],
    });
  }

  private unsubscribeServer(key: string): void {
    const current = this.active.get(key);
    if (!current) return;
    this.active.delete(key);
    this.topicBySubId.delete(current.subId);
    this.sendJson({ op: "unsubscribe", subscriptionIds: [current.subId] });
  }

  private openClientChannel(topic: string, schemaName: string): ClientChannel {
    const schema = this.options.clientSchemas[schemaName];
    const channel: ClientChannel = {
      id: ++this.clientChannelCounter,
      schema,
      readyAt: Date.now() + this.options.publisherWarmupMs,
      queue: [],
      flushTimer: null,
    };
    this.clientChannels.set(topic, channel);
    this.sendJson({
      op: "advertise",
      channels: [
        {
          id: channel.id,
          topic,
          encoding: schema?.encoding ?? "json",
          schemaName,
        },
      ],
    });
    return channel;
  }

  private closeClientChannel(topic: string): void {
    const channel = this.clientChannels.get(topic);
    if (!channel) return;
    if (channel.flushTimer) clearTimeout(channel.flushTimer);
    this.clientChannels.delete(topic);
    this.sendJson({ op: "unadvertise", channelIds: [channel.id] });
  }

  private scheduleFlush(topic: string, channel: ClientChannel): void {
    if (channel.flushTimer) return;
    const delay = Math.max(0, channel.readyAt - Date.now());
    channel.flushTimer = setTimeout(() => {
      channel.flushTimer = null;
      if (this.clientChannels.get(topic) !== channel) return;
      const queued = channel.queue.splice(0);
      for (const bytes of queued) this.sendClientMessage(channel, bytes);
    }, delay);
  }

  private sendClientMessage(channel: ClientChannel, payload: Uint8Array): void {
    if (!this.isOpen()) return;
    const buf = new ArrayBuffer(1 + 4 + payload.byteLength);
    const view = new DataView(buf);
    view.setUint8(0, OPCODE_CLIENT_MESSAGE);
    view.setUint32(1, channel.id, true);
    new Uint8Array(buf).set(payload, 5);
    this.ws!.send(buf);
  }

  private sendJson(message: unknown): void {
    if (this.isOpen()) this.ws!.send(JSON.stringify(message));
  }
}
