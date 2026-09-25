import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { encodePayload } from "./codec";
import { FoxgloveConnection } from "./foxgloveConnection";

class FakeSocket {
  static OPEN = 1;
  binaryType = "";
  readyState = 0;
  sent: (string | ArrayBuffer)[] = [];
  onopen: (() => void) | null = null;
  onclose: (() => void) | null = null;
  onerror: (() => void) | null = null;
  onmessage: ((e: { data: unknown }) => void) | null = null;

  send(data: string | ArrayBuffer) {
    this.sent.push(data);
  }
  close() {
    this.readyState = 3;
    this.onclose?.();
  }
  addEventListener() {}

  open() {
    this.readyState = 1;
    this.onopen?.();
  }
  receiveJson(message: unknown) {
    this.onmessage?.({ data: JSON.stringify(message) });
  }
  receiveBinary(data: ArrayBuffer) {
    this.onmessage?.({ data });
  }
  sentJson(op: string) {
    return this.sent
      .filter((d): d is string => typeof d === "string")
      .map((d) => JSON.parse(d))
      .filter((m) => m.op === op);
  }
}

const TWIST_SCHEMA = {
  encoding: "cdr",
  schemaName: "geometry_msgs/msg/Twist",
  schema:
    "geometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular\n" +
    "================================================================================\n" +
    "MSG: geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z",
};

function channel(id: number, topic: string) {
  return { id, topic, ...TWIST_SCHEMA };
}

function messageFrame(subId: number, value: number) {
  const payload = encodePayload(
    { linear: { x: value, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
    TWIST_SCHEMA,
  );
  const buf = new ArrayBuffer(13 + payload.byteLength);
  const view = new DataView(buf);
  view.setUint8(0, 0x01);
  view.setUint32(1, subId, true);
  new Uint8Array(buf).set(payload, 13);
  return buf;
}

let sockets: FakeSocket[];

function createConnection(extra: Record<string, unknown> = {}) {
  const connection = new FoxgloveConnection({
    url: "ws://test",
    createSocket: () => {
      const socket = new FakeSocket();
      sockets.push(socket);
      return socket as unknown as WebSocket;
    },
    reconnectIntervalMs: 1000,
    serviceTimeoutMs: 500,
    publisherWarmupMs: 300,
    clientSchemas: { "geometry_msgs/msg/Twist": TWIST_SCHEMA },
    ...extra,
  });
  connection.start();
  return connection;
}

beforeEach(() => {
  sockets = [];
  vi.useFakeTimers();
  vi.stubGlobal("WebSocket", FakeSocket);
});

afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

describe("subscribe", () => {
  it("subscribes to the server once the channel is advertised", () => {
    const connection = createConnection();
    const received: unknown[] = [];
    connection.subscribe("/cmd_vel", (m) => received.push(m));
    const ws = sockets[0];
    ws.open();
    expect(ws.sentJson("subscribe")).toHaveLength(0);

    ws.receiveJson({ op: "advertise", channels: [channel(7, "/cmd_vel")] });

    const [sub] = ws.sentJson("subscribe");
    expect(sub.subscriptions[0].channelId).toBe(7);
    ws.receiveBinary(messageFrame(sub.subscriptions[0].id, 0.5));
    expect(received).toEqual([
      { linear: { x: 0.5, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
    ]);
  });

  it("matches topics with and without a leading slash", () => {
    const connection = createConnection();
    const received: unknown[] = [];
    connection.subscribe("slam_gnss_2d/path", (m) => received.push(m));
    const ws = sockets[0];
    ws.open();

    ws.receiveJson({
      op: "advertise",
      channels: [channel(3, "/slam_gnss_2d/path")],
    });

    expect(ws.sentJson("subscribe")).toHaveLength(1);
  });

  it("shares one server subscription between listeners and decodes once", () => {
    const connection = createConnection();
    const a: unknown[] = [];
    const b: unknown[] = [];
    connection.subscribe("/cmd_vel", (m) => a.push(m));
    connection.subscribe("/cmd_vel", (m) => b.push(m));
    const ws = sockets[0];
    ws.open();
    ws.receiveJson({ op: "advertise", channels: [channel(7, "/cmd_vel")] });

    const subs = ws.sentJson("subscribe");
    expect(subs).toHaveLength(1);
    ws.receiveBinary(messageFrame(subs[0].subscriptions[0].id, 1));
    expect(a[0]).toBe(b[0]);
  });

  it("unsubscribes from the server when the last listener leaves", () => {
    const connection = createConnection();
    const unsubscribeA = connection.subscribe("/cmd_vel", () => {});
    const unsubscribeB = connection.subscribe("/cmd_vel", () => {});
    const ws = sockets[0];
    ws.open();
    ws.receiveJson({ op: "advertise", channels: [channel(7, "/cmd_vel")] });

    unsubscribeA();
    expect(ws.sentJson("unsubscribe")).toHaveLength(0);
    unsubscribeB();
    expect(ws.sentJson("unsubscribe")).toHaveLength(1);
    unsubscribeB();
    expect(ws.sentJson("unsubscribe")).toHaveLength(1);
  });

  it("resubscribes with the new channel id after unadvertise and advertise", () => {
    const connection = createConnection();
    const received: unknown[] = [];
    connection.subscribe("/cmd_vel", (m) => received.push(m));
    const ws = sockets[0];
    ws.open();
    ws.receiveJson({ op: "advertise", channels: [channel(7, "/cmd_vel")] });

    ws.receiveJson({ op: "unadvertise", channelIds: [7] });
    ws.receiveJson({ op: "advertise", channels: [channel(9, "/cmd_vel")] });

    const subs = ws.sentJson("subscribe");
    expect(subs).toHaveLength(2);
    expect(subs[1].subscriptions[0].channelId).toBe(9);
    ws.receiveBinary(messageFrame(subs[1].subscriptions[0].id, 2));
    expect(received).toHaveLength(1);
  });

  it("replaces the subscription when a topic is re-advertised with a new id", () => {
    const connection = createConnection();
    connection.subscribe("/cmd_vel", () => {});
    const ws = sockets[0];
    ws.open();
    ws.receiveJson({ op: "advertise", channels: [channel(7, "/cmd_vel")] });

    ws.receiveJson({ op: "advertise", channels: [channel(8, "/cmd_vel")] });

    expect(ws.sentJson("unsubscribe")).toHaveLength(1);
    expect(ws.sentJson("subscribe")).toHaveLength(2);
  });

  it("does not subscribe again for a repeated advertise of the same channel", () => {
    const connection = createConnection();
    connection.subscribe("/cmd_vel", () => {});
    const ws = sockets[0];
    ws.open();
    ws.receiveJson({ op: "advertise", channels: [channel(7, "/cmd_vel")] });
    ws.receiveJson({ op: "advertise", channels: [channel(7, "/cmd_vel")] });

    expect(ws.sentJson("subscribe")).toHaveLength(1);
  });

  it("keeps listeners across reconnects and resubscribes", () => {
    const connection = createConnection();
    const received: unknown[] = [];
    connection.subscribe("/cmd_vel", (m) => received.push(m));
    const first = sockets[0];
    first.open();
    first.receiveJson({ op: "advertise", channels: [channel(7, "/cmd_vel")] });

    first.close();
    expect(connection.status).toBe("disconnected");
    vi.advanceTimersByTime(1000);
    const second = sockets[1];
    second.open();
    second.receiveJson({ op: "advertise", channels: [channel(2, "/cmd_vel")] });

    const [sub] = second.sentJson("subscribe");
    expect(sub.subscriptions[0].channelId).toBe(2);
    second.receiveBinary(messageFrame(sub.subscriptions[0].id, 3));
    expect(received).toHaveLength(1);
  });

  it("isolates a throwing listener from the others", () => {
    const connection = createConnection();
    const errors = vi.spyOn(console, "error").mockImplementation(() => {});
    const received: unknown[] = [];
    connection.subscribe("/cmd_vel", () => {
      throw new Error("boom");
    });
    connection.subscribe("/cmd_vel", (m) => received.push(m));
    const ws = sockets[0];
    ws.open();
    ws.receiveJson({ op: "advertise", channels: [channel(7, "/cmd_vel")] });
    const [sub] = ws.sentJson("subscribe");

    ws.receiveBinary(messageFrame(sub.subscriptions[0].id, 1));

    expect(received).toHaveLength(1);
    expect(errors).toHaveBeenCalled();
  });

  it("records when the last message arrived", () => {
    const connection = createConnection();
    connection.subscribe("/cmd_vel", () => {});
    const ws = sockets[0];
    ws.open();
    ws.receiveJson({ op: "advertise", channels: [channel(7, "/cmd_vel")] });
    expect(connection.getLastMessageAt("/cmd_vel")).toBeNull();

    vi.setSystemTime(5000);
    const [sub] = ws.sentJson("subscribe");
    ws.receiveBinary(messageFrame(sub.subscriptions[0].id, 1));

    expect(connection.getLastMessageAt("cmd_vel")).toBe(5000);
  });
});

describe("services", () => {
  const SERVICE = {
    op: "advertiseServices",
    services: [{ id: 4, name: "/trigger" }],
  };

  it("rejects when the service is unknown", async () => {
    const connection = createConnection();
    sockets[0].open();

    await expect(connection.callService("/trigger", {})).rejects.toThrow(
      "service not found",
    );
  });

  it("rejects pending calls immediately when the connection closes", async () => {
    const connection = createConnection();
    const ws = sockets[0];
    ws.open();
    ws.receiveJson(SERVICE);
    const call = connection.callService("/trigger", {});
    const assertion = expect(call).rejects.toThrow("connection closed");

    ws.close();

    await assertion;
  });

  it("times out and clears the pending call", async () => {
    const connection = createConnection();
    const ws = sockets[0];
    ws.open();
    ws.receiveJson(SERVICE);
    const call = connection.callService("/trigger", {});
    const assertion = expect(call).rejects.toThrow("timed out");

    vi.advanceTimersByTime(500);

    await assertion;
  });

  it("rejects on serviceCallFailure", async () => {
    const connection = createConnection();
    const ws = sockets[0];
    ws.open();
    ws.receiveJson(SERVICE);
    const call = connection.callService("/trigger", {});
    const assertion = expect(call).rejects.toThrow("no such thing");

    ws.receiveJson({ op: "serviceCallFailure", callId: 1, message: "no such thing" });

    await assertion;
  });
});

describe("publish", () => {
  const binaryFrames = (ws: FakeSocket) =>
    ws.sent.filter((d): d is ArrayBuffer => typeof d !== "string");
  const twist = { linear: { x: 1, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };

  it("delays the first message until the publisher is ready", () => {
    const connection = createConnection();
    const ws = sockets[0];
    ws.open();

    connection.publish("/cmd_vel", "geometry_msgs/msg/Twist", twist);

    expect(ws.sentJson("advertise")).toHaveLength(1);
    expect(binaryFrames(ws)).toHaveLength(0);
    vi.advanceTimersByTime(300);
    expect(binaryFrames(ws)).toHaveLength(1);
  });

  it("keeps message order while queued and sends directly afterwards", () => {
    const connection = createConnection();
    const ws = sockets[0];
    ws.open();
    connection.publish("/cmd_vel", "geometry_msgs/msg/Twist", twist);
    connection.publish("/cmd_vel", "geometry_msgs/msg/Twist", twist);
    vi.advanceTimersByTime(300);
    expect(binaryFrames(ws)).toHaveLength(2);

    connection.publish("/cmd_vel", "geometry_msgs/msg/Twist", twist);

    expect(binaryFrames(ws)).toHaveLength(3);
    expect(ws.sentJson("advertise")).toHaveLength(1);
  });

  it("sends immediately when advertised ahead of time", () => {
    const connection = createConnection();
    const ws = sockets[0];
    ws.open();
    connection.advertise("/cmd_vel", "geometry_msgs/msg/Twist");
    vi.advanceTimersByTime(300);

    connection.publish("/cmd_vel", "geometry_msgs/msg/Twist", twist);

    expect(binaryFrames(ws)).toHaveLength(1);
    expect(ws.sentJson("advertise")).toHaveLength(1);
  });

  it("advertises explicit publishers again after reconnecting", () => {
    const connection = createConnection();
    connection.advertise("/cmd_vel", "geometry_msgs/msg/Twist");
    const first = sockets[0];
    first.open();
    expect(first.sentJson("advertise")).toHaveLength(1);

    first.close();
    vi.advanceTimersByTime(1000);
    const second = sockets[1];
    second.open();

    expect(second.sentJson("advertise")).toHaveLength(1);
  });

  it("unadvertises when the last explicit publisher is released", () => {
    const connection = createConnection();
    const ws = sockets[0];
    ws.open();
    const releaseA = connection.advertise("/cmd_vel", "geometry_msgs/msg/Twist");
    const releaseB = connection.advertise("/cmd_vel", "geometry_msgs/msg/Twist");

    releaseA();
    expect(ws.sentJson("unadvertise")).toHaveLength(0);
    releaseB();
    expect(ws.sentJson("unadvertise")).toHaveLength(1);
  });

  it("does nothing while disconnected", () => {
    const connection = createConnection();

    connection.publish("/cmd_vel", "geometry_msgs/msg/Twist", twist);

    expect(sockets[0].sent).toHaveLength(0);
  });

  it("throws instead of sending JSON on a cdr channel with a broken schema", () => {
    const connection = createConnection({
      clientSchemas: {
        "bad/msg/Broken": {
          encoding: "cdr",
          schemaName: "bad/msg/Broken",
          schema: "not_a_type field extra )(",
        },
      },
    });
    const ws = sockets[0];
    ws.open();

    expect(() =>
      connection.publish("/broken", "bad/msg/Broken", { a: 1 }),
    ).toThrow("cannot build a message writer");
    vi.advanceTimersByTime(1000);
    expect(ws.sent.filter((d) => typeof d !== "string")).toHaveLength(0);
  });
});

describe("lifecycle", () => {
  it("stops reconnecting after stop()", () => {
    const connection = createConnection();
    sockets[0].open();

    connection.stop();
    vi.advanceTimersByTime(10000);

    expect(sockets).toHaveLength(1);
  });

  it("reports status changes", () => {
    const connection = createConnection();
    const statuses: string[] = [];
    connection.onStatusChange((s) => statuses.push(s));

    sockets[0].open();
    sockets[0].close();

    expect(statuses).toEqual(["connected", "disconnected"]);
  });
});
