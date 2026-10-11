import { describe, expect, it } from "vitest";
import { isNotable, toEventEntry } from "./EventStreamContext";

const log = (msg: string, level = 20) => ({
  stamp: { sec: 1760154000, nanosec: 500_000_000 },
  level,
  name: "driver_watchdog_node",
  msg,
});

describe("toEventEntry", () => {
  it("イベントのログを、時刻・レベル・ドメインつきの項目にする", () => {
    const entry = toEventEntry(log("evt=watchdog.stale source=top_lidar", 40), 7);
    expect(entry).toMatchObject({
      seq: 7,
      level: "ERROR",
      node: "driver_watchdog_node",
      name: "watchdog.stale",
      domain: "watchdog",
      fields: { source: "top_lidar" },
    });
    expect(entry?.timeMs).toBe(1760154000500);
  });

  it("イベントでないログは捨てる", () => {
    expect(toEventEntry(log("plain message"), 1)).toBeNull();
  });

  it("知らないレベルは INFO にする", () => {
    expect(toEventEntry(log("evt=a.b", 99), 1)?.level).toBe("INFO");
  });
});

describe("isNotable", () => {
  it("WARN 以上を目立たせる", () => {
    expect(isNotable("INFO")).toBe(false);
    expect(isNotable("WARN")).toBe(true);
    expect(isNotable("ERROR")).toBe(true);
    expect(isNotable("FATAL")).toBe(true);
  });
});
