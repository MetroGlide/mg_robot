import { describe, expect, it } from "vitest";
import {
  ACTIVITY_CHANGED_EVENT,
  ActivityHistoryEntry,
  appendEntries,
  durationsSec,
  toActivityEntry,
  toTsv,
} from "./activityHistory";

function event(timeMs: number, fields: Record<string, string>) {
  return { name: ACTIVITY_CHANGED_EVENT, timeMs, fields };
}

function entry(
  timeMs: number,
  from: string,
  to: string,
  fromSec = "0",
  sinceSec = "0",
) {
  return toActivityEntry(
    event(timeMs, {
      from,
      to,
      text: to,
      from_sec: fromSec,
      since_sec: sinceSec,
    }),
  ) as ActivityHistoryEntry;
}

describe("toActivityEntry", () => {
  it("活動の変化のイベントだけを読む", () => {
    expect(
      toActivityEntry({ name: "sequencer.paused", timeMs: 0, fields: {} }),
    ).toBeNull();
    expect(toActivityEntry(event(0, { from: "a" }))).toBeNull();
  });

  it("開始はすでに続いていた秒数だけさかのぼる", () => {
    const e = toActivityEntry(
      event(10_000, {
        from: "navigating",
        to: "recovery",
        text: "リカバリー: 後退",
        from_sec: "12.5",
        since_sec: "1.5",
        index: "2",
        total: "5",
      }),
    );
    expect(e).toMatchObject({
      response: "",
      startMs: 8_500,
      code: "recovery",
      from: "navigating",
      fromSec: 12.5,
      text: "リカバリー: 後退",
      detail: "",
      index: 2,
      total: 5,
    });
  });

  it("地点がないときは -1", () => {
    expect(entry(0, "", "idle")).toMatchObject({ index: -1, total: -1 });
  });
});

describe("appendEntries", () => {
  it("重複を除き、上限を超えたら古いものから捨てる", () => {
    const a = entry(1000, "", "idle");
    const b = entry(2000, "idle", "navigating");
    const c = entry(3000, "navigating", "paused");
    const once = appendEntries([], [a, b], 2);
    expect(appendEntries(once, [a, b], 2)).toBe(once);
    expect(appendEntries(once, [c], 2).map((e) => e.code)).toEqual([
      "navigating",
      "paused",
    ]);
  });
});

describe("durationsSec", () => {
  it("次の件が数えた前の活動の秒数を使い、続いている件と欠けた件は null", () => {
    const entries = [
      entry(1000, "", "idle"),
      entry(5000, "idle", "navigating", "4.0"),
      entry(9000, "recovery", "paused", "2.0"),
    ];
    expect(durationsSec(entries)).toEqual([4.0, null, null]);
  });
});

describe("toTsv", () => {
  it("見出しと、古い順の行を出す", () => {
    const entries = [
      entry(1000, "", "idle"),
      entry(5000, "idle", "navigating", "4.0"),
    ];
    const lines = toTsv(entries, (e) => `lamp:${e.code}`)
      .trimEnd()
      .split("\n");
    expect(lines[0]).toBe(
      "start\tduration_sec\tlamp\tcode\ttext\tdetail\twaypoint",
    );
    expect(lines[1].split("\t").slice(1)).toEqual([
      "4.0",
      "lamp:idle",
      "idle",
      "idle",
      "",
      "",
    ]);
    expect(lines[2].split("\t").slice(1, 4)).toEqual([
      "",
      "lamp:navigating",
      "navigating",
    ]);
  });
});

describe("toActivityEntry のランプの意味", () => {
  it("イベントの lamp_response・lamp_motion を持つ", () => {
    const e = toActivityEntry(
      event(0, {
        from: "",
        to: "recovery",
        lamp_response: "watch",
        lamp_motion: "handling",
      }),
    );
    expect(e).toMatchObject({ response: "watch", motion: "handling" });
  });
});
