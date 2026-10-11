import { describe, expect, it } from "vitest";
import { parseEvent } from "./logfmt";

describe("parseEvent", () => {
  it("名前とフィールドを読む", () => {
    expect(
      parseEvent("evt=sequencer.state_changed from=IDLE to=NAVIGATING index=3"),
    ).toEqual({
      name: "sequencer.state_changed",
      domain: "sequencer",
      fields: { from: "IDLE", to: "NAVIGATING", index: "3" },
    });
  });

  it("フィールドがなくてもよい", () => {
    expect(parseEvent("evt=node.started")).toEqual({
      name: "node.started",
      domain: "node",
      fields: {},
    });
  });

  // Python 側(mg_utils/test/test_event_log.py)の往復テストと同じ値
  it("引用符で囲んだ値と、エスケープを戻す", () => {
    const body =
      'evt=driver.disconnected reason="read timeout" quote="say \\"hi\\"" path="a\\\\b" multi="x\\ny" empty="" eq="k=v" index=3 ok=true';
    expect(parseEvent(body)?.fields).toEqual({
      reason: "read timeout",
      quote: 'say "hi"',
      path: "a\\b",
      multi: "x\ny",
      empty: "",
      eq: "k=v",
      index: "3",
      ok: "true",
    });
  });

  it("イベントでない本文は null", () => {
    expect(parseEvent("Serial device connected")).toBeNull();
    expect(parseEvent("  evt=node.started")).toBeNull();
    expect(parseEvent("FSM: IDLE -> NAVIGATING")).toBeNull();
    expect(parseEvent("")).toBeNull();
  });
});
