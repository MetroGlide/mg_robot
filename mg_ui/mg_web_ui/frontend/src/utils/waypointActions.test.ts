import { describe, expect, it } from "vitest";
import {
  buildMessageTemplate,
  describeAmclGate,
  describeGnssGate,
  describeServiceResponse,
  mapDisplayName,
  parseJsonObject,
  parseNavigationMode,
} from "./waypointActions";

describe("parseJsonObject", () => {
  it("treats empty input as an empty object", () => {
    expect(parseJsonObject("  ")).toEqual({ ok: true, value: {} });
  });

  it("parses an object", () => {
    expect(parseJsonObject('{"data": true}')).toEqual({
      ok: true,
      value: { data: true },
    });
  });

  it("rejects broken JSON and non-objects", () => {
    expect(parseJsonObject("{data:").ok).toBe(false);
    expect(parseJsonObject("[1]").ok).toBe(false);
    expect(parseJsonObject("3").ok).toBe(false);
    expect(parseJsonObject("null").ok).toBe(false);
  });
});

describe("buildMessageTemplate", () => {
  it("fills primitive fields with defaults", () => {
    expect(buildMessageTemplate("bool data")).toEqual({ data: false });
    expect(
      buildMessageTemplate("string requester_id\nbool active\nint32 n"),
    ).toEqual({ requester_id: "", active: false, n: 0 });
  });

  it("skips constants", () => {
    expect(buildMessageTemplate("uint8 NORMAL=0\nuint8 state")).toEqual({
      state: 0,
    });
  });

  it("expands nested messages and arrays", () => {
    const schema = `geometry_msgs/Vector3 linear
string[] names
float64[3] fixed
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z`;
    expect(buildMessageTemplate(schema)).toEqual({
      linear: { x: 0, y: 0, z: 0 },
      names: [],
      fixed: [0, 0, 0],
    });
  });

  it("resolves nested types written with and without the msg segment", () => {
    const schema = `geometry_msgs/msg/Vector3 v
================================================================================
MSG: geometry_msgs/msg/Vector3
float64 x`;
    expect(buildMessageTemplate(schema)).toEqual({ v: { x: 0 } });
  });

  it("returns an empty object for an empty request", () => {
    expect(buildMessageTemplate("")).toEqual({});
  });
});

describe("parseNavigationMode", () => {
  it("reads mode and behavior tree", () => {
    expect(
      parseNavigationMode('{"mode":"normal","behavior_tree":"a.xml"}'),
    ).toEqual({ mode: "normal", behavior_tree: "a.xml" });
  });

  it("returns null for anything else", () => {
    expect(parseNavigationMode("normal")).toBeNull();
    expect(parseNavigationMode('{"mode":"normal"}')).toBeNull();
    expect(parseNavigationMode("null")).toBeNull();
  });
});

describe("describeServiceResponse", () => {
  it("uses success and message when present", () => {
    expect(
      describeServiceResponse({ success: true, message: "attached" }),
    ).toEqual({
      ok: true,
      text: "attached",
    });
    expect(
      describeServiceResponse({ success: false, message: "failed" }),
    ).toEqual({
      ok: false,
      text: "failed",
    });
  });

  it("falls back to the success flag when the message is empty", () => {
    expect(describeServiceResponse({ success: true, message: "" })).toEqual({
      ok: true,
      text: "true",
    });
  });

  it("shows other responses as JSON and treats them as success", () => {
    expect(describeServiceResponse({})).toEqual({ ok: true, text: "{}" });
    expect(describeServiceResponse({ result: 1 })).toEqual({
      ok: true,
      text: '{"result":1}',
    });
  });
});

describe("describeAmclGate", () => {
  it("is unknown until the state arrives", () => {
    expect(describeAmclGate(null).label).toBe("UNKNOWN");
  });

  it("is ON when applied and wanted", () => {
    expect(
      describeAmclGate({ desired: true, applied: true, holders: [] }),
    ).toEqual({ label: "ON", detail: "" });
  });

  it("names who holds the gate closed", () => {
    expect(
      describeAmclGate({
        desired: false,
        applied: true,
        holders: ["supervisor", "waypoint"],
      }),
    ).toEqual({ label: "OFF", detail: "held by: supervisor, waypoint" });
  });

  it("is pending while the wanted value is not applied", () => {
    expect(
      describeAmclGate({
        desired: false,
        applied: false,
        holders: ["waypoint"],
      }),
    ).toEqual({
      label: "PENDING",
      detail: "OFF requested, not applied to the gate yet",
    });
  });
});

describe("describeGnssGate", () => {
  it("maps the published flag", () => {
    expect(describeGnssGate(null).label).toBe("UNKNOWN");
    expect(describeGnssGate(true).label).toBe("ON");
    expect(describeGnssGate(false).label).toBe("OFF");
  });
});

describe("mapDisplayName", () => {
  it("shows the file name", () => {
    expect(mapDisplayName("/root/ros2_data/map/localization_1.yaml")).toBe(
      "localization_1.yaml",
    );
  });

  it("shows a dash when nothing is loaded", () => {
    expect(mapDisplayName("")).toBe("—");
  });
});
