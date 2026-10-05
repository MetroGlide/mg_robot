import { describe, expect, it } from "vitest";
import { DiagnosticStatus } from "../types";
import { countOk, summarizeHealth } from "./healthSummary";

const status = (name: string, level = 0): DiagnosticStatus => ({
  name,
  level,
  message: level === 0 ? "ok" : "low rate",
  hardware_id: "",
  values: [],
});

describe("summarizeHealth", () => {
  it("センサのトピック、その他のトピック、ノードに分類する", () => {
    const s = summarizeHealth([
      status("topic/scan_top_lidar"),
      status("topic/odom", 1),
      status("topic/cmd_vel"),
      status("node/amcl"),
      status("localization_supervisor"),
    ]);
    expect(s.sensors.map((i) => i.name)).toEqual(["scan_top_lidar", "odom"]);
    expect(s.topics.map((i) => i.name)).toEqual(["cmd_vel"]);
    expect(s.nodes.map((i) => i.name)).toEqual(["amcl"]);
  });

  it("名前が topic/ や node/ で始まらない診断は含めない", () => {
    const s = summarizeHealth([status("localization_supervisor")]);
    expect(s).toEqual({ sensors: [], topics: [], nodes: [] });
  });
});

describe("countOk", () => {
  it("level 0 だけを OK として数える", () => {
    const s = summarizeHealth([
      status("topic/scan_top_lidar"),
      status("topic/odom", 1),
    ]);
    expect(countOk(s.sensors)).toEqual({ ok: 1, total: 2 });
  });
});
