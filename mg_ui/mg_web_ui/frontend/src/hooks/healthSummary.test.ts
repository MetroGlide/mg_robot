import { describe, expect, it } from "vitest";
import { DiagnosticStatus } from "../types";
import { buildDiagnosticsTree } from "../utils/diagnosticsTree";
import { countOk, summarizeHealth } from "./healthSummary";

const status = (name: string, level = 0, message = "ok"): DiagnosticStatus => ({
  name,
  level,
  message,
  hardware_id: "",
  values: [],
});

const tree = (...statuses: DiagnosticStatus[]) =>
  buildDiagnosticsTree([status("/Robot"), ...statuses], "/Robot");

describe("summarizeHealth", () => {
  it("区分ごとに、元の診断を項目にする", () => {
    const regions = summarizeHealth(
      tree(
        status("/Robot/Drivers"),
        status("/Robot/Drivers/LiDAR"),
        status("/Robot/Drivers/LiDAR/driver_watchdog_node: top_lidar", 2, "no data for 2.0 s"),
        status("/Robot/Localization"),
        status("/Robot/Localization/Monitor"),
        status("/Robot/Localization/Monitor/localization_supervisor_node: localization", 0, "NORMAL"),
      ),
    );
    expect(regions.map((r) => r.id)).toEqual(["Drivers", "Localization"]);
    expect(regions[0].label).toBe("ドライバ");
    expect(regions[0].items).toEqual([
      { name: "driver_watchdog_node: top_lidar", level: 2, message: "no data for 2.0 s" },
    ]);
  });

  it("診断が 1 つも届いていないグループ(STALE)は、そのグループを項目にする", () => {
    const regions = summarizeHealth(
      tree(status("/Robot/Drivers"), status("/Robot/Drivers/LiDAR", 3, "Stale")),
    );
    expect(regions[0].items).toEqual([{ name: "LiDAR", level: 3, message: "Stale" }]);
  });

  it("木に無い区分は出さない", () => {
    expect(summarizeHealth(tree(status("/Robot/System")))).toHaveLength(1);
    expect(summarizeHealth(tree())).toEqual([]);
  });

  it("まだ受信していなければ空", () => {
    expect(summarizeHealth(null)).toEqual([]);
  });
});

describe("countOk", () => {
  it("level 0 だけを OK として数える", () => {
    expect(
      countOk([
        { name: "a", level: 0, message: "" },
        { name: "b", level: 1, message: "" },
        { name: "c", level: 3, message: "" },
      ]),
    ).toEqual({ ok: 1, total: 3 });
  });
});
