import { describe, expect, it } from "vitest";
import { DiagnosticStatus } from "../types";
import { buildDiagnosticsTree } from "../utils/diagnosticsTree";
import { countOk, countRegion, layoutGroup, summarizeHealth } from "./healthSummary";

const status = (name: string, level = 0, message = "ok"): DiagnosticStatus => ({
  name,
  level,
  message,
  hardware_id: "",
  values: [],
});

const tree = (...statuses: DiagnosticStatus[]) =>
  buildDiagnosticsTree([status("/Robot"), ...statuses], "/Robot");

const nodesTree = (levels: number[]) =>
  tree(
    status("/Robot/Navigation"),
    status("/Robot/Navigation/Nodes"),
    ...levels.map((level, i) =>
      status(
        `/Robot/Navigation/Nodes/topic_monitor_node: node n${i}`,
        level,
        level === 0 ? "alive" : "not found",
      ),
    ),
  );

describe("summarizeHealth", () => {
  it("区分ごとに、グループと元の診断(項目)にする。表示名はタスク名", () => {
    const regions = summarizeHealth(
      tree(
        status("/Robot/Drivers", 2),
        status("/Robot/Drivers/LiDAR", 2),
        status("/Robot/Drivers/LiDAR/driver_watchdog_node: top_lidar", 2, "no data for 2.0 s"),
        status("/Robot/Localization"),
        status("/Robot/Localization/Monitor"),
        status("/Robot/Localization/Monitor/localization_supervisor_node: localization", 0, "NORMAL"),
      ),
    );
    expect(regions.map((r) => r.id)).toEqual(["Drivers", "Localization"]);
    expect(regions[0].label).toBe("ドライバ");
    expect(regions[0].level).toBe(2);
    expect(regions[0].groups).toEqual([
      {
        name: "LiDAR",
        level: 2,
        items: [
          {
            name: "driver_watchdog_node: top_lidar",
            label: "top_lidar",
            level: 2,
            message: "no data for 2.0 s",
          },
        ],
      },
    ]);
  });

  it("グループ内でタスク名が重なるときは、ノード名を表示名にする", () => {
    const [region] = summarizeHealth(
      tree(
        status("/Robot/Navigation"),
        status("/Robot/Navigation/Nav2"),
        status("/Robot/Navigation/Nav2/lifecycle_manager_a: Nav2 Health"),
        status("/Robot/Navigation/Nav2/lifecycle_manager_b: Nav2 Health"),
      ),
    );
    expect(region.groups[0].items.map((i) => i.label)).toEqual([
      "lifecycle_manager_a",
      "lifecycle_manager_b",
    ]);
  });

  it("全項目に共通する最初の単語(node など)は外し、OK の項目に繰り返すメッセージは出さない", () => {
    const [region] = summarizeHealth(nodesTree([0, 0, 2]));
    const items = region.groups[0].items;
    expect(items.map((i) => i.label)).toEqual(["n0", "n1", "n2"]);
    expect(items.map((i) => i.message)).toEqual(["", "", "not found"]);
  });

  it("区分に直接入る診断は、その診断が 1 項目のグループになる", () => {
    const [region] = summarizeHealth(
      tree(
        status("/Robot/Other"),
        status("/Robot/Other/slam_gnss_nav_bridge: adoption", 1, "not adopting GNSS"),
      ),
    );
    expect(region.groups).toHaveLength(1);
    expect(region.groups[0].name).toBe("adoption");
  });

  it("診断が 1 つも届いていないグループ(STALE)は、そのグループを項目にする", () => {
    const regions = summarizeHealth(
      tree(status("/Robot/Drivers"), status("/Robot/Drivers/LiDAR", 3, "Stale")),
    );
    expect(regions[0].groups[0].items).toEqual([
      { name: "LiDAR", label: "LiDAR", level: 3, message: "Stale" },
    ]);
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
        { name: "a", label: "a", level: 0, message: "" },
        { name: "b", label: "b", level: 1, message: "" },
        { name: "c", label: "c", level: 3, message: "" },
      ]),
    ).toEqual({ ok: 1, total: 3 });
  });

  it("区分は、グループをまたいで数える", () => {
    const [region] = summarizeHealth(nodesTree([0, 2, 0]));
    expect(countRegion(region)).toEqual({ ok: 2, total: 3 });
  });
});

describe("layoutGroup", () => {
  it("項目が 1 つなら 1 行。意味のあるメッセージだけを出す", () => {
    const [region] = summarizeHealth(
      tree(
        status("/Robot/Localization"),
        status("/Robot/Localization/Monitor"),
        status("/Robot/Localization/Monitor/amcl_watchdog_node: localization", 0, "NORMAL"),
        status("/Robot/Localization/Gate"),
        status("/Robot/Localization/Gate/amcl_gate_arbiter: gate", 0, "OK"),
      ),
    );
    const [gate, monitor] = region.groups.map((g) => layoutGroup(g, "full"));
    expect(gate).toMatchObject({ title: "Gate", summary: "", rows: [] });
    expect(monitor).toMatchObject({ title: "Monitor", summary: "NORMAL", rows: [] });
  });

  it("full は全項目を並べる", () => {
    const [region] = summarizeHealth(nodesTree([0, 2, 0]));
    const layout = layoutGroup(region.groups[0], "full");
    expect(layout.summary).toBe("2/3");
    expect(layout.rows).toHaveLength(3);
  });

  it("compact は OK の項目を件数にまとめ、異常な項目だけを並べる", () => {
    const [region] = summarizeHealth(nodesTree([0, 2, 0]));
    const layout = layoutGroup(region.groups[0], "compact");
    expect(layout.summary).toBe("2/3");
    expect(layout.rows.map((r) => r.level)).toEqual([2]);
    expect(layoutGroup(summarizeHealth(nodesTree([0, 0]))[0].groups[0], "compact").rows).toEqual([]);
  });
});
