import { describe, expect, it } from "vitest";
import { DiagnosticStatus } from "../types";
import { buildDiagnosticsTree, findNode, leaves, worstLevel } from "./diagnosticsTree";

const status = (name: string, level = 0, message = ""): DiagnosticStatus => ({
  name,
  level,
  message,
  hardware_id: "",
  values: [],
});

const STATUSES = [
  status("/Robot", 2, "Error"),
  status("/Robot/Drivers", 2, "Error"),
  status("/Robot/Drivers/LiDAR", 2, "Error"),
  status("/Robot/Drivers/LiDAR/driver_watchdog_node: top_lidar", 2, "no data for 2.0 s"),
  status("/Robot/Drivers/GNSS", 0, "OK"),
  status("/Robot/Drivers/GNSS/ublox_gps_node: fix", 0, "3D fix"),
  status("/Robot/System", 0, "OK"),
];

describe("buildDiagnosticsTree", () => {
  it("パスから木を作り、子を名前順に並べる", () => {
    const root = buildDiagnosticsTree(STATUSES, "/Robot");
    expect(root?.children.map((c) => c.name)).toEqual(["Drivers", "System"]);
    const drivers = findNode(root, "/Robot/Drivers");
    expect(drivers?.children.map((c) => c.name)).toEqual(["GNSS", "LiDAR"]);
    expect(drivers?.isLeaf).toBe(false);
  });

  it("元の診断(': ' を含む名前)を葉にする", () => {
    const root = buildDiagnosticsTree(STATUSES, "/Robot");
    const leaf = findNode(root, "/Robot/Drivers/LiDAR/driver_watchdog_node: top_lidar");
    expect(leaf?.isLeaf).toBe(true);
    expect(leaf?.name).toBe("driver_watchdog_node: top_lidar");
    expect(leaf?.message).toBe("no data for 2.0 s");
  });

  it("根がなければ null", () => {
    expect(buildDiagnosticsTree([status("/Other/x")], "/Robot")).toBeNull();
    expect(buildDiagnosticsTree([], "/Robot")).toBeNull();
  });

  it("根の外の診断は含めない", () => {
    const root = buildDiagnosticsTree([status("/Robot"), status("/RobotX/a")], "/Robot");
    expect(root?.children).toEqual([]);
  });

  it("集約が出さなかった途中のパスは、子の最も重いレベルで補う", () => {
    const root = buildDiagnosticsTree(
      [status("/Robot"), status("/Robot/Nav/a: x", 1), status("/Robot/Nav/b: y", 3)],
      "/Robot",
    );
    const nav = findNode(root, "/Robot/Nav");
    expect(nav?.level).toBe(3);
    expect(nav?.children).toHaveLength(2);
  });
});

describe("leaves", () => {
  it("葉だけを集める", () => {
    const root = buildDiagnosticsTree(STATUSES, "/Robot");
    expect(leaves(root!).map((l) => l.name)).toEqual([
      "ublox_gps_node: fix",
      "driver_watchdog_node: top_lidar",
    ]);
  });
});

describe("worstLevel", () => {
  it("STALE は ERROR より重い", () => {
    expect(worstLevel([0, 1, 2])).toBe(2);
    expect(worstLevel([0, 3, 2])).toBe(3);
    expect(worstLevel([])).toBe(0);
  });
});
