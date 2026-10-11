import { describe, expect, it } from "vitest";
import type { ProgressEntry, ScenarioInfo } from "../types/scenarioTest";
import {
  collectGroups,
  collectTags,
  countProgress,
  currentEntry,
  descriptionsByName,
  elapsedSinceSec,
  failedEntries,
  filterScenarios,
  formatElapsed,
  isValidRobotAddress,
  runPhase,
  segmentsFromCounts,
  segmentsFromEntries,
} from "./scenarioTest";

function scenario(name: string, group: string, tags: string[]): ScenarioInfo {
  return { name, group, tags, description: "" };
}

const SCENARIOS = [
  scenario("a", "regression", ["smoke", "nav2_goals"]),
  scenario("b", "regression", ["full", "known_issue"]),
  scenario("c", "examples", ["example", "smoke"]),
];

function entry(status: ProgressEntry["status"]): ProgressEntry {
  return {
    name: "x",
    file: "/x.yaml",
    result_dir: "x",
    status,
    message: "",
    elapsed_sec: 0,
    attempt: 0,
  };
}

describe("filterScenarios", () => {
  it("hides known issues unless requested", () => {
    const all = { group: "", tag: "", showKnownIssues: false };
    expect(filterScenarios(SCENARIOS, all).map((s) => s.name)).toEqual([
      "a",
      "c",
    ]);
    expect(
      filterScenarios(SCENARIOS, { ...all, showKnownIssues: true }).map(
        (s) => s.name,
      ),
    ).toEqual(["a", "b", "c"]);
  });

  it("filters by group and tag", () => {
    const filter = { group: "regression", tag: "smoke", showKnownIssues: true };
    expect(filterScenarios(SCENARIOS, filter).map((s) => s.name)).toEqual([
      "a",
    ]);
  });
});

describe("collect", () => {
  it("lists groups in order and tags sorted", () => {
    expect(collectGroups(SCENARIOS)).toEqual(["regression", "examples"]);
    expect(collectTags(SCENARIOS)).toEqual([
      "example",
      "full",
      "known_issue",
      "nav2_goals",
      "smoke",
    ]);
  });
});

describe("countProgress", () => {
  it("counts finished entries by status", () => {
    const counts = countProgress([
      entry("PASSED"),
      entry("FAILED"),
      entry("ERROR"),
      entry("RUNNING"),
      entry("PENDING"),
    ]);
    expect(counts).toEqual({
      total: 5,
      done: 3,
      passed: 1,
      failed: 1,
      error: 1,
    });
  });
});

describe("formatElapsed", () => {
  it("formats seconds and minutes", () => {
    expect(formatElapsed(0)).toBe("0s");
    expect(formatElapsed(59.4)).toBe("59s");
    expect(formatElapsed(65)).toBe("1m 05s");
  });
});

function progressEntry(
  name: string,
  status: ProgressEntry["status"],
): ProgressEntry {
  return {
    name,
    file: `${name}.yaml`,
    result_dir: name,
    status,
    message: "",
    elapsed_sec: 0,
    attempt: 0,
  };
}

describe("descriptionsByName", () => {
  it("maps scenario names to descriptions", () => {
    const map = descriptionsByName([
      { name: "a", group: "g", tags: [], description: "説明 A" },
    ]);
    expect(map.get("a")).toBe("説明 A");
    expect(map.get("x")).toBeUndefined();
  });
});

describe("currentEntry / failedEntries", () => {
  const entries = [
    progressEntry("a", "PASSED"),
    progressEntry("b", "FAILED"),
    progressEntry("c", "RUNNING"),
    progressEntry("d", "ERROR"),
    progressEntry("e", "PENDING"),
  ];

  it("finds the running entry and its position", () => {
    expect(currentEntry(entries)?.index).toBe(2);
    expect(currentEntry(entries)?.entry.name).toBe("c");
    expect(currentEntry([progressEntry("a", "PASSED")])).toBeNull();
  });

  it("lists failed and errored entries", () => {
    expect(failedEntries(entries).map((e) => e.name)).toEqual(["b", "d"]);
  });
});

describe("segments", () => {
  it("follows the order of the entries", () => {
    const segments = segmentsFromEntries([
      progressEntry("a", "PASSED"),
      progressEntry("b", "RUNNING"),
    ]);
    expect(segments).toEqual([
      { status: "PASSED", label: "a" },
      { status: "RUNNING", label: "b" },
    ]);
  });

  it("builds from counts, filling the rest as pending", () => {
    const statuses = segmentsFromCounts({ PASSED: 2, FAILED: 1 }, 5).map(
      (s) => s.status,
    );
    expect(statuses).toEqual([
      "PASSED",
      "PASSED",
      "FAILED",
      "PENDING",
      "PENDING",
    ]);
  });
});

describe("elapsedSinceSec", () => {
  const now = Date.parse("2026-10-11T03:00:30+00:00");

  it("is independent of the writer's time zone", () => {
    expect(elapsedSinceSec("2026-10-11T03:00:00+00:00", now)).toBe(30);
    expect(elapsedSinceSec("2026-10-11T12:00:00+09:00", now)).toBe(30);
  });

  it("returns null for missing or unreadable values", () => {
    expect(elapsedSinceSec(undefined, now)).toBeNull();
    expect(elapsedSinceSec("", now)).toBeNull();
    expect(elapsedSinceSec("not a date", now)).toBeNull();
  });

  it("never goes negative when the clocks differ", () => {
    expect(elapsedSinceSec("2026-10-11T03:01:00+00:00", now)).toBe(0);
  });
});

describe("runPhase", () => {
  const run = {
    run_id: "r",
    started_at: "",
    options: {},
    entries: [],
    finished: false,
    exit_code: null,
  };

  it("is idle when the container is not running", () => {
    expect(runPhase(null)).toBe("idle");
    expect(
      runPhase({ services: { "scenario-test": "exited" }, run: null }),
    ).toBe("idle");
  });

  it("is starting until the new run writes its progress", () => {
    expect(
      runPhase({
        services: { "scenario-test": "running" },
        run: { ...run, state: "finished" },
      }),
    ).toBe("starting");
    expect(
      runPhase({
        services: { "scenario-test": "running" },
        run: { ...run, state: "running" },
      }),
    ).toBe("running");
  });
});

describe("isValidRobotAddress", () => {
  it("accepts empty, IPv4 and host names", () => {
    expect(isValidRobotAddress("")).toBe(true);
    expect(isValidRobotAddress("192.168.0.10")).toBe(true);
    expect(isValidRobotAddress("mg-robot.local")).toBe(true);
    expect(isValidRobotAddress("1.2.3.4;id")).toBe(false);
    expect(isValidRobotAddress(" 1.2.3.4")).toBe(false);
  });
});
