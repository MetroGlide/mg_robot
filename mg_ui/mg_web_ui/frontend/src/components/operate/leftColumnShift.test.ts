import { describe, expect, it } from "vitest";
import { leftColumnShift } from "./leftColumnShift";

const rect = (top: number, bottom: number, left: number, right: number) => ({
  top,
  bottom,
  left,
  right,
});

describe("leftColumnShift", () => {
  it("縦に離れていればずらさない", () => {
    expect(leftColumnShift(rect(12, 300, 12, 400), rect(500, 700, 12, 560), 8)).toBe(0);
  });

  it("間隔が gap 未満なら左上の列の右へずらす", () => {
    expect(leftColumnShift(rect(12, 495, 12, 400), rect(500, 700, 12, 560), 8)).toBe(396);
  });

  it("重なっていれば左上の列の幅と gap の分ずらす", () => {
    expect(leftColumnShift(rect(12, 650, 12, 300), rect(500, 700, 12, 560), 8)).toBe(296);
  });

  it("どちらかが空 (高さ 0) ならずらさない", () => {
    expect(leftColumnShift(rect(12, 650, 12, 300), rect(700, 700, 12, 12), 8)).toBe(0);
    expect(leftColumnShift(rect(12, 12, 12, 12), rect(500, 700, 12, 560), 8)).toBe(0);
  });

  it("測れていなければずらさない", () => {
    expect(leftColumnShift(null, rect(500, 700, 12, 560), 8)).toBe(0);
  });
});
