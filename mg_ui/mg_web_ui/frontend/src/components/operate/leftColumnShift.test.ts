import { describe, expect, it } from "vitest";
import { leftColumnShift } from "./leftColumnShift";

const rect = (top: number, bottom: number, left: number, right: number) => ({
  top,
  bottom,
  left,
  right,
});

// 横長の KPI の行と、それより狭い操作パネル
const KPI = rect(12, 80, 12, 900);
const ACTIONS = rect(88, 600, 12, 400);

describe("leftColumnShift", () => {
  it("縦に離れていればずらさない", () => {
    expect(leftColumnShift([KPI, ACTIONS], 12, rect(700, 900, 12, 560), 8)).toBe(0);
  });

  it("間隔が gap 未満なら重なる部品の右へずらす", () => {
    expect(leftColumnShift([KPI, ACTIONS], 12, rect(605, 900, 12, 560), 8)).toBe(396);
  });

  it("重ならない横長の KPI の行の幅ではずらさない", () => {
    expect(leftColumnShift([KPI, ACTIONS], 12, rect(300, 900, 12, 560), 8)).toBe(396);
  });

  it("KPI の行とも重なれば、その右へずらす", () => {
    expect(leftColumnShift([KPI, ACTIONS], 12, rect(50, 900, 12, 560), 8)).toBe(896);
  });

  it("高さ 0 の部品や列は無視する", () => {
    expect(leftColumnShift([rect(600, 600, 12, 400)], 12, rect(300, 900, 12, 560), 8)).toBe(0);
    expect(leftColumnShift([KPI, ACTIONS], 12, rect(900, 900, 12, 12), 8)).toBe(0);
  });

  it("測れていなければずらさない", () => {
    expect(leftColumnShift([KPI, ACTIONS], 12, null, 8)).toBe(0);
  });
});
