import { describe, expect, it } from "vitest";
import { evaluateFreshness } from "./useFreshness";

describe("evaluateFreshness", () => {
  it("一度も受信していなければ古い扱いにする", () => {
    expect(evaluateFreshness(null, 10_000, 3)).toEqual({
      ageSec: null,
      stale: true,
    });
  });

  it("上限以内なら新しい", () => {
    expect(evaluateFreshness(8_000, 10_000, 3)).toEqual({
      ageSec: 2,
      stale: false,
    });
  });

  it("上限を超えたら古い", () => {
    expect(evaluateFreshness(5_000, 10_000, 3)).toEqual({
      ageSec: 5,
      stale: true,
    });
  });

  it("受信時刻が未来でも経過秒は負にしない", () => {
    expect(evaluateFreshness(11_000, 10_000, 3).ageSec).toBe(0);
  });
});
