import { describe, expect, it } from "vitest";
import { fillGridRgba } from "./gridColors";

function fill(
  data: number[],
  width: number,
  height: number,
  palette: Parameters<typeof fillGridRgba>[3],
  flipY = false,
) {
  const out = new Uint8ClampedArray(width * height * 4);
  fillGridRgba(data, width, height, palette, out, flipY);
  return Array.from(out);
}

describe("fillGridRgba", () => {
  it("colors the map palette", () => {
    expect(fill([-1, 0, 100, 50], 4, 1, "map")).toEqual([
      127, 127, 127, 255, 210, 210, 210, 255, 0, 0, 0, 255, 127, 127, 127, 255,
    ]);
  });

  it("treats 255 as unknown for unsigned data", () => {
    expect(fill([255], 1, 1, "map")).toEqual([127, 127, 127, 255]);
  });

  it("makes free and unknown cells transparent in the costmap palette", () => {
    const out = fill([-1, 0, 100, 50], 4, 1, "costmap");

    expect(out[3]).toBe(0);
    expect(out[7]).toBe(0);
    expect(out.slice(8, 12)).toEqual([255, 0, 0, 255]);
    expect(out.slice(12, 16)).toEqual([127, 127, 0, 255]);
  });

  it("makes unknown cells transparent and free cells white in the overlay palette", () => {
    expect(fill([-1, 0], 2, 1, "overlay")).toEqual([
      128, 128, 128, 0, 255, 255, 255, 255,
    ]);
  });

  it("keeps row order for textures and reverses it for canvas images", () => {
    const data = [0, 100]; // 1 列 2 行: 下の行が 0, 上の行が 100
    const texture = fill(data, 1, 2, "map", false);
    const canvas = fill(data, 1, 2, "map", true);

    expect(texture.slice(0, 4)).toEqual([210, 210, 210, 255]);
    expect(canvas.slice(0, 4)).toEqual([0, 0, 0, 255]);
    expect(canvas.slice(4, 8)).toEqual([210, 210, 210, 255]);
  });

  it("accepts signed typed arrays", () => {
    expect(fill(Array.from(new Int8Array([-1, 0])), 2, 1, "map")).toEqual([
      127, 127, 127, 255, 210, 210, 210, 255,
    ]);
  });

  it("treats non-numeric entries as free space", () => {
    const out = new Uint8ClampedArray(4);

    fillGridRgba([undefined as unknown as number], 1, 1, "map", out, false);

    expect(Array.from(out)).toEqual([210, 210, 210, 255]);
  });
});
