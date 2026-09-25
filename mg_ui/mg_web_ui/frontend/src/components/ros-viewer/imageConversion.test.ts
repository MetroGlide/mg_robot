import { describe, expect, it } from "vitest";
import { RosImage } from "../../types/ros-types";
import { convertColorImage, convertDepthImage } from "./imageConversion";

function image(overrides: Partial<RosImage>): RosImage {
  return {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "cam" },
    height: 1,
    width: 2,
    encoding: "rgb8",
    is_bigendian: false,
    step: 0,
    data: new Uint8Array(),
    ...overrides,
  } as RosImage;
}

function rgba(width: number, height: number) {
  return new Uint8ClampedArray(width * height * 4);
}

describe("convertColorImage", () => {
  it("converts rgb8", () => {
    const out = rgba(2, 1);

    const ok = convertColorImage(
      image({ data: new Uint8Array([1, 2, 3, 4, 5, 6]) }),
      out,
    );

    expect(ok).toBe(true);
    expect(Array.from(out)).toEqual([1, 2, 3, 255, 4, 5, 6, 255]);
  });

  it("swaps channels for bgr8", () => {
    const out = rgba(2, 1);

    convertColorImage(
      image({ encoding: "bgr8", data: new Uint8Array([1, 2, 3, 4, 5, 6]) }),
      out,
    );

    expect(Array.from(out)).toEqual([3, 2, 1, 255, 6, 5, 4, 255]);
  });

  it("keeps alpha for rgba8 and swaps for bgra8", () => {
    const rgbaOut = rgba(1, 1);
    const bgraOut = rgba(1, 1);

    convertColorImage(
      image({ width: 1, encoding: "rgba8", data: new Uint8Array([1, 2, 3, 4]) }),
      rgbaOut,
    );
    convertColorImage(
      image({ width: 1, encoding: "bgra8", data: new Uint8Array([1, 2, 3, 4]) }),
      bgraOut,
    );

    expect(Array.from(rgbaOut)).toEqual([1, 2, 3, 4]);
    expect(Array.from(bgraOut)).toEqual([3, 2, 1, 4]);
  });

  it("honors the row stride (step) when rows are padded", () => {
    const out = rgba(1, 2);

    convertColorImage(
      image({
        width: 1,
        height: 2,
        step: 4,
        data: new Uint8Array([1, 2, 3, 99, 4, 5, 6, 99]),
      }),
      out,
    );

    expect(Array.from(out)).toEqual([1, 2, 3, 255, 4, 5, 6, 255]);
  });

  it("accepts a plain number array as data", () => {
    const out = rgba(1, 1);

    convertColorImage(image({ width: 1, data: [7, 8, 9] }), out);

    expect(Array.from(out)).toEqual([7, 8, 9, 255]);
  });

  it("returns false and leaves the output untouched for unsupported encodings", () => {
    const out = rgba(2, 1);

    const ok = convertColorImage(
      image({ encoding: "yuv422", data: new Uint8Array(4) }),
      out,
    );

    expect(ok).toBe(false);
    expect(Array.from(out)).toEqual(new Array(8).fill(0));
  });
});

describe("convertDepthImage", () => {
  it("maps 16UC1 millimeters to brightness and makes zero transparent", () => {
    const data = new Uint8Array(new Uint16Array([0, 2500, 5000]).buffer);
    const out = rgba(3, 1);

    const ok = convertDepthImage(
      image({ width: 3, encoding: "16UC1", data }),
      out,
    );

    expect(ok).toBe(true);
    expect(out[3]).toBe(0);
    expect(Array.from(out.slice(4, 8))).toEqual([127, 127, 127, 255]);
    expect(Array.from(out.slice(8, 12))).toEqual([0, 0, 0, 255]);
  });

  it("maps 32FC1 meters and hides invalid values", () => {
    const data = new Uint8Array(
      new Float32Array([NaN, 2.5, -1, Infinity]).buffer,
    );
    const out = rgba(4, 1);

    convertDepthImage(image({ width: 4, encoding: "32FC1", data }), out);

    expect(out[3]).toBe(0);
    expect(Array.from(out.slice(4, 8))).toEqual([127, 127, 127, 255]);
    expect(out[11]).toBe(0);
    expect(out[15]).toBe(0);
  });

  it("reads big-endian 16-bit data", () => {
    const out = rgba(1, 1);

    convertDepthImage(
      image({
        width: 1,
        encoding: "mono16",
        is_bigendian: true,
        data: new Uint8Array([0x13, 0x88]),
      }),
      out,
    );

    expect(Array.from(out)).toEqual([0, 0, 0, 255]);
  });

  it("handles data that is a view into a larger buffer", () => {
    const backing = new Uint8Array([9, 9, 0xc4, 0x09, 9]);
    const data = backing.subarray(2, 4);
    const out = rgba(1, 1);

    convertDepthImage(image({ width: 1, encoding: "16UC1", data }), out);

    expect(Array.from(out)).toEqual([127, 127, 127, 255]);
  });

  it("treats truncated data as transparent instead of throwing", () => {
    const out = rgba(2, 1);

    const ok = convertDepthImage(
      image({ encoding: "16UC1", data: new Uint8Array([1, 0]) }),
      out,
    );

    expect(ok).toBe(true);
    expect(out[7]).toBe(0);
  });

  it("returns false for unsupported encodings", () => {
    expect(
      convertDepthImage(image({ encoding: "rgb8" }), rgba(2, 1)),
    ).toBe(false);
  });
});
