import { RosImage } from "../../types/ros-types";

const MAX_DEPTH_MM = 5000;
const MAX_DEPTH_M = 5.0;

function toBytes(data: Uint8Array | number[]): Uint8Array {
  return data instanceof Uint8Array ? data : new Uint8Array(data);
}

/**
 * カラー画像を RGBA に変換して out に書き込む。
 * 対応しない encoding の場合は何も書かずに false を返す。
 */
export function convertColorImage(
  image: RosImage,
  out: Uint8ClampedArray,
): boolean {
  const { width, height, encoding } = image;
  const channels =
    encoding === "rgb8" || encoding === "bgr8"
      ? 3
      : encoding === "rgba8" || encoding === "bgra8"
        ? 4
        : 0;
  if (channels === 0) return false;

  const data = toBytes(image.data);
  const step = image.step > 0 ? image.step : width * channels;
  const swapRb = encoding === "bgr8" || encoding === "bgra8";
  const [ri, bi] = swapRb ? [2, 0] : [0, 2];

  for (let y = 0; y < height; y++) {
    let src = y * step;
    let dst = y * width * 4;
    for (let x = 0; x < width; x++) {
      out[dst] = data[src + ri];
      out[dst + 1] = data[src + 1];
      out[dst + 2] = data[src + bi];
      out[dst + 3] = channels === 4 ? data[src + 3] : 255;
      src += channels;
      dst += 4;
    }
  }
  return true;
}

/**
 * 深度画像を近いほど明るいグレースケールの RGBA に変換して out に書き込む。
 * 深度が 0 や無効な画素は透明にする。対応しない encoding の場合は false を返す。
 */
export function convertDepthImage(
  image: RosImage,
  out: Uint8ClampedArray,
): boolean {
  const { width, height, encoding } = image;
  const bytesPerPixel =
    encoding === "16UC1" || encoding === "mono16"
      ? 2
      : encoding === "32FC1"
        ? 4
        : 0;
  if (bytesPerPixel === 0) return false;

  const data = toBytes(image.data);
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const step = image.step > 0 ? image.step : width * bytesPerPixel;
  const littleEndian = !image.is_bigendian;
  const isFloat = bytesPerPixel === 4;

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const src = y * step + x * bytesPerPixel;
      const dst = (y * width + x) * 4;
      if (src + bytesPerPixel > data.byteLength) {
        out[dst + 3] = 0;
        continue;
      }
      const depth = isFloat
        ? view.getFloat32(src, littleEndian) / MAX_DEPTH_M
        : view.getUint16(src, littleEndian) / MAX_DEPTH_MM;
      if (!isFinite(depth) || depth <= 0) {
        out[dst + 3] = 0;
        continue;
      }
      const v = Math.max(0, Math.floor(255 * (1 - depth)));
      out[dst] = v;
      out[dst + 1] = v;
      out[dst + 2] = v;
      out[dst + 3] = 255;
    }
  }
  return true;
}
