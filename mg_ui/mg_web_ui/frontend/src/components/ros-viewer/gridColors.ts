export type GridPalette = "map" | "costmap" | "overlay";

type PixelWriter = (value: number, out: Uint8ClampedArray, index: number) => void;

function setPixel(
  out: Uint8ClampedArray,
  index: number,
  r: number,
  g: number,
  b: number,
  a: number,
) {
  out[index] = r;
  out[index + 1] = g;
  out[index + 2] = b;
  out[index + 3] = a;
}

const isUnknown = (value: number) => value < 0 || value === 255;

const PALETTES: Record<GridPalette, PixelWriter> = {
  // 地図: 未知=灰, 自由=薄い灰, 占有率が高いほど暗い(すべて不透明)
  map: (value, out, i) => {
    if (isUnknown(value)) return setPixel(out, i, 127, 127, 127, 255);
    if (value === 0) return setPixel(out, i, 210, 210, 210, 255);
    const v = Math.floor((255 * (100 - value)) / 100);
    setPixel(out, i, v, v, v, 255);
  },
  // コストマップ: 0 以下は透明, 100 は赤, 中間は緑から赤へ
  costmap: (value, out, i) => {
    if (value <= 0) return setPixel(out, i, 0, 0, 0, 0);
    if (value === 100) return setPixel(out, i, 255, 0, 0, 255);
    const t = value / 100;
    setPixel(out, i, Math.floor(255 * t), Math.floor(255 * (1 - t)), 0, 255);
  },
  // 衛星画像への重ね合わせ: 未知は透明(枠を見せない), 自由=白, 占有は暗色
  overlay: (value, out, i) => {
    if (isUnknown(value)) return setPixel(out, i, 128, 128, 128, 0);
    if (value === 0) return setPixel(out, i, 255, 255, 255, 255);
    const v = Math.floor((255 * (100 - value)) / 100);
    setPixel(out, i, v, v, v, 255);
  },
};

/**
 * OccupancyGrid の data を RGBA に変換して out に書き込む。
 *
 * ROS の地図は左下が原点で、data は下の行から並ぶ。テクスチャ(DataTexture)は下の行から
 * 並ぶのでそのまま書き込み、canvas の ImageData(上の行から並ぶ)に描く場合は flipY を指定する。
 */
export function fillGridRgba(
  data: ArrayLike<number>,
  width: number,
  height: number,
  palette: GridPalette,
  out: Uint8ClampedArray,
  flipY: boolean,
): void {
  const write = PALETTES[palette];
  for (let y = 0; y < height; y++) {
    const row = flipY ? height - 1 - y : y;
    let dst = row * width * 4;
    let src = y * width;
    for (let x = 0; x < width; x++) {
      const raw = data[src++];
      write(typeof raw === "number" ? raw : 0, out, dst);
      dst += 4;
    }
  }
}
