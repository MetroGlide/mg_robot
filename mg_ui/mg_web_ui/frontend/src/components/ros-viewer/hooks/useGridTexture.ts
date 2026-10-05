import { useEffect, useMemo, useRef } from "react"
import * as THREE from "three"
import { OccupancyGrid } from "../../../types/ros-types"
import { fillGridRgba, GridPalette } from "../gridColors"

/**
 * OccupancyGrid からテクスチャを作る。
 *
 * グリッドのサイズが変わらない限り、同じ DataTexture とバッファを使い回して中身だけ更新する
 * (更新のたびにテクスチャを作り直して GPU メモリを確保・解放しない)。
 */
export function useGridTexture(
  grid: OccupancyGrid | null,
  palette: GridPalette,
): THREE.DataTexture | null {
  const textureRef = useRef<THREE.DataTexture | null>(null)

  const texture = useMemo(() => {
    if (!grid) return null
    const { width, height } = grid.info
    let current = textureRef.current
    if (!current || current.image.width !== width || current.image.height !== height) {
      current?.dispose()
      current = new THREE.DataTexture(
        new Uint8Array(width * height * 4),
        width,
        height,
        THREE.RGBAFormat,
      )
      current.minFilter = THREE.NearestFilter
      current.magFilter = THREE.NearestFilter
      textureRef.current = current
    }
    const pixels = current.image.data as unknown as Uint8Array
    fillGridRgba(
      grid.data,
      width,
      height,
      palette,
      new Uint8ClampedArray(pixels.buffer, pixels.byteOffset, pixels.byteLength),
      false,
    )
    current.needsUpdate = true
    return current
  }, [grid, palette])

  useEffect(
    () => () => {
      textureRef.current?.dispose()
      textureRef.current = null
    },
    [],
  )

  return texture
}
