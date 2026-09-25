import { RefObject, useEffect, useRef, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { RosImage } from "../../../types/ros-types"

export type ImageConverter = (image: RosImage, out: Uint8ClampedArray) => boolean

/**
 * sensor_msgs/Image を購読し、canvas に直接描画する。
 *
 * 受信のたびに描画せず、次の描画フレームで最新の 1 枚だけを描く(処理が追いつかない場合は
 * 古い画像を捨てる)。ImageData は画像サイズが変わらない限り使い回す。
 */
export function useImageCanvas(
  client: FoxgloveClientHandle,
  topic: string,
  convert: ImageConverter,
): { canvasRef: RefObject<HTMLCanvasElement>; hasImage: boolean } {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [hasImage, setHasImage] = useState(false)
  const { subscribe } = client

  useEffect(() => {
    let latest: RosImage | null = null
    let frame = 0
    let imageData: ImageData | null = null

    const draw = () => {
      frame = 0
      const canvas = canvasRef.current
      const image = latest
      latest = null
      if (!canvas || !image || !image.width || !image.height) return

      const ctx = canvas.getContext("2d")
      if (!ctx) return
      if (canvas.width !== image.width || canvas.height !== image.height) {
        canvas.width = image.width
        canvas.height = image.height
      }
      if (!imageData || imageData.width !== image.width || imageData.height !== image.height) {
        imageData = ctx.createImageData(image.width, image.height)
      }
      if (!convert(image, imageData.data)) return
      ctx.putImageData(imageData, 0, 0)
      setHasImage(true)
    }

    const unsubscribe = subscribe(topic, "sensor_msgs/msg/Image", (msg) => {
      latest = msg as RosImage
      if (!frame) frame = requestAnimationFrame(draw)
    })

    return () => {
      unsubscribe()
      if (frame) cancelAnimationFrame(frame)
      setHasImage(false)
    }
  }, [subscribe, topic, convert])

  return { canvasRef, hasImage }
}
