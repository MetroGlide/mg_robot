import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { TOPICS } from "../../ros/interfaces";
import { convertDepthImage } from "./imageConversion";
import { useImageCanvas } from "./hooks/useImageCanvas";

interface DepthImagePanelProps {
  client: FoxgloveClientHandle;
  className?: string;
}

export default function DepthImagePanel({
  client,
  className,
}: DepthImagePanelProps) {
  const { canvasRef, hasImage } = useImageCanvas(
    client,
    TOPICS.DEPTH_IMAGE,
    convertDepthImage,
  );

  return (
    <div
      className={`flex items-center justify-center bg-black overflow-hidden ${className ?? "w-full h-full"}`}
    >
      <canvas
        ref={canvasRef}
        className={`max-w-full max-h-full object-contain ${hasImage ? "" : "hidden"}`}
      />
      {!hasImage && (
        <span className="text-gray-500 text-xs">No depth image</span>
      )}
    </div>
  );
}
