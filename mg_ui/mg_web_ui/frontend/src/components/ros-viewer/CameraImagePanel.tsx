import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { TOPICS } from "../../ros/interfaces";
import { convertColorImage } from "./imageConversion";
import { useImageCanvas } from "./hooks/useImageCanvas";

interface CameraImagePanelProps {
  client: FoxgloveClientHandle;
  className?: string;
}

export default function CameraImagePanel({
  client,
  className,
}: CameraImagePanelProps) {
  const { canvasRef, hasImage } = useImageCanvas(
    client,
    TOPICS.CAMERA_IMAGE,
    convertColorImage,
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
        <span className="text-gray-500 text-xs">No camera image</span>
      )}
    </div>
  );
}
