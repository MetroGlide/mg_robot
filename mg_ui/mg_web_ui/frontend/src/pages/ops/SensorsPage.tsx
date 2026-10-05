import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useVisualization } from "../../contexts/VisualizationContext";
import {
  CameraImagePanel,
  DepthImagePanel,
  RosViewer,
} from "../../components/ros-viewer";
import LayerPanel from "../../components/sensors/LayerPanel";

/**
 * RViz ライクなセンサビュー。地図・センサ・点群・画像を重ねて確認する。
 * 負荷の大きい表示はここに集め、運用ビューには持ち込まない。
 */
export default function SensorsPage({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const { layers } = useVisualization();

  return (
    <div className="relative h-full w-full">
      <RosViewer client={client} initialMode="2d" className="h-full w-full" />
      <div className="pointer-events-none absolute inset-0">
        <div className="pointer-events-auto absolute left-3 top-3 max-h-[calc(100%-1.5rem)] overflow-y-auto">
          <LayerPanel />
        </div>
        <div className="pointer-events-auto absolute bottom-3 right-3 flex gap-2">
          {layers.colorImage && (
            <div className="h-36 w-56 overflow-hidden rounded-xl border border-line shadow-card">
              <CameraImagePanel client={client} className="h-full w-full" />
            </div>
          )}
          {layers.depthImage && (
            <div className="h-36 w-56 overflow-hidden rounded-xl border border-line shadow-card">
              <DepthImagePanel client={client} className="h-full w-full" />
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
