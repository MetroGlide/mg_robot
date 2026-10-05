import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useGpsFix } from "../../hooks/useGpsFix";
import { useVisualization } from "../../contexts/VisualizationContext";
import GpsMapOverlay from "../panels/GpsMapOverlay";

/**
 * 衛星画像の上に GNSS の位置と軌跡を描くミニ地図。
 * 表示している間だけ GNSS を購読し、地図タイルを読み込む(表示の切り替えは呼び出し側)。
 */
export default function GpsMiniMap({ client }: { client: FoxgloveClientHandle }) {
  const { gpsMapSize } = useVisualization();
  const { fix, trail } = useGpsFix(client);
  return (
    <GpsMapOverlay
      fix={fix}
      trail={trail}
      mapWidth={gpsMapSize === "default" ? 192 : 384}
      mapHeight={gpsMapSize === "2x-square" ? 384 : 192}
    />
  );
}
