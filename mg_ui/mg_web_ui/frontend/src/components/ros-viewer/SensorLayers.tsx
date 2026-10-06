import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useVisualization } from "../../contexts/VisualizationContext";
import { TOPICS } from "../../ros/interfaces";
import { TfBuffer } from "./hooks/useTfBuffer";
import { SceneColors } from "./sceneColors";
import CostmapLayer from "./layers/CostmapLayer";
import LaserScanLayer from "./layers/LaserScanLayer";
import PathLine from "./layers/PathLine";
import CollisionPolygons from "./layers/CollisionPolygons";
import ParticleCloud from "./layers/ParticleCloud";
import PointCloud2Layer from "./layers/PointCloud2Layer";

interface Props {
  client: FoxgloveClientHandle;
  tfBuffer: TfBuffer;
  colors: SceneColors;
}

/**
 * センサ系のレイヤー。どれを描くか(=購読するか)は、設定のレイヤー(VisualizationContext)の ON/OFF に従う。
 * センサビューと、運用ビューのセンサ表示で共有する。画像(カメラ・深度)はセンサビューだけが扱う。
 */
export default function SensorLayers({ client, tfBuffer, colors }: Props) {
  const { layers } = useVisualization();
  return (
    <>
      {layers.globalCostmap && (
        <CostmapLayer
          client={client}
          topic={TOPICS.GLOBAL_COSTMAP}
          opacity={0.3}
          tfBuffer={tfBuffer}
        />
      )}
      {layers.localCostmap && (
        <CostmapLayer
          client={client}
          topic={TOPICS.LOCAL_COSTMAP}
          opacity={0.5}
          tfBuffer={tfBuffer}
        />
      )}
      {layers.lidarTop && (
        <LaserScanLayer
          client={client}
          topic={TOPICS.SCAN_TOP}
          color={colors.lidar}
          haloColor={colors.halo}
          tfBuffer={tfBuffer}
        />
      )}
      {layers.particleCloud && (
        <ParticleCloud client={client} color={colors.particle} />
      )}
      {layers.actualPath && (
        <PathLine
          client={client}
          topic={TOPICS.ACTUAL_PATH}
          color={colors.actualPath}
          lineWidth={3}
        />
      )}
      {layers.collisionPolygons && (
        <CollisionPolygons
          client={client}
          tfBuffer={tfBuffer}
          color={colors.collision}
        />
      )}
      {layers.pointCloud && (
        <PointCloud2Layer client={client} tfBuffer={tfBuffer} />
      )}
    </>
  );
}
