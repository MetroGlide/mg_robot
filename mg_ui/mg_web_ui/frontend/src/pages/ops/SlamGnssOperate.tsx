import { useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useOpsValue } from "../../hooks/useOpsValue";
import { usePoseGraph } from "../../hooks/usePoseGraph";
import { useSlamGnssMaps } from "../../hooks/useSlamGnssMaps";
import { TOPICS } from "../../ros/topics";
import Card from "../../components/ui/Card";
import OperateLayout from "../../components/operate/OperateLayout";
import OperateMap from "../../components/operate/OperateMap";
import KpiRow from "../../components/operate/KpiRow";
import MapToolbar from "../../components/operate/MapToolbar";
import HealthTabsCard from "../../components/operate/HealthTabsCard";
import SlamGnssMapCard from "../../components/operate/SlamGnssMapCard";
import SlamGnssViewCard, {
  DEFAULT_SATELLITE,
  DEFAULT_SLAM_GNSS_LAYERS,
} from "../../components/operate/SlamGnssViewCard";
import { MapCommand } from "../../components/operate/MapCameraControls";
import { PoseGraphDetailPanel } from "../../components/panels/PoseGraphDetailPanel";
import MapLayer from "../../components/ros-viewer/layers/MapLayer";
import PathLine from "../../components/ros-viewer/layers/PathLine";
import { PoseGraphLayer } from "../../components/ros-viewer/layers/PoseGraphLayer";
import SatelliteOverlayViewer from "../../components/ros-viewer/SatelliteOverlayViewer";

const VIEW_KEY = "slamGnss";

/**
 * SLAM-GNSS-2D の運用ビュー。ポーズグラフと最適化前後の経路を見ながら、地図の保存・プレビュー・再最適化を行う。
 * 衛星画像への重ね表示を使っている間は、地図を衛星画像のビューに置き換える(カメラの操作は、そのビュー自身が持つ)。
 */
export default function SlamGnssOperate({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [command, setCommand] = useState<MapCommand | null>(null);
  const [follow, setFollow] = useOpsValue<boolean>(`${VIEW_KEY}.follow`, true);
  const [layers, setLayers] = useOpsValue(`${VIEW_KEY}.layers`, DEFAULT_SLAM_GNSS_LAYERS);
  // 衛星画像はタイルを読み込むので、再読み込みでは保持しない
  const [satellite, setSatellite] = useOpsValue(`${VIEW_KEY}.satellite`, DEFAULT_SATELLITE, false);
  const [selectedNodeIndex, setSelectedNodeIndex] = useState<number | null>(null);
  const { state: poseGraphState } = usePoseGraph(client);
  const maps = useSlamGnssMaps(sysManager);
  const connected = client.status === "connected";

  const issueCommand = (kind: MapCommand["kind"]) =>
    setCommand((prev) => ({ kind, n: (prev?.n ?? 0) + 1 }));

  const map = satellite.enabled ? (
    <SatelliteOverlayViewer
      client={client}
      tileType={satellite.tileType}
      slamOpacity={satellite.slamOpacity}
    />
  ) : (
    <OperateMap
      viewKey={VIEW_KEY}
      client={client}
      command={command}
      follow={follow}
      onUserPan={() => setFollow(false)}
      interactionMode="none"
      onPoseSet={() => {}}
      showNavLayers={false}
      sceneChildren={
        <>
          <MapLayer client={client} topic={TOPICS.SLAM_GNSS2D_MAP} />
          <PathLine
            client={client}
            topic={TOPICS.SLAM_GNSS2D_PATH}
            color="#00bcd4"
            lineWidth={2}
          />
          {layers.pathBefore && (
            <PathLine
              client={client}
              topic={TOPICS.SLAM_GNSS2D_PATH_BEFORE}
              color="#888888"
              lineWidth={1}
            />
          )}
          <PoseGraphLayer
            state={poseGraphState}
            showNodes={layers.poseGraphNodes}
            showSeqEdges={layers.poseGraphSeqEdges}
            showLoopEdges={layers.poseGraphLoopEdges}
            showGnssPriors={layers.poseGraphGnssPrior}
            onNodeClick={setSelectedNodeIndex}
          />
        </>
      }
    />
  );

  return (
    <OperateLayout
      map={map}
      topLeft={
        <div className="flex flex-col items-start gap-2">
          <KpiRow client={client} />
          <SlamGnssViewCard
            layers={layers}
            onLayersChange={setLayers}
            satellite={satellite}
            onSatelliteChange={setSatellite}
          />
        </div>
      }
      toolbar={
        satellite.enabled ? undefined : (
          <MapToolbar
            onCommand={issueCommand}
            follow={follow}
            onToggleFollow={() => setFollow(!follow)}
            disabled={!connected}
          />
        )
      }
      topRight={
        layers.poseGraphNodes && !satellite.enabled ? (
          <PoseGraphDetailPanel
            state={poseGraphState}
            selectedNodeIndex={selectedNodeIndex}
            onClose={() => setSelectedNodeIndex(null)}
            className=""
          />
        ) : undefined
      }
      notice={
        maps.notice && (
          <Card className="flex items-center gap-2 px-3 py-1.5">
            <p className={`text-xs ${maps.notice.ok ? "text-ok" : "text-error"}`}>
              {maps.notice.text}
            </p>
            <button
              type="button"
              onClick={maps.clearNotice}
              className="text-xs text-muted hover:text-content"
              aria-label="閉じる"
            >
              ✕
            </button>
          </Card>
        )
      }
      bottomLeft={
        <SlamGnssMapCard sysManager={sysManager} maps={maps} />
      }
      bottomRight={<HealthTabsCard client={client} />}
    />
  );
}
