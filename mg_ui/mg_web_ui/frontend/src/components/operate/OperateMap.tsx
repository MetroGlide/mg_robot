import { ReactNode, Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useTheme } from "../../contexts/ThemeContext";
import { useTfBuffer } from "../ros-viewer/hooks/useTfBuffer";
import MapLayer from "../ros-viewer/layers/MapLayer";
import RobotArrow from "../ros-viewer/layers/RobotArrow";
import PathLine from "../ros-viewer/layers/PathLine";
import WaypointPins from "../ros-viewer/layers/WaypointPins";
import SensorLayers from "../ros-viewer/SensorLayers";
import { SCENE_COLORS } from "../ros-viewer/sceneColors";
import PoseArrowInteraction, {
  PoseInteractionMode,
} from "../ros-viewer/layers/PoseArrowInteraction";
import { RenderTicker } from "../ros-viewer/RosViewer";
import { TOPICS } from "../../ros/interfaces";
import { useOpsValue } from "../../hooks/useOpsValue";
import {
  CameraView,
  DEFAULT_ZOOM,
  FollowRobot,
  MapCameraControls,
  MapCommand,
} from "./MapCameraControls";

// 運用ビューは地図と経路だけを描く軽い構成にする。TF の反映のため 10fps で描画を要求する。
// (センサビューの RosViewer は 20fps)
const OPERATE_RENDER_INTERVAL_MS = 100;

const BACKGROUND = { light: "#eef2fb", dark: "#0b111e" } as const;

interface Props {
  client: FoxgloveClientHandle;
  command: MapCommand | null;
  follow: boolean;
  onUserPan: () => void;
  interactionMode: "none" | PoseInteractionMode;
  onPoseSet: (x: number, y: number, yaw: number) => void;
  /** 計画経路とウェイポイントを描く(購読する)か。SLAM のように使わないビューでは false にして負荷を避ける */
  showNavLayers?: boolean;
  /** 地図の上に重ねる、ユースケース固有のシーン要素 */
  sceneChildren?: ReactNode;
  /** センサのレイヤー(LiDAR・コストマップなど)を描く(購読する)か。描く内容は設定のレイヤーに従う */
  showSensors?: boolean;
  className?: string;
  /** カメラを画面の切り替えをまたいで保持するためのキー(ユースケースの id など) */
  viewKey: string;
}

type SceneProps = Omit<Props, "className" | "viewKey"> & {
  initialView: CameraView | null;
  onViewSave: (view: CameraView) => void;
};

function OperateScene({
  client,
  command,
  follow,
  onUserPan,
  initialView,
  onViewSave,
  interactionMode,
  onPoseSet,
  showNavLayers = true,
  showSensors = false,
  sceneChildren,
}: SceneProps) {
  const { resolved } = useTheme();
  const colors = SCENE_COLORS[resolved];
  const tfBuffer = useTfBuffer(client);

  return (
    <>
      <color attach="background" args={[BACKGROUND[resolved]]} />
      <MapCameraControls
        command={command}
        dragPanEnabled={interactionMode === "none"}
        onUserPan={onUserPan}
        initialView={initialView}
        onViewSave={onViewSave}
      />
      <RenderTicker intervalMs={OPERATE_RENDER_INTERVAL_MS} />
      {follow && <FollowRobot tfBuffer={tfBuffer} />}
      <ambientLight intensity={1} />
      <MapLayer
        client={client}
        palette={resolved === "light" ? "mapLight" : "mapDark"}
      />
      {showNavLayers && (
        <>
          <PathLine
            client={client}
            topic={TOPICS.NAV_PLAN}
            color={colors.plan}
            lineWidth={3}
          />
          <WaypointPins client={client} colors={colors} />
        </>
      )}
      {showSensors && (
        <SensorLayers client={client} tfBuffer={tfBuffer} colors={colors} />
      )}
      <RobotArrow
        client={client}
        tfBuffer={tfBuffer}
        color={colors.robot}
        haloColor={colors.halo}
      />
      {interactionMode !== "none" && (
        <PoseArrowInteraction mode={interactionMode} onPoseSet={onPoseSet} />
      )}
      {sceneChildren}
    </>
  );
}

export default function OperateMap({
  className,
  viewKey,
  ...sceneProps
}: Props) {
  // カメラは再読み込みでは残さず、画面の切り替えの間だけ保持する
  const [view, setView] = useOpsValue<CameraView | null>(
    `${viewKey}.camera`,
    null,
    false,
  );
  return (
    <div className={className ?? "h-full w-full"}>
      <Canvas
        orthographic
        camera={{
          zoom: DEFAULT_ZOOM,
          position: [0, 0, 100],
          near: 0.1,
          far: 10000,
        }}
        gl={{ antialias: false }}
        frameloop="demand"
        dpr={1}
      >
        <Suspense fallback={null}>
          <OperateScene
            {...sceneProps}
            initialView={view}
            onViewSave={setView}
          />
        </Suspense>
      </Canvas>
    </div>
  );
}
