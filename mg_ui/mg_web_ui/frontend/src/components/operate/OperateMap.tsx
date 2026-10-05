import { ReactNode, Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useTheme } from "../../contexts/ThemeContext";
import { useTfBuffer } from "../ros-viewer/hooks/useTfBuffer";
import MapLayer from "../ros-viewer/layers/MapLayer";
import RobotArrow from "../ros-viewer/layers/RobotArrow";
import PathLine from "../ros-viewer/layers/PathLine";
import WaypointMarkers from "../ros-viewer/layers/WaypointMarkers";
import PoseArrowInteraction, {
  PoseInteractionMode,
} from "../ros-viewer/layers/PoseArrowInteraction";
import { RenderTicker } from "../ros-viewer/RosViewer";
import { TOPICS } from "../../ros/interfaces";
import {
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
  /** 地図の上に重ねる、ユースケース固有のシーン要素 */
  sceneChildren?: ReactNode;
  className?: string;
}

function OperateScene({
  client,
  command,
  follow,
  onUserPan,
  interactionMode,
  onPoseSet,
  sceneChildren,
}: Omit<Props, "className">) {
  const { resolved } = useTheme();
  const tfBuffer = useTfBuffer(client);

  return (
    <>
      <color attach="background" args={[BACKGROUND[resolved]]} />
      <MapCameraControls
        command={command}
        dragPanEnabled={interactionMode === "none"}
        onUserPan={onUserPan}
      />
      <RenderTicker intervalMs={OPERATE_RENDER_INTERVAL_MS} />
      {follow && <FollowRobot tfBuffer={tfBuffer} />}
      <ambientLight intensity={1} />
      <MapLayer
        client={client}
        palette={resolved === "light" ? "mapLight" : "map"}
      />
      <PathLine client={client} topic={TOPICS.NAV_PLAN} color="#ef4444" />
      <WaypointMarkers client={client} />
      <RobotArrow client={client} tfBuffer={tfBuffer} />
      {interactionMode !== "none" && (
        <PoseArrowInteraction mode={interactionMode} onPoseSet={onPoseSet} />
      )}
      {sceneChildren}
    </>
  );
}

export default function OperateMap({ className, ...sceneProps }: Props) {
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
          <OperateScene {...sceneProps} />
        </Suspense>
      </Canvas>
    </div>
  );
}
