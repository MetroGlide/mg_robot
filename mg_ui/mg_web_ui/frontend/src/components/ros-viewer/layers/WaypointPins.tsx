import { useMemo, useRef } from "react";
import * as THREE from "three";
import { Text } from "@react-three/drei";
import { useFrame } from "@react-three/fiber";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { useThrottledTopic } from "../../../hooks/useThrottledTopic";
import { SequencerStatus } from "../../../types";
import { TOPICS } from "../../../ros/interfaces";
import { useMarkerArray } from "../hooks/useMarkerArray";
import { SceneColors } from "../sceneColors";
import { PinState, pinState, waypointPinsFromMarkers } from "./waypointPins";

// ピンの半径(画面上の px)。ズームしても大きさを変えない
const PIN_RADIUS_PX = 12;
// いまの目標は、同じ形を少し大きくして縁取りを足す
const CURRENT_SCALE = 1.35;
const STATUS_HZ = 2;
// 走行していない状態。これ以外のときだけ通過済み・目標を区別する
const IDLE_STATE = "IDLE";

const OPACITY: Record<PinState, number> = {
  past: 0.4,
  current: 1,
  upcoming: 1,
};

// 向きを示す三角形(円の半径を 1 とした座標)
const TRIANGLE = new Float32Array([1.15, 0.6, 0, 1.15, -0.6, 0, 2.1, 0, 0]);
const TRIANGLE_HALO = new Float32Array([1.0, 0.8, 0, 1.0, -0.8, 0, 2.4, 0, 0]);

function Triangle({
  positions,
  color,
  opacity,
  order,
}: {
  positions: Float32Array;
  color: string;
  opacity: number;
  order: number;
}) {
  return (
    <mesh renderOrder={order}>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
      </bufferGeometry>
      <meshBasicMaterial
        color={color}
        transparent
        opacity={opacity}
        depthTest={false}
        depthWrite={false}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}

function Disc({
  radius,
  color,
  opacity,
  order,
}: {
  radius: number;
  color: string;
  opacity: number;
  order: number;
}) {
  return (
    <mesh renderOrder={order}>
      <circleGeometry args={[radius, 28]} />
      <meshBasicMaterial
        color={color}
        transparent
        opacity={opacity}
        depthTest={false}
        depthWrite={false}
      />
    </mesh>
  );
}

function Pin({
  x,
  y,
  yaw,
  label,
  fill,
  state,
  colors,
}: {
  x: number;
  y: number;
  yaw: number;
  label: string;
  fill: string;
  state: PinState;
  colors: SceneColors;
}) {
  const scaleRef = useRef<THREE.Group>(null);
  const emphasis = state === "current" ? CURRENT_SCALE : 1;
  const opacity = OPACITY[state];

  // 正投影のカメラでは、1px が 1/zoom [m]。毎フレーム合わせて、画面上の大きさを一定にする
  useFrame(({ camera }) => {
    if (!scaleRef.current) return;
    const s = (PIN_RADIUS_PX * emphasis) / camera.zoom;
    scaleRef.current.scale.set(s, s, 1);
  });

  return (
    <group position={[x, y, 0.3]}>
      <group ref={scaleRef}>
        <group rotation={[0, 0, yaw]}>
          <Triangle positions={TRIANGLE_HALO} color={colors.halo} opacity={opacity} order={10} />
          <Triangle positions={TRIANGLE} color={fill} opacity={opacity} order={11} />
        </group>
        <Disc radius={1.2} color={colors.halo} opacity={opacity} order={12} />
        <Disc radius={1} color={fill} opacity={opacity} order={13} />
        <Text
          fontSize={1.15}
          color={colors.waypointLabel}
          anchorX="center"
          anchorY="middle"
          renderOrder={14}
          fillOpacity={opacity}
        >
          {label}
        </Text>
      </group>
    </group>
  );
}

/**
 * 運用ビューのウェイポイント。番号入りの円と向きの三角形を、ズームによらず一定の大きさで描く。
 * 止まる点と通過する点は色で分け、走行中は通過済みを薄く、いまの目標を大きく描く。
 * 購読するのはウェイポイントのマーカーとシーケンサの状態(2Hz に間引く)だけ。
 */
export default function WaypointPins({
  client,
  colors,
}: {
  client: FoxgloveClientHandle;
  colors: SceneColors;
}) {
  const markerArray = useMarkerArray(client, TOPICS.WAYPOINT_MARKERS);
  const status = useThrottledTopic<SequencerStatus>(
    client,
    TOPICS.WAYPOINT_STATUS,
    "mg_msgs/msg/SequencerStatus",
    STATUS_HZ,
  );

  const pins = useMemo(
    () => (markerArray ? waypointPinsFromMarkers(markerArray.markers) : []),
    [markerArray],
  );
  const running = status !== null && status.state !== IDLE_STATE;
  const currentIndex = status ? status.current_index : null;

  return (
    <>
      {pins.map((pin) => (
        <Pin
          key={pin.index}
          x={pin.x}
          y={pin.y}
          yaw={pin.yaw}
          label={String(pin.index)}
          fill={pin.through ? colors.waypointThrough : colors.waypointStop}
          state={pinState(pin.index, currentIndex, running)}
          colors={colors}
        />
      ))}
    </>
  );
}
