import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { useOccupancyGrid } from "../hooks/useOccupancyGrid";
import { useGridTexture } from "../hooks/useGridTexture";
import { TfBuffer } from "../hooks/useTfBuffer";

interface CostmapLayerProps {
  client: FoxgloveClientHandle;
  topic: string;
  opacity?: number;
  tfBuffer?: TfBuffer;
}

export default function CostmapLayer({
  client,
  topic,
  opacity = 0.5,
  tfBuffer,
}: CostmapLayerProps) {
  const grid = useOccupancyGrid(client, topic);
  const texture = useGridTexture(grid, "costmap");

  if (!grid || !texture) return null;

  const { resolution, width, height, origin } = grid.info;
  const w = width * resolution;
  const h = height * resolution;
  let posX = origin.position.x + w / 2;
  let posY = origin.position.y + h / 2;
  let rotZ = 0;

  if (tfBuffer && grid.header.frame_id !== "map") {
    const tf = tfBuffer.lookupTransform("map", grid.header.frame_id);
    if (tf) {
      const v = new THREE.Vector3(posX, posY, 0).applyMatrix4(tf);
      posX = v.x;
      posY = v.y;
      rotZ = new THREE.Euler().setFromRotationMatrix(tf, "XYZ").z;
    }
  }

  return (
    <mesh position={[posX, posY, 0.01]} rotation={[0, 0, rotZ]}>
      <planeGeometry args={[w, h]} />
      <meshBasicMaterial
        map={texture}
        transparent
        opacity={opacity}
        depthWrite={false}
      />
    </mesh>
  );
}
