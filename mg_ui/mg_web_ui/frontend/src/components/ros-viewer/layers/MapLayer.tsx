import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { useOccupancyGrid } from "../hooks/useOccupancyGrid";
import { useGridTexture } from "../hooks/useGridTexture";

export default function MapLayer({
  client,
  topic,
}: {
  client: FoxgloveClientHandle;
  topic?: string;
}) {
  const grid = useOccupancyGrid(client, topic);
  const texture = useGridTexture(grid, "map");

  if (!grid || !texture) return null;

  const { resolution, width, height, origin } = grid.info;
  const w = width * resolution;
  const h = height * resolution;

  return (
    <mesh position={[origin.position.x + w / 2, origin.position.y + h / 2, -0.01]}>
      <planeGeometry args={[w, h]} />
      <meshBasicMaterial map={texture} depthWrite={false} />
    </mesh>
  );
}
