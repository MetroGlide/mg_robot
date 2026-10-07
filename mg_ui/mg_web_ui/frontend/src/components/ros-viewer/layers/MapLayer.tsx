import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { useOccupancyGrid } from "../hooks/useOccupancyGrid";
import { useGridTexture } from "../hooks/useGridTexture";
import { GridPalette } from "../gridColors";

export default function MapLayer({
  client,
  topic,
  palette = "map",
}: {
  client: FoxgloveClientHandle;
  topic?: string;
  palette?: GridPalette;
}) {
  const grid = useOccupancyGrid(client, topic);
  const texture = useGridTexture(grid, palette);

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
