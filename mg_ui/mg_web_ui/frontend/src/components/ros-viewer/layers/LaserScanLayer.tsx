import { useMemo, useEffect } from "react";
import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { useLaserScan } from "../hooks/useLaserScan";
import { TfBuffer } from "../hooks/useTfBuffer";

// 点の大きさは画面上の px で指定する(ズームしても小さくならない)
const POINT_SIZE_PX = 3.5;
const HALO_SIZE_PX = 6;

interface LaserScanLayerProps {
  client: FoxgloveClientHandle;
  topic: string;
  color?: string;
  /** 点の縁取りの色。地図の明るさによらず点が見えるようにする */
  haloColor?: string;
  tfBuffer: TfBuffer;
}

export default function LaserScanLayer({
  client,
  topic,
  color = "#ffffff",
  haloColor = "#000000",
  tfBuffer,
}: LaserScanLayerProps) {
  const scan = useLaserScan(client, topic);
  const geometry = useMemo(() => new THREE.BufferGeometry(), []);

  // Transform scan points to map frame whenever a new scan arrives.
  // tfBuffer.lookupTransform reads from a ref, so always returns the latest TF.
  const positions = useMemo(() => {
    if (!scan) return null;

    const { ranges, angle_min, angle_increment, range_min, range_max, header } =
      scan;
    const tf = tfBuffer.lookupTransform("map", header.frame_id);

    const local: number[] = [];
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (!isFinite(r) || r < range_min || r > range_max) continue;
      const angle = angle_min + i * angle_increment;
      local.push(r * Math.cos(angle), r * Math.sin(angle), 0);
    }

    if (!tf) return new Float32Array(local);

    const v = new THREE.Vector3();
    const out: number[] = [];
    for (let i = 0; i < local.length; i += 3) {
      v.set(local[i], local[i + 1], local[i + 2]).applyMatrix4(tf);
      out.push(v.x, v.y, v.z);
    }
    return new Float32Array(out);
  }, [scan, tfBuffer]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (!positions) return;
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setDrawRange(0, positions.length / 3);
    geometry.computeBoundingSphere();
  }, [geometry, positions]);

  useEffect(() => () => geometry.dispose(), [geometry]);

  if (!positions || positions.length === 0) return null;

  return (
    <>
      <points geometry={geometry} renderOrder={1}>
        <pointsMaterial
          color={haloColor}
          size={HALO_SIZE_PX}
          sizeAttenuation={false}
          depthTest={false}
        />
      </points>
      <points geometry={geometry} renderOrder={2}>
        <pointsMaterial
          color={color}
          size={POINT_SIZE_PX}
          sizeAttenuation={false}
          depthTest={false}
        />
      </points>
    </>
  );
}
