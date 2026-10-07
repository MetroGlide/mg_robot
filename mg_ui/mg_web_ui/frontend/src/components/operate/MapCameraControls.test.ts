import { describe, expect, it } from "vitest";
import * as THREE from "three";
import { clampZoom, MAX_ZOOM, MIN_ZOOM, panCameraByPixels } from "./MapCameraControls";

function makeCamera(zoom: number) {
  const cam = new THREE.OrthographicCamera(-100, 100, 100, -100, 0.1, 1000);
  cam.zoom = zoom;
  cam.position.set(0, 0, 100);
  cam.updateMatrixWorld();
  return cam;
}

describe("panCameraByPixels", () => {
  it("右へ 20px 動かすと、カメラは左へ 20/zoom だけ動く", () => {
    const cam = makeCamera(10);
    panCameraByPixels(cam, 20, 0);
    expect(cam.position.x).toBeCloseTo(-2);
    expect(cam.position.y).toBeCloseTo(0);
  });

  it("下へ 10px 動かすと、カメラは上へ 10/zoom だけ動く", () => {
    const cam = makeCamera(5);
    panCameraByPixels(cam, 0, 10);
    expect(cam.position.y).toBeCloseTo(2);
  });

  it("カメラを 90 度回していても、画面の右へ動かす", () => {
    const cam = makeCamera(10);
    cam.up.set(-1, 0, 0);
    cam.lookAt(0, 0, 0);
    cam.updateMatrixWorld();
    panCameraByPixels(cam, 10, 0);
    // 画面の右がワールドの +y を向いているので、カメラは -y 方向へ動く
    expect(cam.position.x).toBeCloseTo(0);
    expect(Math.abs(cam.position.y)).toBeCloseTo(1);
  });
});

describe("clampZoom", () => {
  it("範囲内に収める", () => {
    expect(clampZoom(0.01)).toBe(MIN_ZOOM);
    expect(clampZoom(1e6)).toBe(MAX_ZOOM);
    expect(clampZoom(10)).toBe(10);
  });
});
