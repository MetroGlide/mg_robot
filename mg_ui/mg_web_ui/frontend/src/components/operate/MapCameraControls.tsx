import { useEffect, useRef } from "react";
import * as THREE from "three";
import { useFrame, useThree } from "@react-three/fiber";
import { TfBuffer } from "../ros-viewer/hooks/useTfBuffer";

/** 地図のカメラ操作の指示。n は同じ指示を続けて出せるようにする連番 */
export interface MapCommand {
  kind: "zoomIn" | "zoomOut" | "rotate" | "reset";
  n: number;
}

export const MIN_ZOOM = 1;
export const MAX_ZOOM = 200;
// 1 ピクセルが 1/zoom [m]。20 なら 0.8m のロボットが約 16px で見える
export const DEFAULT_ZOOM = 20;
const BUTTON_ZOOM_STEP = 1.25;
// ピンチ(ctrl+wheel)の感度
const PINCH_SENSITIVITY = 0.01;

export function clampZoom(zoom: number): number {
  return Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, zoom));
}

/**
 * 画面上で (dx, dy) ピクセルだけ地図を動かしたように、カメラを平行移動する。
 * カメラの向き(回転)に合わせて、画面の右と上の方向に動かす。
 */
export function panCameraByPixels(
  camera: THREE.OrthographicCamera,
  dx: number,
  dy: number,
): void {
  const right = new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 0);
  const up = new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 1);
  camera.position.addScaledVector(right, -dx / camera.zoom);
  camera.position.addScaledVector(up, dy / camera.zoom);
}

function lookDown(camera: THREE.Camera): void {
  camera.lookAt(camera.position.x, camera.position.y, 0);
  camera.updateMatrixWorld();
}

/** 地図のカメラの位置・拡大・向き。画面を切り替えても保持するために、離れるときに保存する */
export interface CameraView {
  x: number;
  y: number;
  zoom: number;
  upX: number;
  upY: number;
}

interface Props {
  command: MapCommand | null;
  /** false にすると、ドラッグでの移動を止める(姿勢・ゴールの指定中に使う) */
  dragPanEnabled: boolean;
  /** 利用者が手で動かしたときに呼ぶ。追従を切るために使う */
  onUserPan?: () => void;
  /** 最初に復元するカメラ。マウント時の値だけを使う */
  initialView?: CameraView | null;
  /** 画面を離れる(アンマウントする)ときに、最後のカメラを渡す */
  onViewSave?: (view: CameraView) => void;
}

/**
 * ノート PC のタッチパッド向けの 2D カメラ操作。
 * 2 本指のスクロールで移動、ピンチ(ctrl+wheel)でズーム、ドラッグでも移動する。
 * 回転・ズームのボタンと初期位置への復帰は command で受ける。
 */
export function MapCameraControls({
  command,
  dragPanEnabled,
  onUserPan,
  initialView,
  onViewSave,
}: Props) {
  const { camera, gl, invalidate } = useThree();
  const initialViewRef = useRef(initialView);
  const onViewSaveRef = useRef(onViewSave);
  onViewSaveRef.current = onViewSave;

  useEffect(() => {
    const view = initialViewRef.current;
    if (!view) return;
    const cam = camera as THREE.OrthographicCamera;
    cam.position.set(view.x, view.y, 100);
    cam.up.set(view.upX, view.upY, 0);
    cam.zoom = clampZoom(view.zoom);
    lookDown(cam);
    cam.updateProjectionMatrix();
    invalidate();
  }, [camera, invalidate]);

  useEffect(
    () => () => {
      const cam = camera as THREE.OrthographicCamera;
      onViewSaveRef.current?.({
        x: cam.position.x,
        y: cam.position.y,
        zoom: cam.zoom,
        upX: cam.up.x,
        upY: cam.up.y,
      });
    },
    [camera],
  );
  const onUserPanRef = useRef(onUserPan);
  onUserPanRef.current = onUserPan;
  const dragPanEnabledRef = useRef(dragPanEnabled);
  dragPanEnabledRef.current = dragPanEnabled;

  useEffect(() => {
    const el = gl.domElement;
    const cam = camera as THREE.OrthographicCamera;
    let dragging = false;
    let lastX = 0;
    let lastY = 0;

    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      if (e.ctrlKey) {
        cam.zoom = clampZoom(cam.zoom * Math.exp(-e.deltaY * PINCH_SENSITIVITY));
        cam.updateProjectionMatrix();
      } else {
        panCameraByPixels(cam, -e.deltaX, -e.deltaY);
        onUserPanRef.current?.();
      }
      invalidate();
    };
    const onDown = (e: MouseEvent) => {
      if (e.button !== 0 || !dragPanEnabledRef.current) return;
      dragging = true;
      lastX = e.clientX;
      lastY = e.clientY;
    };
    const onMove = (e: MouseEvent) => {
      if (!dragging) return;
      panCameraByPixels(cam, e.clientX - lastX, e.clientY - lastY);
      lastX = e.clientX;
      lastY = e.clientY;
      onUserPanRef.current?.();
      invalidate();
    };
    const onUp = () => {
      dragging = false;
    };

    el.addEventListener("wheel", onWheel, { passive: false });
    el.addEventListener("mousedown", onDown);
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
    return () => {
      el.removeEventListener("wheel", onWheel);
      el.removeEventListener("mousedown", onDown);
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
    };
  }, [camera, gl, invalidate]);

  useEffect(() => {
    if (!command) return;
    const cam = camera as THREE.OrthographicCamera;
    switch (command.kind) {
      case "zoomIn":
        cam.zoom = clampZoom(cam.zoom * BUTTON_ZOOM_STEP);
        break;
      case "zoomOut":
        cam.zoom = clampZoom(cam.zoom / BUTTON_ZOOM_STEP);
        break;
      case "rotate": {
        const { x, y } = cam.up;
        // 90 度ずつ回す
        cam.up.set(-y, x, 0);
        lookDown(cam);
        break;
      }
      case "reset":
        cam.position.set(0, 0, 100);
        cam.up.set(0, 1, 0);
        cam.zoom = DEFAULT_ZOOM;
        lookDown(cam);
        break;
    }
    cam.updateProjectionMatrix();
    invalidate();
    // command は連番付きなので、同じ種類の指示も続けて実行できる
  }, [command, camera, invalidate]);

  return null;
}

/** ロボット(base_link)の位置にカメラを合わせ続ける */
export function FollowRobot({ tfBuffer }: { tfBuffer: TfBuffer }) {
  const camera = useThree((state) => state.camera);
  useFrame(() => {
    const mat = tfBuffer.lookupTransform("map", "base_link");
    if (!mat) return;
    const pos = new THREE.Vector3().setFromMatrixPosition(mat);
    camera.position.x = pos.x;
    camera.position.y = pos.y;
  });
  return null;
}
