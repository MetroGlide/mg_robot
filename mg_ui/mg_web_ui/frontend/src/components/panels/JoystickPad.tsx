import { useRef, useEffect } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useTeleop } from "../../contexts/TeleopContext";
import { TOPICS } from "../../ros/interfaces";

interface JoystickPadProps {
  client: FoxgloveClientHandle;
  onVelChange?: (linear: number, angular: number) => void;
}

const RADIUS = 52;
const THUMB_R = 16;
const PUBLISH_INTERVAL_MS = 100;
// 停止コマンドは 1 回だけだと取りこぼされうるので、間隔をあけて繰り返す
const STOP_REPEAT_COUNT = 3;
const STOP_REPEAT_INTERVAL_MS = 50;

export default function JoystickPad({ client, onVelChange }: JoystickPadProps) {
  const { maxLinear, maxAngular } = useTeleop();
  const svgRef = useRef<SVGSVGElement>(null);
  const thumbRef = useRef<SVGCircleElement>(null);
  const velRef = useRef({ linear: 0, angular: 0 });
  const dragging = useRef(false);
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);

  // 最新の値を ref に持たせ、イベントハンドラの張り直しを避ける
  const limitsRef = useRef({ maxLinear, maxAngular });
  limitsRef.current = { maxLinear, maxAngular };
  const clientRef = useRef(client);
  clientRef.current = client;
  const onVelChangeRef = useRef(onVelChange);
  onVelChangeRef.current = onVelChange;

  const { advertise } = client;
  useEffect(
    () => advertise(TOPICS.CMD_VEL, "geometry_msgs/msg/Twist"),
    [advertise],
  );

  useEffect(() => {
    const svg = svgRef.current;
    if (!svg) return;

    const publish = (linear: number, angular: number) => {
      try {
        clientRef.current.publish(TOPICS.CMD_VEL, "geometry_msgs/msg/Twist", {
          linear: { x: linear, y: 0.0, z: 0.0 },
          angular: { x: 0.0, y: 0.0, z: angular },
        });
      } catch (e) {
        console.error("failed to publish cmd_vel", e);
      }
      onVelChangeRef.current?.(linear, angular);
    };

    const stopLoop = () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };

    const startLoop = () => {
      if (intervalRef.current) return;
      intervalRef.current = setInterval(() => {
        // 非表示のタブでもタイマーは動き続けるため、ここでも確認する
        if (document.hidden || clientRef.current.status !== "connected") {
          endDrag();
          return;
        }
        publish(velRef.current.linear, velRef.current.angular);
      }, PUBLISH_INTERVAL_MS);
    };

    // 操作を終了して、速度 0 を送る。アンマウントや非表示でも呼ばれる。
    const endDrag = () => {
      const wasActive = dragging.current || intervalRef.current !== null;
      dragging.current = false;
      stopLoop();
      velRef.current = { linear: 0, angular: 0 };
      if (thumbRef.current) {
        thumbRef.current.setAttribute("cx", String(RADIUS + THUMB_R));
        thumbRef.current.setAttribute("cy", String(RADIUS + THUMB_R));
      }
      if (!wasActive) return;
      publish(0, 0);
      for (let i = 1; i < STOP_REPEAT_COUNT; i++) {
        setTimeout(() => publish(0, 0), i * STOP_REPEAT_INTERVAL_MS);
      }
    };

    const getRelPos = (e: PointerEvent) => {
      const rect = svg.getBoundingClientRect();
      return {
        dx: e.clientX - (rect.left + rect.width / 2),
        dy: e.clientY - (rect.top + rect.height / 2),
      };
    };

    const onPointerDown = (e: PointerEvent) => {
      dragging.current = true;
      svg.setPointerCapture(e.pointerId);
      startLoop();
    };

    const onPointerMove = (e: PointerEvent) => {
      if (!dragging.current) return;
      const { dx, dy } = getRelPos(e);
      const dist = Math.min(Math.sqrt(dx * dx + dy * dy), RADIUS);
      const angle = Math.atan2(dy, dx);
      const clampedX = Math.cos(angle) * dist;
      const clampedY = Math.sin(angle) * dist;

      velRef.current = {
        linear: (-clampedY / RADIUS) * limitsRef.current.maxLinear,
        angular: (-clampedX / RADIUS) * limitsRef.current.maxAngular,
      };

      if (thumbRef.current) {
        thumbRef.current.setAttribute("cx", String(RADIUS + THUMB_R + clampedX));
        thumbRef.current.setAttribute("cy", String(RADIUS + THUMB_R + clampedY));
      }
    };

    const onVisibilityChange = () => {
      if (document.hidden) endDrag();
    };

    svg.addEventListener("pointerdown", onPointerDown);
    svg.addEventListener("pointermove", onPointerMove);
    svg.addEventListener("pointerup", endDrag);
    svg.addEventListener("pointercancel", endDrag);
    svg.addEventListener("lostpointercapture", endDrag);
    window.addEventListener("blur", endDrag);
    document.addEventListener("visibilitychange", onVisibilityChange);

    return () => {
      svg.removeEventListener("pointerdown", onPointerDown);
      svg.removeEventListener("pointermove", onPointerMove);
      svg.removeEventListener("pointerup", endDrag);
      svg.removeEventListener("pointercancel", endDrag);
      svg.removeEventListener("lostpointercapture", endDrag);
      window.removeEventListener("blur", endDrag);
      document.removeEventListener("visibilitychange", onVisibilityChange);
      endDrag();
    };
  }, []);

  // 接続が切れたら操作を止める(再接続後に古い速度で走り出さないようにする)
  useEffect(() => {
    if (client.status === "connected") return;
    dragging.current = false;
    velRef.current = { linear: 0, angular: 0 };
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    if (thumbRef.current) {
      thumbRef.current.setAttribute("cx", String(RADIUS + THUMB_R));
      thumbRef.current.setAttribute("cy", String(RADIUS + THUMB_R));
    }
  }, [client.status]);

  const size = (RADIUS + THUMB_R) * 2;

  return (
    <div className="flex flex-col items-center select-none">
      <svg
        ref={svgRef}
        width={size}
        height={size}
        className="cursor-grab active:cursor-grabbing touch-none"
        style={{ userSelect: "none" }}
      >
        <circle
          cx={RADIUS + THUMB_R}
          cy={RADIUS + THUMB_R}
          r={RADIUS}
          fill="rgba(30,30,40,0.85)"
          stroke="rgba(100,120,180,0.6)"
          strokeWidth={2}
        />
        <line
          x1={RADIUS + THUMB_R}
          y1={THUMB_R}
          x2={RADIUS + THUMB_R}
          y2={size - THUMB_R}
          stroke="rgba(255,255,255,0.1)"
          strokeWidth={1}
        />
        <line
          x1={THUMB_R}
          y1={RADIUS + THUMB_R}
          x2={size - THUMB_R}
          y2={RADIUS + THUMB_R}
          stroke="rgba(255,255,255,0.1)"
          strokeWidth={1}
        />
        <circle
          ref={thumbRef}
          cx={RADIUS + THUMB_R}
          cy={RADIUS + THUMB_R}
          r={THUMB_R}
          fill="rgba(80,120,220,0.9)"
          stroke="rgba(140,170,255,0.8)"
          strokeWidth={2}
        />
      </svg>
      <span className="text-xs text-gray-500 mt-1">drag to drive</span>
    </div>
  );
}
