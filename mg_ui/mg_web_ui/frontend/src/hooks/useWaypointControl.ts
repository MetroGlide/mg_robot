import { useState } from "react";
import { FoxgloveClientHandle } from "./useFoxgloveClient";
import { useServiceCaller } from "./useServiceCaller";
import { useActionRunner } from "./useActionRunner";
import { TOPICS, SERVICES } from "../ros/interfaces";
import {
  describeServiceResponse,
  GoalBtMode,
} from "../utils/waypointActions";

export interface PoseInput {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

export type MapInteractionMode = "none" | "pose_estimate" | "nav_goal";

function stampNow() {
  const nowMs = Date.now();
  return {
    sec: Math.floor(nowMs / 1000),
    nanosec: Math.floor((nowMs % 1000) * 1_000_000),
  };
}

export function buildInitialPoseMessage(pose: PoseInput) {
  const qz = Math.sin(pose.yaw / 2.0);
  const qw = Math.cos(pose.yaw / 2.0);
  const covariance = Array(36).fill(0.0);
  covariance[0] = 0.25;
  covariance[7] = 0.25;
  covariance[35] = 0.06853891945200942;
  return {
    header: { stamp: stampNow(), frame_id: "map" },
    pose: {
      pose: {
        position: { x: pose.x, y: pose.y, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: qz, w: qw },
      },
      covariance,
    },
  };
}

export function buildNavGoalMessage(pose: PoseInput) {
  const qz = Math.sin(pose.yaw / 2.0);
  const qw = Math.cos(pose.yaw / 2.0);
  return {
    header: { stamp: stampNow(), frame_id: "map" },
    pose: {
      position: { x: pose.x, y: pose.y, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: qz, w: qw },
    },
  };
}

/**
 * waypoint_sequencer の操作(開始・停止・一時停止・再開・番号指定・再読込)と、
 * 地図上での初期姿勢・ゴールの指定をまとめる。旧ページと新 UI の両方で使う。
 *
 * onError は地図操作の失敗を呼び出し側のエラー表示に渡す。操作の開始時に null で呼んで前回のエラーを消す。
 * initialCountdownMs は、出発までの待ち時間の初期値(画面を切り替えても値を保持したい呼び出し側が渡す)。
 */
export function useWaypointControl(
  client: FoxgloveClientHandle,
  goalBt: GoalBtMode,
  onError: (message: string | null) => void,
  initialCountdownMs = 3000,
) {
  const [countdownMs, setCountdownMs] = useState(initialCountdownMs);
  const [jumpIndex, setJumpIndex] = useState(0);
  const [interactionMode, setInteractionMode] =
    useState<MapInteractionMode>("none");
  const goalRunner = useActionRunner();
  const { call, loading, error } = useServiceCaller(client);

  const start = () =>
    call(SERVICES.WAYPOINT_START, { countdown_ms: countdownMs });
  const startImmediate = () => call(SERVICES.WAYPOINT_START, { countdown_ms: 0 });
  const stop = () => call(SERVICES.WAYPOINT_STOP, {});
  const pause = () =>
    client.publish(TOPICS.WAYPOINT_PAUSE_REQUEST, "mg_msgs/msg/PauseRequest", {
      requester_id: "web_ui",
      active: true,
      reason: "manual pause",
    });
  const resume = () =>
    client.publish(TOPICS.WAYPOINT_PAUSE_REQUEST, "mg_msgs/msg/PauseRequest", {
      requester_id: "web_ui",
      active: false,
      reason: "",
    });
  const jump = () =>
    client.publish(TOPICS.WAYPOINT_SET_NEXT_INDEX, "std_msgs/msg/Int16", {
      data: jumpIndex,
    });
  const reload = () => call(SERVICES.WAYPOINT_RELOAD, {});

  const publishInitialPose = (pose: PoseInput) => {
    onError(null);
    try {
      client.publish(
        TOPICS.INITIALPOSE,
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        buildInitialPoseMessage(pose),
      );
    } catch (e) {
      onError(e instanceof Error ? e.message : String(e));
    }
  };

  const handleMapPoseSet = (x: number, y: number, yaw: number) => {
    onError(null);
    try {
      if (interactionMode === "pose_estimate") {
        client.publish(
          TOPICS.INITIALPOSE,
          "geometry_msgs/msg/PoseWithCovarianceStamped",
          buildInitialPoseMessage({ x, y, z: 0.0, yaw }),
        );
      } else if (interactionMode === "nav_goal") {
        // BT を指定できるよう、/goal_pose ではなく sequencer のサービスでゴールを送る
        const pose = buildNavGoalMessage({ x, y, z: 0.0, yaw });
        void goalRunner.run(async () =>
          describeServiceResponse(
            await client.callService(SERVICES.WAYPOINT_NAVIGATE_TO_POSE, {
              pose,
              navigation_mode: goalBt,
            }),
          ),
        );
      }
    } catch (e) {
      onError(e instanceof Error ? e.message : String(e));
    } finally {
      setInteractionMode("none");
    }
  };

  return {
    countdownMs,
    setCountdownMs,
    jumpIndex,
    setJumpIndex,
    interactionMode,
    setInteractionMode,
    goalRunner,
    loading,
    error,
    start,
    startImmediate,
    stop,
    pause,
    resume,
    jump,
    reload,
    publishInitialPose,
    handleMapPoseSet,
  };
}
