import { describe, expect, it } from "vitest";
import { buildInitialPoseMessage, buildNavGoalMessage } from "./useWaypointControl";

describe("buildInitialPoseMessage", () => {
  it("yaw をクォータニオンにし、共分散を設定する", () => {
    const msg = buildInitialPoseMessage({ x: 1, y: 2, z: 0, yaw: Math.PI });
    expect(msg.header.frame_id).toBe("map");
    expect(msg.pose.pose.position).toEqual({ x: 1, y: 2, z: 0 });
    expect(msg.pose.pose.orientation.z).toBeCloseTo(1);
    expect(msg.pose.pose.orientation.w).toBeCloseTo(0);
    expect(msg.pose.covariance).toHaveLength(36);
    expect(msg.pose.covariance[0]).toBe(0.25);
    expect(msg.pose.covariance[35]).toBeCloseTo(0.0685, 4);
  });
});

describe("buildNavGoalMessage", () => {
  it("map フレームの PoseStamped を作る", () => {
    const msg = buildNavGoalMessage({ x: 3, y: -1, z: 0, yaw: 0 });
    expect(msg.header.frame_id).toBe("map");
    expect(msg.pose.position).toEqual({ x: 3, y: -1, z: 0 });
    expect(msg.pose.orientation).toEqual({ x: 0, y: 0, z: 0, w: 1 });
  });
});
