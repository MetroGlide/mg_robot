import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, renderHook } from "@testing-library/react";
import { FoxgloveClientHandle } from "./useFoxgloveClient";
import { useNavigationActivity } from "./useNavigationActivity";

const MESSAGE = {
  stamp: { sec: 0, nanosec: 0 },
  code: "obstacle_stop",
  text: "障害物で停止中",
  detail: "stop_front",
  emphasis: 1,
  elapsed_sec: 2,
  index: 3,
  total: 10,
  remaining: -1,
};

function makeClient() {
  let listener: ((data: unknown) => void) | null = null;
  let lastAt: number | null = null;
  const subscribe = vi.fn(
    (_topic: string, _schema: string, cb: (data: unknown) => void) => {
      listener = cb;
      return vi.fn();
    },
  );
  const getLastMessageAt = vi.fn(() => lastAt);
  const client = { subscribe, getLastMessageAt } as unknown as FoxgloveClientHandle;
  return {
    client,
    subscribe,
    emit: (v: unknown) => {
      lastAt = Date.now();
      listener?.(v);
    },
  };
}

describe("useNavigationActivity", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(10_000);
  });
  afterEach(() => {
    vi.useRealTimers();
  });

  it("届くまでは null", () => {
    const { client } = makeClient();
    const { result } = renderHook(() => useNavigationActivity(client));
    expect(result.current).toBeNull();
  });

  it("NavigationActivity を購読して返す", () => {
    const { client, subscribe, emit } = makeClient();
    const { result } = renderHook(() => useNavigationActivity(client));
    expect(subscribe).toHaveBeenCalledWith(
      "/waypoint_sequencer_node/activity",
      "mg_msgs/msg/NavigationActivity",
      expect.any(Function),
    );
    act(() => emit(MESSAGE));
    // 鮮度の判定は 1 秒ごと
    act(() => {
      vi.advanceTimersByTime(1_000);
    });
    expect(result.current).toEqual(MESSAGE);
  });

  it("途絶えたら古い表示を残さず null にする", () => {
    const { client, emit } = makeClient();
    const { result } = renderHook(() => useNavigationActivity(client));
    act(() => emit(MESSAGE));
    act(() => {
      vi.advanceTimersByTime(1_000);
    });
    expect(result.current).not.toBeNull();
    act(() => {
      vi.advanceTimersByTime(5_000);
    });
    expect(result.current).toBeNull();
  });
});
