import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, renderHook } from "@testing-library/react";
import { FoxgloveClientHandle } from "./useFoxgloveClient";
import { useThrottledTopic } from "./useThrottledTopic";

function makeClient() {
  let listener: ((data: unknown) => void) | null = null;
  const unsubscribe = vi.fn();
  const subscribe = vi.fn(
    (_topic: string, _schema: string, cb: (data: unknown) => void) => {
      listener = cb;
      return unsubscribe;
    },
  );
  const client = { subscribe } as unknown as FoxgloveClientHandle;
  return { client, unsubscribe, emit: (v: unknown) => listener?.(v) };
}

describe("useThrottledTopic", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(10_000);
  });
  afterEach(() => {
    vi.useRealTimers();
  });

  it("最初のメッセージはすぐに反映する", () => {
    const { client, emit } = makeClient();
    const { result } = renderHook(() =>
      useThrottledTopic<number>(client, "/t", "x/msg/Y", 2),
    );
    act(() => emit(1));
    expect(result.current).toBe(1);
  });

  it("間引き中のメッセージは捨てるが、最後の値は間隔の終わりに反映する", () => {
    const { client, emit } = makeClient();
    const { result } = renderHook(() =>
      useThrottledTopic<number>(client, "/t", "x/msg/Y", 2),
    );
    act(() => emit(1));
    act(() => {
      emit(2);
      emit(3);
    });
    expect(result.current).toBe(1);
    act(() => {
      vi.advanceTimersByTime(500);
    });
    expect(result.current).toBe(3);
  });

  it("アンマウントで購読を解除し、保留中の更新を取り消す", () => {
    const { client, emit, unsubscribe } = makeClient();
    const { result, unmount } = renderHook(() =>
      useThrottledTopic<number>(client, "/t", "x/msg/Y", 2),
    );
    act(() => emit(1));
    act(() => emit(2));
    unmount();
    expect(unsubscribe).toHaveBeenCalledTimes(1);
    act(() => {
      vi.advanceTimersByTime(1000);
    });
    expect(result.current).toBe(1);
  });
});
