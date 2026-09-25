import { act, renderHook } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { useSystemManagerClient } from "./useSystemManagerClient";

function jsonResponse(body: unknown, ok = true) {
  return { ok, json: async () => body } as Response;
}

let fetchMock: ReturnType<typeof vi.fn>;

beforeEach(() => {
  vi.useFakeTimers();
  fetchMock = vi.fn(async (url: string) => {
    if (url.endsWith("/status")) return jsonResponse({ slam: "running" });
    return jsonResponse({ success: true, message: "ok" });
  });
  vi.stubGlobal("fetch", fetchMock);
  Object.defineProperty(document, "hidden", {
    configurable: true,
    get: () => false,
  });
});

afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

describe("useSystemManagerClient", () => {
  it("loads container status and polls periodically", async () => {
    const { result } = renderHook(() => useSystemManagerClient());
    await act(async () => {});
    expect(result.current.containers).toEqual({ slam: "running" });

    fetchMock.mockClear();
    await act(async () => {
      await vi.advanceTimersByTimeAsync(4000);
    });

    expect(fetchMock).toHaveBeenCalledTimes(2);
  });

  it("does not poll while the tab is hidden", async () => {
    Object.defineProperty(document, "hidden", {
      configurable: true,
      get: () => true,
    });
    renderHook(() => useSystemManagerClient());
    await act(async () => {});
    fetchMock.mockClear();

    await act(async () => {
      await vi.advanceTimersByTimeAsync(10000);
    });

    expect(fetchMock).not.toHaveBeenCalled();
  });

  it("refreshes status right after an API call", async () => {
    const { result } = renderHook(() => useSystemManagerClient());
    await act(async () => {});
    fetchMock.mockClear();

    await act(async () => {
      await result.current.callApi("/slam/start");
    });

    const urls = fetchMock.mock.calls.map((call) => call[0] as string);
    expect(urls.some((u) => u.endsWith("/slam/start"))).toBe(true);
    expect(urls.some((u) => u.endsWith("/status"))).toBe(true);
  });

  it("logs a failed HTTP response as a failure", async () => {
    fetchMock.mockImplementation(async (url: string) =>
      url.endsWith("/status")
        ? jsonResponse({})
        : jsonResponse({ detail: "invalid input" }, false),
    );
    const { result } = renderHook(() => useSystemManagerClient());
    await act(async () => {});

    let response: { success: boolean; message: string } | undefined;
    await act(async () => {
      response = await result.current.callApi("/x/start", { a: 1 });
    });

    expect(response).toMatchObject({ success: false, message: "invalid input" });
    expect(result.current.logs[0]).toMatchObject({
      path: "/x/start",
      success: false,
      message: "invalid input",
    });
  });

  it("logs and rethrows network errors", async () => {
    fetchMock.mockImplementation(async (url: string) => {
      if (url.endsWith("/status")) return jsonResponse({});
      throw new Error("network down");
    });
    const { result } = renderHook(() => useSystemManagerClient());
    await act(async () => {});

    await act(async () => {
      await expect(result.current.callApi("/slam/start")).rejects.toThrow(
        "network down",
      );
    });

    expect(result.current.logs[0]).toMatchObject({
      success: false,
      message: "network down",
    });
  });

  it("keeps the last known status when system_manager is unreachable", async () => {
    const { result } = renderHook(() => useSystemManagerClient());
    await act(async () => {});
    fetchMock.mockRejectedValue(new Error("offline"));

    await act(async () => {
      await vi.advanceTimersByTimeAsync(2000);
    });

    expect(result.current.containers).toEqual({ slam: "running" });
  });
});
