import { describe, expect, it, vi } from "vitest";
import { createOpsViewStore, KeyValueStorage } from "./opsViewStore";

function memoryStorage(initial: Record<string, string> = {}): KeyValueStorage & {
  data: Record<string, string>;
} {
  const data = { ...initial };
  return {
    data,
    getItem: (k) => (k in data ? data[k] : null),
    setItem: (k, v) => {
      data[k] = v;
    },
  };
}

describe("opsViewStore", () => {
  it("値がなければ既定値を返す", () => {
    const store = createOpsViewStore(memoryStorage());
    expect(store.get("a", 5)).toBe(5);
  });

  it("保存した値を、別のストア(再読み込み)でも読める", () => {
    const storage = memoryStorage();
    createOpsViewStore(storage).set("follow", true);
    expect(createOpsViewStore(storage).get("follow", false)).toBe(true);
  });

  it("persist=false の値は、メモリにだけ持ち保存しない", () => {
    const storage = memoryStorage();
    const store = createOpsViewStore(storage);
    store.set("camera", { x: 1 }, false);
    expect(store.get("camera", null)).toEqual({ x: 1 });
    expect(createOpsViewStore(storage).get("camera", null)).toBeNull();
  });

  it("壊れた保存値は無視する", () => {
    const store = createOpsViewStore(memoryStorage({ "mg-ui-ops-view": "{oops" }));
    expect(store.get("a", 1)).toBe(1);
  });

  it("保存できない環境(storage なし)でも、メモリ上では保持する", () => {
    const store = createOpsViewStore(null);
    store.set("a", 2);
    expect(store.get("a", 0)).toBe(2);
  });

  it("変更を購読者に通知し、解除後は通知しない", () => {
    const store = createOpsViewStore(null);
    const listener = vi.fn();
    const off = store.subscribe(listener);
    store.set("a", 1);
    off();
    store.set("a", 2);
    expect(listener).toHaveBeenCalledTimes(1);
  });
});
