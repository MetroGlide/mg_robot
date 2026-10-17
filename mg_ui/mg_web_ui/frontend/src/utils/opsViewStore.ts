/**
 * 新 UI の操作状態(追従・開閉・タブの選択・入力値など)を、画面の切り替えをまたいで保持するストア。
 *
 * ルートを切り替えるとページのコンポーネントは破棄されるので、state に持つと失われる。
 * ここに置いた値は、同じ端末のブラウザの localStorage にも保存して、再読み込みでも残す
 * (全端末で共有される settingsApi は使わない。端末ごとの画面の好みなので)。
 * 走行に関わる操作(指定モードなど)は、誤操作を避けるため保持しない。
 */

export interface KeyValueStorage {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
}

export interface OpsViewStore {
  get<T>(key: string, fallback: T): T;
  /** persist が false の値は、メモリにだけ持つ(カメラの位置など、再読み込みで捨ててよいもの) */
  set<T>(key: string, value: T, persist?: boolean): void;
  subscribe(listener: () => void): () => void;
}

const STORAGE_KEY = "mg-ui-ops-view";

function load(storage: KeyValueStorage | null): Record<string, unknown> {
  if (!storage) return {};
  try {
    const raw = storage.getItem(STORAGE_KEY);
    if (raw === null) return {};
    const parsed: unknown = JSON.parse(raw);
    return typeof parsed === "object" && parsed !== null && !Array.isArray(parsed)
      ? (parsed as Record<string, unknown>)
      : {};
  } catch {
    // 壊れた保存値は、なかったものとして扱う
    return {};
  }
}

export function createOpsViewStore(storage: KeyValueStorage | null): OpsViewStore {
  const persisted = load(storage);
  const transient = new Map<string, unknown>();
  const listeners = new Set<() => void>();

  const save = () => {
    if (!storage) return;
    try {
      storage.setItem(STORAGE_KEY, JSON.stringify(persisted));
    } catch {
      // 保存できない環境では、このセッションだけ有効にする
    }
  };

  return {
    get<T>(key: string, fallback: T): T {
      if (transient.has(key)) return transient.get(key) as T;
      return key in persisted ? (persisted[key] as T) : fallback;
    },
    set<T>(key: string, value: T, persist = true) {
      if (persist) {
        persisted[key] = value;
        transient.delete(key);
        save();
      } else {
        transient.set(key, value);
      }
      listeners.forEach((l) => l());
    },
    subscribe(listener) {
      listeners.add(listener);
      return () => {
        listeners.delete(listener);
      };
    },
  };
}

function browserStorage(): KeyValueStorage | null {
  try {
    return window.localStorage;
  } catch {
    return null;
  }
}

export const opsViewStore = createOpsViewStore(browserStorage());
