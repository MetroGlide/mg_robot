import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
  ReactNode,
} from "react";

export type ThemeMode = "light" | "dark" | "system";
export type ResolvedTheme = "light" | "dark";

// テーマは端末ごとの好み(屋外のタブレットは明るい、夜間は暗いなど)なので、
// 全端末で共有される settingsApi ではなく localStorage に保存する。
const STORAGE_KEY = "mg-ui-theme";
const LOW_LOAD_KEY = "mg-ui-low-load";

interface ThemeContextType {
  mode: ThemeMode;
  resolved: ResolvedTheme;
  setMode: (mode: ThemeMode) => void;
  /** 影・トランジションを切り、描画と更新の頻度を下げる */
  lowLoad: boolean;
  setLowLoad: (value: boolean) => void;
}

const ThemeContext = createContext<ThemeContextType>({
  mode: "system",
  resolved: "dark",
  setMode: () => {},
  lowLoad: false,
  setLowLoad: () => {},
});

function readStorage(key: string): string | null {
  try {
    return window.localStorage.getItem(key);
  } catch {
    return null;
  }
}

function writeStorage(key: string, value: string): void {
  try {
    window.localStorage.setItem(key, value);
  } catch {
    // 保存できない環境(プライベートウィンドウなど)では、このセッションだけ有効にする
  }
}

export function parseThemeMode(value: string | null): ThemeMode {
  return value === "light" || value === "dark" ? value : "system";
}

export function resolveTheme(
  mode: ThemeMode,
  systemPrefersDark: boolean,
): ResolvedTheme {
  if (mode === "system") return systemPrefersDark ? "dark" : "light";
  return mode;
}

export function ThemeProvider({ children }: { children: ReactNode }) {
  const [mode, setModeState] = useState<ThemeMode>(() =>
    parseThemeMode(readStorage(STORAGE_KEY)),
  );
  const [lowLoad, setLowLoadState] = useState<boolean>(
    () => readStorage(LOW_LOAD_KEY) === "1",
  );
  const [systemDark, setSystemDark] = useState<boolean>(
    () => window.matchMedia("(prefers-color-scheme: dark)").matches,
  );

  useEffect(() => {
    const query = window.matchMedia("(prefers-color-scheme: dark)");
    const onChange = (e: MediaQueryListEvent) => setSystemDark(e.matches);
    query.addEventListener("change", onChange);
    return () => query.removeEventListener("change", onChange);
  }, []);

  const resolved = resolveTheme(mode, systemDark);

  useEffect(() => {
    const root = document.documentElement;
    root.classList.toggle("dark", resolved === "dark");
    root.classList.toggle("low-load", lowLoad);
  }, [resolved, lowLoad]);

  const setMode = useCallback((next: ThemeMode) => {
    setModeState(next);
    writeStorage(STORAGE_KEY, next);
  }, []);

  const setLowLoad = useCallback((value: boolean) => {
    setLowLoadState(value);
    writeStorage(LOW_LOAD_KEY, value ? "1" : "0");
  }, []);

  const value = useMemo(
    () => ({ mode, resolved, setMode, lowLoad, setLowLoad }),
    [mode, resolved, setMode, lowLoad, setLowLoad],
  );

  return (
    <ThemeContext.Provider value={value}>{children}</ThemeContext.Provider>
  );
}

export function useTheme(): ThemeContextType {
  return useContext(ThemeContext);
}
