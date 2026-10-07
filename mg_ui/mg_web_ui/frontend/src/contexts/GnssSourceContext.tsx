import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
  ReactNode,
} from "react";
import { loadSettings, saveSettings } from "../utils/settingsApi";
import type { GnssSource } from "../utils/gnssReading";

const SETTINGS_KEY = "gnss";
const DEFAULT_SOURCE: GnssSource = "navpvt";

interface GnssSourceContextType {
  /** GNSS の表示(KPI と詳細)が購読するトピック。選んだ 1 つだけを購読する */
  source: GnssSource;
  setSource: (source: GnssSource) => void;
}

const GnssSourceContext = createContext<GnssSourceContextType>({
  source: DEFAULT_SOURCE,
  setSource: () => {},
});

function isGnssSource(value: unknown): value is GnssSource {
  return value === "navpvt" || value === "navsatfix";
}

/** GNSS の購読元。system_manager の設定に保存し、端末間で共有する */
export function GnssSourceProvider({ children }: { children: ReactNode }) {
  const [source, setSourceState] = useState<GnssSource>(DEFAULT_SOURCE);

  useEffect(() => {
    loadSettings().then((data) => {
      const saved = (data[SETTINGS_KEY] as { source?: unknown } | undefined)?.source;
      if (isGnssSource(saved)) setSourceState(saved);
    });
  }, []);

  const setSource = useCallback((next: GnssSource) => {
    setSourceState(next);
    saveSettings(SETTINGS_KEY, { source: next });
  }, []);

  const value = useMemo(() => ({ source, setSource }), [source, setSource]);
  return (
    <GnssSourceContext.Provider value={value}>
      {children}
    </GnssSourceContext.Provider>
  );
}

export function useGnssSource() {
  return useContext(GnssSourceContext);
}
