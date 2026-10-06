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

const SETTINGS_KEY = "robot";
const DEFAULT_ROBOT_NAME = "MG-02";

interface RobotProfileContextType {
  /** 画面の見出しやタブのタイトルに出すロボットの名前 */
  name: string;
  setName: (name: string) => void;
}

const RobotProfileContext = createContext<RobotProfileContextType>({
  name: DEFAULT_ROBOT_NAME,
  setName: () => {},
});

/** ロボットの名前。system_manager の設定に保存し、端末間で共有する */
export function RobotProfileProvider({ children }: { children: ReactNode }) {
  const [name, setNameState] = useState(DEFAULT_ROBOT_NAME);

  useEffect(() => {
    loadSettings().then((data) => {
      const saved = (data[SETTINGS_KEY] as { name?: unknown } | undefined)?.name;
      if (typeof saved === "string" && saved.trim() !== "") setNameState(saved);
    });
  }, []);

  useEffect(() => {
    document.title = `${name} Control UI`;
  }, [name]);

  const setName = useCallback((next: string) => {
    setNameState(next);
    saveSettings(SETTINGS_KEY, { name: next });
  }, []);

  const value = useMemo(() => ({ name, setName }), [name, setName]);
  return (
    <RobotProfileContext.Provider value={value}>
      {children}
    </RobotProfileContext.Provider>
  );
}

export function useRobotProfile() {
  return useContext(RobotProfileContext);
}
