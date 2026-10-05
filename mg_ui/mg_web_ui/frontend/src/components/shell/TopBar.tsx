import { NavLink } from "react-router-dom";
import { ConnectionStatus, FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useTopicSubscriber } from "../../hooks/useTopicSubscriber";
import { useTheme } from "../../contexts/ThemeContext";
import { BoolMsg } from "../../types";
import { TOPICS } from "../../ros/interfaces";
import IconButton from "../ui/IconButton";
import Pill, { Tone } from "../ui/Pill";
import { MoonIcon, SunIcon } from "../ui/icons";

const CONNECTION: Record<ConnectionStatus, { label: string; tone: Tone }> = {
  connected: { label: "接続中", tone: "ok" },
  connecting: { label: "接続しています", tone: "warn" },
  disconnected: { label: "切断", tone: "error" },
  error: { label: "エラー", tone: "error" },
};

function navClass({ isActive }: { isActive: boolean }) {
  return `rounded-lg px-3 py-1.5 text-sm font-semibold transition-colors ${
    isActive
      ? "bg-accent text-surface-elevated"
      : "text-muted hover:bg-surface-sunken hover:text-content"
  }`;
}

export interface UseCaseLink {
  id: string;
  label: string;
}

interface Props {
  client: FoxgloveClientHandle;
  useCases: UseCaseLink[];
  onSettingClick: () => void;
}

export default function TopBar({ client, useCases, onSettingClick }: Props) {
  const { resolved, setMode, lowLoad, setLowLoad } = useTheme();
  // E-stop は安全に関わるので、どのビューでも見えるようにする(1Hz の軽いトピック)
  const estop = useTopicSubscriber<BoolMsg>(
    client,
    TOPICS.EMERGENCY_STOP,
    "std_msgs/msg/Bool",
  );
  const connection = CONNECTION[client.status];

  return (
    <header className="flex items-center gap-4 border-b border-line bg-surface-elevated px-4 py-2">
      <span className="text-base font-bold tracking-wide">MG-01</span>
      <nav className="flex gap-1">
        {useCases.map((u) => (
          <NavLink key={u.id} to={`/ops/${u.id}`} className={navClass}>
            {u.label}
          </NavLink>
        ))}
        <span className="mx-1 w-px bg-line" />
        <NavLink to="/ops/sensors" className={navClass}>
          センサ
        </NavLink>
      </nav>
      <div className="ml-auto flex items-center gap-2">
        <Pill tone={estop?.data ? "error" : "neutral"}>
          E-Stop {estop?.data ? "作動中" : "OFF"}
        </Pill>
        <Pill tone={connection.tone}>{connection.label}</Pill>
        <button
          type="button"
          aria-pressed={lowLoad}
          title="影とアニメーションを切り、更新を減らす(端末ごとの設定)"
          onClick={() => setLowLoad(!lowLoad)}
          className={`rounded-lg px-2.5 py-1 text-xs font-semibold ${
            lowLoad ? "bg-accent text-surface-elevated" : "text-muted hover:bg-surface-sunken"
          }`}
        >
          低負荷
        </button>
        <IconButton
          icon={resolved === "dark" ? <SunIcon /> : <MoonIcon />}
          title={resolved === "dark" ? "ライトテーマにする" : "ダークテーマにする"}
          onClick={() => setMode(resolved === "dark" ? "light" : "dark")}
        />
        <button
          type="button"
          onClick={onSettingClick}
          className="rounded-lg px-2.5 py-1 text-xs font-semibold text-muted hover:bg-surface-sunken"
        >
          設定
        </button>
        <NavLink to="/" className="text-xs text-muted underline">
          旧 UI
        </NavLink>
      </div>
    </header>
  );
}
