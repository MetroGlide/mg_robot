import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useSystemManagerAction } from "../../hooks/useSystemManagerAction";
import OpsButton from "../ui/OpsButton";
import Pill, { Tone } from "../ui/Pill";

const STATE_DISPLAY: Record<string, { label: string; tone: Tone }> = {
  running: { label: "起動中", tone: "ok" },
  exited: { label: "停止", tone: "neutral" },
  dead: { label: "異常終了", tone: "error" },
};

/** system_manager が返すコンテナの状態を、表示の文言と色にする */
export function containerDisplay(state: string | undefined): {
  label: string;
  tone: Tone;
} {
  if (!state) return { label: "不明(system_manager に届いていません)", tone: "warn" };
  return STATE_DISPLAY[state] ?? { label: state, tone: "neutral" };
}

interface Props {
  sysManager: SystemManagerHandle;
  /** system_manager のコンテナ名。API は /<container>/start などになる */
  container: string;
  title: string;
  /** 起動中に「再起動」を出すか(/<container>/restart があるコンテナだけ) */
  restartable?: boolean;
}

/** コンテナの状態と、起動・再起動・停止のボタン。枠は呼び出し側で付ける */
export default function ContainerControl({
  sysManager,
  container,
  title,
  restartable = true,
}: Props) {
  const { call, loading, error } = useSystemManagerAction(sysManager);
  const state = sysManager.containers[container];
  const display = containerDisplay(state);
  const running = state === "running";

  return (
    <div className="space-y-2">
      <div className="flex items-center justify-between gap-2">
        <span className="text-xs font-semibold">{title}</span>
        <Pill tone={display.tone}>{display.label}</Pill>
      </div>
      <div className="flex flex-wrap gap-2">
        {running && restartable ? (
          <OpsButton disabled={loading} onClick={() => call(`/${container}/restart`)}>
            再起動
          </OpsButton>
        ) : (
          <OpsButton
            tone="primary"
            disabled={loading || running}
            onClick={() => call(`/${container}/start`)}
          >
            起動
          </OpsButton>
        )}
        <OpsButton
          tone="danger"
          disabled={loading || !running}
          onClick={() => call(`/${container}/stop`)}
        >
          停止
        </OpsButton>
      </div>
      {error && <p className="text-xs text-error">{error}</p>}
    </div>
  );
}
