import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { ContainerDetail, ContainerLevel } from "../../types/api";
import { useSystemManagerAction } from "../../hooks/useSystemManagerAction";
import OpsButton from "../ui/OpsButton";
import Pill, { Tone } from "../ui/Pill";

const LEVEL_TONE: Record<ContainerLevel, Tone> = {
  ok: "ok",
  warn: "warn",
  error: "error",
  off: "neutral",
};

const STATE_LABEL: Record<string, string> = {
  running: "起動中",
  exited: "停止",
  dead: "異常終了",
  restarting: "再起動中",
  paused: "一時停止",
  created: "未起動",
};

/** system_manager が判定した深刻度 `level` を、表示の文言と色にする。UI は判定しない */
export function containerDisplay(detail: ContainerDetail | undefined): {
  label: string;
  tone: Tone;
} {
  if (!detail) return { label: "不明(system_manager に届いていません)", tone: "warn" };
  const label = STATE_LABEL[detail.state] ?? detail.state;
  return {
    label: detail.health ? `${label} (${detail.health})` : label,
    tone: LEVEL_TONE[detail.level] ?? "neutral",
  };
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
  const display = containerDisplay(sysManager.details[container]);
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
