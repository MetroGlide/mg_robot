import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useTopicSubscriber } from "../../hooks/useTopicSubscriber";
import { useWaypointControl } from "../../hooks/useWaypointControl";
import { SequencerStatus } from "../../types";
import { TOPICS } from "../../ros/interfaces";
import Card from "../ui/Card";
import OpsButton from "../ui/OpsButton";
import Stepper from "../ui/Stepper";
import { SEQUENCER_STEPS, sequencerStep } from "./displayState";

interface Props {
  client: FoxgloveClientHandle;
  control: ReturnType<typeof useWaypointControl>;
  /** 切断中は操作を受け付けない */
  connected: boolean;
}

/**
 * Waypoint シーケンサの進捗と、開始・停止・一時停止の操作。
 * 状態の変化を見逃さないよう、status は間引かずに受ける(10Hz で、この部品だけが再描画される)。
 */
export default function WaypointProgress({ client, control, connected }: Props) {
  const status = useTopicSubscriber<SequencerStatus>(
    client,
    TOPICS.WAYPOINT_STATUS,
    "mg_msgs/msg/SequencerStatus",
  );
  const { currentId, tone } = sequencerStep(status?.state ?? null);
  const busy = control.loading || !connected;

  return (
    <Card className="w-[34rem] max-w-full p-4">
      <div className="mb-3 flex items-baseline justify-between">
        <h2 className="text-sm font-bold">Waypoint 走行</h2>
        {status && (
          <span className="text-xs tabular-nums text-muted">
            {Math.min(status.current_index + 1, status.total_waypoints)} /{" "}
            {status.total_waypoints} 地点 ・ 残り{" "}
            {status.distance_remaining.toFixed(1)} m
          </span>
        )}
      </div>
      <Stepper steps={SEQUENCER_STEPS} currentId={currentId} tone={tone} />
      {status && status.countdown_ms_remaining > 0 && (
        <p className="mt-2 text-xs text-warn">
          出発まで {(status.countdown_ms_remaining / 1000).toFixed(1)} 秒
        </p>
      )}
      {status?.is_paused && (
        <p className="mt-2 text-xs text-warn">
          一時停止中: {status.pause_requesters.join(", ")}
        </p>
      )}
      {status?.state === "ERROR" && (
        <p className="mt-2 text-xs text-error">シーケンサがエラー状態です</p>
      )}
      <div className="mt-3 flex flex-wrap items-center gap-2">
        <OpsButton tone="primary" disabled={busy} onClick={control.start}>
          START
        </OpsButton>
        <OpsButton tone="danger" disabled={busy} onClick={control.stop}>
          STOP
        </OpsButton>
        {status?.is_paused ? (
          <OpsButton tone="primary" disabled={!connected} onClick={control.resume}>
            RESUME
          </OpsButton>
        ) : (
          <OpsButton tone="warn" disabled={!connected} onClick={control.pause}>
            PAUSE
          </OpsButton>
        )}
        <label className="ml-auto flex items-center gap-1.5 text-xs text-muted">
          出発までの待ち
          <input
            type="number"
            min={0}
            step={500}
            value={control.countdownMs}
            onChange={(e) => control.setCountdownMs(Number(e.target.value))}
            className="w-20 rounded-md border border-line bg-surface-sunken px-2 py-1 text-xs text-content"
          />
          ms
        </label>
      </div>
      {control.error && (
        <p className="mt-2 text-xs text-error">{control.error}</p>
      )}
    </Card>
  );
}
