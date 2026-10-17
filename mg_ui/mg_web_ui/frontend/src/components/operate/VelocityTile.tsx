import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useThrottledTopic } from "../../hooks/useThrottledTopic";
import { useFreshness } from "../../hooks/useFreshness";
import { useTeleop } from "../../contexts/TeleopContext";
import { OdomMsg, TwistMsg } from "../../types";
import { TOPICS } from "../../ros/interfaces";

// 指令と実測の表示は人が読む速さで十分なので間引く
const VELOCITY_HZ = 4;
const ODOM_STALE_AFTER_SEC = 2;

interface BarProps {
  label: string;
  unit: string;
  /** 指令値(cmd_vel) */
  cmd: number | null;
  /** 実測値(odom)。受信が途切れたら null */
  actual: number | null;
  max: number;
}

function ratio(value: number | null, max: number): number {
  if (value === null || max <= 0) return 0;
  return Math.min(Math.abs(value) / max, 1);
}

function format(value: number | null): string {
  return value === null ? "--" : value.toFixed(2);
}

/** 上段が指令、下段が実測。バーの長さは上限に対する大きさ(符号は数値で見る) */
function VelocityBar({ label, unit, cmd, actual, max }: BarProps) {
  return (
    <div className="flex items-center gap-2">
      <span className="w-9 shrink-0 text-xs text-muted">{label}</span>
      <div className="w-24 shrink-0 space-y-0.5">
        <div className="h-1.5 overflow-hidden rounded bg-surface-sunken">
          <div
            className="h-full rounded bg-accent"
            style={{ width: `${ratio(cmd, max) * 100}%` }}
          />
        </div>
        <div className="h-1.5 overflow-hidden rounded bg-surface-sunken">
          <div
            className="h-full rounded bg-ok"
            style={{ width: `${ratio(actual, max) * 100}%` }}
          />
        </div>
      </div>
      <div className="w-28 shrink-0 text-right text-[11px] tabular-nums leading-tight">
        <div className="text-accent">
          {format(cmd)} <span className="text-muted">{unit}</span>
        </div>
        <div className="text-ok">
          {format(actual)} <span className="text-muted">{unit}</span>
        </div>
      </div>
    </div>
  );
}

/**
 * 並進と旋回の速度。cmd_vel(指令、上段)と odom(実測、下段)を並べる。
 * 幅は固定で、値が変わっても大きさは変わらない。
 */
export default function VelocityTile({ client }: { client: FoxgloveClientHandle }) {
  const { effectiveGaugeMaxLinear, effectiveGaugeMaxAngular } = useTeleop();
  const cmdVel = useThrottledTopic<TwistMsg>(
    client,
    TOPICS.CMD_VEL,
    "geometry_msgs/msg/Twist",
    VELOCITY_HZ,
  );
  const odom = useThrottledTopic<OdomMsg>(
    client,
    TOPICS.ODOM,
    "nav_msgs/msg/Odometry",
    VELOCITY_HZ,
  );
  const odomAge = useFreshness(client, TOPICS.ODOM, ODOM_STALE_AFTER_SEC);

  const actual = odomAge.stale ? null : (odom?.twist.twist ?? null);

  return (
    <div className="w-[21rem] shrink-0 space-y-1 px-3 py-2">
      <VelocityBar
        label="並進"
        unit="m/s"
        cmd={cmdVel ? cmdVel.linear.x : null}
        actual={actual ? actual.linear.x : null}
        max={effectiveGaugeMaxLinear}
      />
      <VelocityBar
        label="旋回"
        unit="rad/s"
        cmd={cmdVel ? cmdVel.angular.z : null}
        actual={actual ? actual.angular.z : null}
        max={effectiveGaugeMaxAngular}
      />
    </div>
  );
}
