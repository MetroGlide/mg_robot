import { NavigationActivity } from "../../types";
import { formatActivity } from "./activityText";
import StatusLamp from "./StatusLamp";
import { lampFor } from "./statusLamp";

/**
 * 走行の「いまの活動」を、状態ランプと 1 行の文で出す。
 * 届いていない・古いときは、通信途絶のランプ (赤) を出す。
 */
export default function ActivityLine({
  activity,
  lampSize = 22,
  className = "",
}: {
  activity: NavigationActivity | null;
  lampSize?: number;
  className?: string;
}) {
  return (
    <div className={`flex items-center gap-3 ${className}`}>
      <StatusLamp lamp={lampFor(activity?.code ?? null)} size={lampSize} />
      <p className="min-w-0">
        {activity ? formatActivity(activity) : "活動が届いていません"}
      </p>
    </div>
  );
}
