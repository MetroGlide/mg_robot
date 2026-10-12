import { NavigationActivity } from "../types";
import { TOPICS } from "../ros/topics";
import { FoxgloveClientHandle } from "./useFoxgloveClient";
import { useFreshness } from "./useFreshness";
import { useTopicSubscriber } from "./useTopicSubscriber";

// シーケンサは 2Hz で配信する。これを超えて途絶えたら、古い表示を残さない
const MAX_AGE_SEC = 3;

/**
 * Waypoint 走行の「いまの活動」。判定はロボット側 (waypoint_sequencer_node) が行う。
 * 一度も届いていない、または途絶えているときは null (配信を止めている場合も含む)。
 */
export function useNavigationActivity(
  client: FoxgloveClientHandle,
): NavigationActivity | null {
  const activity = useTopicSubscriber<NavigationActivity>(
    client,
    TOPICS.WAYPOINT_ACTIVITY,
    "mg_msgs/msg/NavigationActivity",
  );
  const { stale } = useFreshness(client, TOPICS.WAYPOINT_ACTIVITY, MAX_AGE_SEC);
  return stale ? null : activity;
}
