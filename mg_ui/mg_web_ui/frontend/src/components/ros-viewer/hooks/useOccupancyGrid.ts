import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { useTopicSubscriber } from "../../../hooks/useTopicSubscriber"
import { OccupancyGrid } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

/** 占有格子地図・コストマップの最新メッセージ。topic 省略時は /map。 */
export function useOccupancyGrid(client: FoxgloveClientHandle, topic?: string): OccupancyGrid | null {
  return useTopicSubscriber<OccupancyGrid>(
    client,
    topic ?? TOPICS.MAP,
    "nav_msgs/msg/OccupancyGrid",
  )
}
