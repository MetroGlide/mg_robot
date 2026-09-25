import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { useTopicSubscriber } from "../../../hooks/useTopicSubscriber"
import { MarkerArray } from "../../../types/ros-types"

export function useMarkerArray(
  client: FoxgloveClientHandle,
  topic: string,
): MarkerArray | null {
  return useTopicSubscriber<MarkerArray>(
    client,
    topic,
    "visualization_msgs/msg/MarkerArray",
  )
}
