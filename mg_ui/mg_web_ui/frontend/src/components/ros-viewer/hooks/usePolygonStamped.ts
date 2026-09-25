import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { useTopicSubscriber } from "../../../hooks/useTopicSubscriber"
import { PolygonStamped } from "../../../types/ros-types"

export function usePolygonStamped(
  client: FoxgloveClientHandle,
  topic: string,
): PolygonStamped | null {
  return useTopicSubscriber<PolygonStamped>(
    client,
    topic,
    "geometry_msgs/msg/PolygonStamped",
  )
}
