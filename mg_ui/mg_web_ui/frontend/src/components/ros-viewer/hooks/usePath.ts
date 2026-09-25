import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { useTopicSubscriber } from "../../../hooks/useTopicSubscriber"
import { Path } from "../../../types/ros-types"

export function usePath(client: FoxgloveClientHandle, topic: string): Path | null {
  return useTopicSubscriber<Path>(client, topic, "nav_msgs/msg/Path")
}
