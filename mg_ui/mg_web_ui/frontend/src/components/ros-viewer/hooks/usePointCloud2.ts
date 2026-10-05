import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { useTopicSubscriber } from "../../../hooks/useTopicSubscriber"
import { PointCloud2 } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

export function usePointCloud2(client: FoxgloveClientHandle): PointCloud2 | null {
  return useTopicSubscriber<PointCloud2>(
    client,
    TOPICS.DEPTH_POINTS,
    "sensor_msgs/msg/PointCloud2",
  )
}
