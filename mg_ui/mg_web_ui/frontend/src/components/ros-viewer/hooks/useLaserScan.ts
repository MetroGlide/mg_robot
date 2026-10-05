import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { useTopicSubscriber } from "../../../hooks/useTopicSubscriber"
import { LaserScan } from "../../../types/ros-types"

export function useLaserScan(
  client: FoxgloveClientHandle,
  topic: string,
): LaserScan | null {
  return useTopicSubscriber<LaserScan>(client, topic, "sensor_msgs/msg/LaserScan")
}
