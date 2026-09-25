import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { useTopicSubscriber } from "../../../hooks/useTopicSubscriber"
import { ParticleCloud } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

export function useParticleCloud(client: FoxgloveClientHandle): ParticleCloud | null {
  return useTopicSubscriber<ParticleCloud>(
    client,
    TOPICS.PARTICLE_CLOUD,
    "nav2_msgs/msg/ParticleCloud",
  )
}
