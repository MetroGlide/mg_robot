import { BoolMsg, GateArbiterState } from "../types";
import { TOPICS } from "../ros/topics";
import { FoxgloveClientHandle } from "./useFoxgloveClient";
import { useTopicSubscriber } from "./useTopicSubscriber";

export interface LocalizationGates {
  /** AMCL のゲートの状態。状態トピックが届いていなければ null */
  amcl: GateArbiterState | null;
  /** GNSS ブリッジが /odom/gps を配信しているか。状態トピックが届いていなければ null */
  gnssPublishing: boolean | null;
}

/** AMCL と GNSS の入切の現在の状態。ノードが状態を変えたとき (latched) に更新される。 */
export function useLocalizationGates(
  client: FoxgloveClientHandle,
): LocalizationGates {
  const amcl = useTopicSubscriber<GateArbiterState>(
    client,
    TOPICS.AMCL_GATE_STATE,
    "mg_msgs/msg/GateArbiterState",
  );
  const gnss = useTopicSubscriber<BoolMsg>(
    client,
    TOPICS.GNSS_PUBLISH_STATE,
    "std_msgs/msg/Bool",
  );
  return { amcl, gnssPublishing: gnss === null ? null : gnss.data };
}
