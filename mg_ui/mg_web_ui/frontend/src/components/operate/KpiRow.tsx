import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useTopicSubscriber } from "../../hooks/useTopicSubscriber";
import { useThrottledTopic } from "../../hooks/useThrottledTopic";
import { useFreshness } from "../../hooks/useFreshness";
import {
  Float32Msg,
  LocalizationStatus,
  LOCALIZATION_STATE_NAME,
  NavSatFix,
} from "../../types";
import { TOPICS } from "../../ros/interfaces";
import Card from "../ui/Card";
import StatTile from "../ui/StatTile";
import VelocityTile from "./VelocityTile";
import { CpuIcon, SatelliteIcon } from "../ui/icons";
import { gnssDisplay, localizationTone } from "./displayState";

// これを超えて受信がなければ値を古いものとして扱う(秒)
const STALE_AFTER_SEC = 5;
// GNSS の表示は人が読む速さで十分なので間引く
const GPS_HZ = 1;

/**
 * CPU・メモリ・自己位置推定・GNSS・速度の指標。受信が途切れたら値を消して経過秒を出す。
 * 値の桁や文字数が変わっても大きさが変わらないよう、各タイルの幅を固定している。
 */
export default function KpiRow({ client }: { client: FoxgloveClientHandle }) {
  const cpu = useTopicSubscriber<Float32Msg>(client, TOPICS.CPU_USAGE, "std_msgs/msg/Float32");
  const mem = useTopicSubscriber<Float32Msg>(client, TOPICS.MEMORY_USAGE, "std_msgs/msg/Float32");
  const loc = useTopicSubscriber<LocalizationStatus>(
    client,
    TOPICS.LOCALIZATION_STATUS,
    "mg_msgs/msg/LocalizationStatus",
  );
  const gps = useThrottledTopic<NavSatFix>(client, TOPICS.GPS_FIX, "sensor_msgs/msg/NavSatFix", GPS_HZ);

  const cpuAge = useFreshness(client, TOPICS.CPU_USAGE, STALE_AFTER_SEC);
  const memAge = useFreshness(client, TOPICS.MEMORY_USAGE, STALE_AFTER_SEC);
  const locAge = useFreshness(client, TOPICS.LOCALIZATION_STATUS, STALE_AFTER_SEC);
  const gpsAge = useFreshness(client, TOPICS.GPS_FIX, STALE_AFTER_SEC);

  const gnss = gps ? gnssDisplay(gps.status.status) : null;

  return (
    <Card className="flex divide-x divide-line">
      <StatTile
        widthClass="w-32"
        icon={<CpuIcon />}
        label="CPU"
        value={cpu ? cpu.data.toFixed(0) : "--"}
        unit="%"
        tone={cpu && cpu.data > 85 ? "warn" : "neutral"}
        stale={cpuAge.stale}
        ageSec={cpuAge.ageSec}
      />
      <StatTile
        widthClass="w-28"
        label="Memory"
        value={mem ? mem.data.toFixed(0) : "--"}
        unit="%"
        tone={mem && mem.data > 90 ? "warn" : "neutral"}
        stale={memAge.stale}
        ageSec={memAge.ageSec}
      />
      <StatTile
        widthClass="w-36"
        label="自己位置"
        value={loc ? (LOCALIZATION_STATE_NAME[loc.state] ?? String(loc.state)) : "--"}
        tone={loc ? localizationTone(loc.state) : "neutral"}
        stale={locAge.stale}
        ageSec={locAge.ageSec}
      />
      <StatTile
        widthClass="w-32"
        icon={<SatelliteIcon />}
        label="GNSS"
        value={gnss ? gnss.label : "--"}
        tone={gnss ? gnss.tone : "neutral"}
        stale={gpsAge.stale}
        ageSec={gpsAge.ageSec}
      />
      <VelocityTile client={client} />
    </Card>
  );
}
