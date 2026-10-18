import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useTopicSubscriber } from "../../hooks/useTopicSubscriber";
import { useGnss } from "../../hooks/useGnss";
import { useFreshness } from "../../hooks/useFreshness";
import { useDiagnosticsAgg } from "../../hooks/useDiagnosticsAgg";
import {
  Float32Msg,
  LocalizationStatus,
  LOCALIZATION_STATE_NAME,
} from "../../types";
import { TOPICS } from "../../ros/interfaces";
import { DIAG_PATHS } from "../../ros/diagnosticPaths";
import { findNode } from "../../utils/diagnosticsTree";
import Card from "../ui/Card";
import StatTile from "../ui/StatTile";
import VelocityTile from "./VelocityTile";
import { usePanelVisible } from "./operatePanels";
import { localizationDetail } from "./localizationText";
import { CpuIcon, SatelliteIcon } from "../ui/icons";
import { Tone, toneFromDiagLevel } from "../ui/Pill";

// これを超えて受信がなければ値を古いものとして扱う(秒)
const STALE_AFTER_SEC = 5;
// GNSS の表示は人が読む速さで十分なので間引く
const GNSS_HZ = 2;

/**
 * CPU・メモリ・自己位置推定・GNSS・速度の指標。受信が途切れたら値を消して経過秒を出す。
 * 値は型付きのトピックから、色(良し悪し)は ROS 側の診断(/diagnostics_agg)から決める。UI では判定しない。
 * 値の桁や文字数が変わっても大きさが変わらないよう、各タイルの幅を固定している。
 */
export default function KpiRow({ client }: { client: FoxgloveClientHandle }) {
  const cpu = useTopicSubscriber<Float32Msg>(client, TOPICS.CPU_USAGE, "std_msgs/msg/Float32");
  const mem = useTopicSubscriber<Float32Msg>(client, TOPICS.MEMORY_USAGE, "std_msgs/msg/Float32");
  const loc = useTopicSubscriber<LocalizationStatus>(
    client,
    TOPICS.LOCALIZATION_STATUS,
    "localization_supervisor_msgs/msg/LocalizationStatus",
  );
  const gnss = useGnss(client, GNSS_HZ);
  const diagnostics = useDiagnosticsAgg(client);
  const diagTone = (path: string): Tone => {
    const node = findNode(diagnostics, path);
    return node ? toneFromDiagLevel(node.level) : "neutral";
  };
  const [gnssOpen, setGnssOpen] = usePanelVisible("gnssDetail");

  const cpuAge = useFreshness(client, TOPICS.CPU_USAGE, STALE_AFTER_SEC);
  const memAge = useFreshness(client, TOPICS.MEMORY_USAGE, STALE_AFTER_SEC);
  const locAge = useFreshness(client, TOPICS.LOCALIZATION_STATUS, STALE_AFTER_SEC);

  return (
    <Card className="flex divide-x divide-line">
      <StatTile
        widthClass="w-32"
        icon={<CpuIcon />}
        label="CPU"
        value={cpu ? cpu.data.toFixed(0) : "--"}
        unit="%"
        tone={diagTone(DIAG_PATHS.CPU)}
        stale={cpuAge.stale}
        ageSec={cpuAge.ageSec}
      />
      <StatTile
        widthClass="w-28"
        label="Memory"
        value={mem ? mem.data.toFixed(0) : "--"}
        unit="%"
        tone={diagTone(DIAG_PATHS.MEMORY)}
        stale={memAge.stale}
        ageSec={memAge.ageSec}
      />
      <StatTile
        widthClass="w-36"
        label="自己位置"
        value={loc ? (LOCALIZATION_STATE_NAME[loc.state] ?? String(loc.state)) : "--"}
        detail={loc ? localizationDetail(loc) : ""}
        tone={diagTone(DIAG_PATHS.LOCALIZATION)}
        stale={locAge.stale}
        ageSec={locAge.ageSec}
      />
      <StatTile
        widthClass="w-36"
        icon={<SatelliteIcon />}
        label="GNSS"
        value={gnss.reading ? gnss.reading.label : "--"}
        tone={diagTone(DIAG_PATHS.GNSS)}
        stale={gnss.freshness.stale}
        ageSec={gnss.freshness.ageSec}
        onClick={() => setGnssOpen(!gnssOpen)}
        active={gnssOpen}
      />
      <VelocityTile client={client} />
    </Card>
  );
}
