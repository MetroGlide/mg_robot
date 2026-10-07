import { ReactNode } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useActionRunner } from "../../hooks/useActionRunner";
import { useLocalizationGates } from "../../hooks/useLocalizationGates";
import { useNavigationMaps } from "../../hooks/useNavigationMaps";
import { SERVICES } from "../../ros/services";
import {
  describeAmclGate,
  describeGnssGate,
  describeServiceResponse,
} from "../../utils/waypointActions";
import SectionCard from "../layout/SectionCard";
import ActionButton from "../ui/ActionButton";
import ActionResultText from "./ActionResultText";
import GateRow from "./GateRow";
import GenericServiceCaller from "./GenericServiceCaller";
import GenericTopicPublisher from "./GenericTopicPublisher";
import MapSwitchPanel from "./MapSwitchPanel";

interface ActionsProps {
  client: FoxgloveClientHandle;
}

/** AMCL と GNSS の入切、AMCL の初期化。ウェイポイントの on_reached_actions と同じ操作を手動で行う。 */
export function LocalizationActions({ client }: ActionsProps) {
  const gates = useLocalizationGates(client);
  const amclRunner = useActionRunner();
  const gnssRunner = useActionRunner();
  const resetRunner = useActionRunner();

  const setGate = (service: string, enable: boolean) => async () =>
    describeServiceResponse(
      await client.callService(service, { data: enable }),
    );

  return (
    <div className="space-y-3">
      <GateRow
        title="AMCL"
        view={describeAmclGate(gates.amcl)}
        busy={amclRunner.busy}
        result={amclRunner.result}
        onSet={(enable) =>
          void amclRunner.run(setGate(SERVICES.AMCL_GATE_WAYPOINT, enable))
        }
      />
      <p className="text-xs text-muted">
        Uses the same requester as the waypoint amcl_on / amcl_off actions, so
        the next waypoint action overrides it.
      </p>
      <GateRow
        title="GNSS"
        view={describeGnssGate(gates.gnssPublishing)}
        busy={gnssRunner.busy}
        result={gnssRunner.result}
        onSet={(enable) =>
          void gnssRunner.run(
            setGate(SERVICES.GNSS_CHANGE_PUBLISH_STATE, enable),
          )
        }
      />
      <div className="flex flex-wrap gap-2">
        <ActionButton
          label="Reset AMCL (global)"
          variant="yellow"
          disabled={resetRunner.busy}
          onClick={() =>
            void resetRunner.run(async () =>
              describeServiceResponse(
                await client.callService(
                  SERVICES.AMCL_REINITIALIZE_GLOBAL,
                  {},
                ),
              ),
            )
          }
        />
        <ActionButton
          label="Re-init AMCL from GNSS"
          variant="yellow"
          disabled={resetRunner.busy}
          onClick={() =>
            void resetRunner.run(async () =>
              describeServiceResponse(
                await client.callService(SERVICES.GNSS_AMCL_REINIT, {}),
              ),
            )
          }
        />
      </div>
      <ActionResultText result={resetRunner.result} />
    </div>
  );
}

/** map_list.txt の地図の切り替え */
export function MapActions({ client }: ActionsProps) {
  const maps = useNavigationMaps(client);
  return <MapSwitchPanel client={client} maps={maps} />;
}

function Section({ title, children }: { title: string; children: ReactNode }) {
  return <SectionCard title={title}>{children}</SectionCard>;
}

/** ウェイポイントの on_reached_actions と同じ操作を、手動で実行する(旧 UI の Actions)。 */
export default function WaypointActionsSection({ client }: ActionsProps) {
  return (
    <div className="space-y-2">
      <Section title="Localization">
        <LocalizationActions client={client} />
      </Section>
      <Section title="Map">
        <MapActions client={client} />
      </Section>
      <Section title="Call service">
        <GenericServiceCaller client={client} />
      </Section>
      <Section title="Publish topic">
        <GenericTopicPublisher client={client} />
      </Section>
    </div>
  );
}
