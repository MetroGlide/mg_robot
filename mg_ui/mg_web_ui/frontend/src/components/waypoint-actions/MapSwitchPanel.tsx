import { useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useActionRunner } from "../../hooks/useActionRunner";
import { NavigationMaps } from "../../hooks/useNavigationMaps";
import { SERVICES } from "../../ros/services";
import {
  describeServiceResponse,
  mapDisplayName,
} from "../../utils/waypointActions";
import ActionButton from "../ui/ActionButton";
import LabeledValue from "../ui/LabeledValue";
import ActionResultText from "./ActionResultText";

const NO_CHANGE = "";

interface MapSelectProps {
  label: string;
  value: string;
  maps: NavigationMaps["maps"];
  onChange: (path: string) => void;
}

function MapSelect({ label, value, maps, onChange }: MapSelectProps) {
  return (
    <label className="block text-xs text-muted">
      {label}
      <select
        value={value}
        onChange={(e) => onChange(e.target.value)}
        className="mt-1 block w-full rounded bg-surface-sunken px-2 py-1 text-sm text-content"
      >
        <option value={NO_CHANGE}>(no change)</option>
        {maps.map((m) => (
          <option key={m.path} value={m.path} disabled={m.missing}>
            {m.missing ? `${m.name} (file missing)` : m.name}
          </option>
        ))}
      </select>
    </label>
  );
}

interface MapSwitchPanelProps {
  client: FoxgloveClientHandle;
  maps: NavigationMaps;
}

/** map_list.txt の地図を選んで map_server に読み込ませる。読み込み中の地図は sequencer の記録を表示する。 */
export default function MapSwitchPanel({ client, maps }: MapSwitchPanelProps) {
  const [localization, setLocalization] = useState(NO_CHANGE);
  const [planning, setPlanning] = useState(NO_CHANGE);
  const { busy, result, run } = useActionRunner();

  const handleLoad = () =>
    run(async () =>
      describeServiceResponse(
        await client.callService(SERVICES.WAYPOINT_LOAD_MAP, {
          localization,
          planning,
        }),
      ),
    );

  const loaded = maps.loaded;
  return (
    <div className="space-y-2">
      <div className="grid grid-cols-2 gap-2 text-sm">
        <div title={loaded?.localization}>
          <LabeledValue
            label="Loaded: localization"
            value={loaded ? mapDisplayName(loaded.localization) : "unknown"}
            variant={loaded ? "normal" : "muted"}
          />
        </div>
        <div title={loaded?.planning}>
          <LabeledValue
            label="Loaded: planning"
            value={loaded ? mapDisplayName(loaded.planning) : "unknown"}
            variant={loaded ? "normal" : "muted"}
          />
        </div>
      </div>

      {maps.error && <p className="text-xs text-error">{maps.error}</p>}
      {maps.skipped.length > 0 && (
        <p className="text-xs text-warn">
          Ignored entries in map_list.txt: {maps.skipped.join(", ")}
        </p>
      )}

      <div className="grid grid-cols-2 gap-2">
        <MapSelect
          label="Localization map"
          value={localization}
          maps={maps.maps}
          onChange={setLocalization}
        />
        <MapSelect
          label="Planning map"
          value={planning}
          maps={maps.maps}
          onChange={setPlanning}
        />
      </div>

      <div className="flex gap-2">
        <ActionButton
          label="Load maps"
          variant="blue"
          disabled={
            busy || (localization === NO_CHANGE && planning === NO_CHANGE)
          }
          onClick={handleLoad}
        />
        <ActionButton
          label="Reload list"
          variant="gray"
          disabled={maps.loading}
          onClick={() => void maps.reload()}
        />
      </div>
      <ActionResultText result={result} />
    </div>
  );
}
