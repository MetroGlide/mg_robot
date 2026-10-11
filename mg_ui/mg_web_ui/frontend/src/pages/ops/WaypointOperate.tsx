import { useEffect, useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useOpsValue } from "../../hooks/useOpsValue";
import { useSensorsVisible } from "../../hooks/useSensorsVisible";
import { useWaypointControl } from "../../hooks/useWaypointControl";
import { GoalBtMode } from "../../utils/waypointActions";
import Card from "../../components/ui/Card";
import OperateLayout from "../../components/operate/OperateLayout";
import OperateMap from "../../components/operate/OperateMap";
import KpiRow from "../../components/operate/KpiRow";
import MapToolbar from "../../components/operate/MapToolbar";
import DiagnosticsBoard from "../../components/operate/DiagnosticsBoard";
import RobotDetailCard from "../../components/operate/RobotDetailCard";
import WaypointProgress from "../../components/operate/WaypointProgress";
import ActionsCard from "../../components/operate/ActionsCard";
import { MapCommand } from "../../components/operate/MapCameraControls";
import GnssDetailCard from "../../components/operate/GnssDetailCard";
import TogglePanel from "../../components/operate/TogglePanel";
import {
  JoystickCard,
  OverlayCards,
  OverlayToggleButtons,
} from "../../components/operate/MapOverlays";

const VIEW_KEY = "waypoint";

/**
 * Waypoint 走行の運用ビュー。
 * 追従・カード・タブの開閉と選択・入力値は、他の画面に移って戻っても保持する(useOpsValue)。
 * 走行中は操作しない運用を想定し、状態の表示は 1 画面に収める(カードは閉じられる)。
 */
export default function WaypointOperate({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [command, setCommand] = useState<MapCommand | null>(null);
  const [follow, setFollow] = useOpsValue<boolean>(`${VIEW_KEY}.follow`, false);
  const [goalBt, setGoalBt] = useOpsValue<GoalBtMode>(
    `${VIEW_KEY}.goalBt`,
    "default",
  );
  const [savedCountdownMs, setSavedCountdownMs] = useOpsValue<number>(
    `${VIEW_KEY}.countdownMs`,
    3000,
  );
  const [sensorsVisible] = useSensorsVisible();
  const [mapError, setMapError] = useState<string | null>(null);
  const control = useWaypointControl(
    client,
    goalBt,
    setMapError,
    savedCountdownMs,
  );
  const connected = client.status === "connected";

  const { countdownMs } = control;
  useEffect(() => {
    setSavedCountdownMs(countdownMs);
  }, [countdownMs, setSavedCountdownMs]);

  const issueCommand = (kind: MapCommand["kind"]) =>
    setCommand((prev) => ({ kind, n: (prev?.n ?? 0) + 1 }));

  const goalResult = control.goalRunner.result;
  const notice = mapError ?? goalResult?.text ?? null;
  const noticeIsError = mapError !== null || goalResult?.ok === false;

  return (
    <OperateLayout
      map={
        <OperateMap
          viewKey={VIEW_KEY}
          client={client}
          command={command}
          follow={follow}
          onUserPan={() => setFollow(false)}
          interactionMode={control.interactionMode}
          onPoseSet={control.handleMapPoseSet}
          showSensors={sensorsVisible}
        />
      }
      topLeft={
        <div className="flex flex-col items-start gap-2">
          <KpiRow client={client} />
          <TogglePanel id="actions">
            {(close) => (
              <ActionsCard
                client={client}
                sysManager={sysManager}
                control={control}
                goalBt={goalBt}
                onGoalBtChange={setGoalBt}
                connected={connected}
                onClose={close}
              />
            )}
          </TogglePanel>
        </div>
      }
      toolbar={
        <MapToolbar
          onCommand={issueCommand}
          follow={follow}
          onToggleFollow={() => setFollow(!follow)}
          interactionMode={control.interactionMode}
          onInteractionModeChange={control.setInteractionMode}
          disabled={!connected}
          extra={<OverlayToggleButtons disabled={!connected} />}
        />
      }
      topRight={
        <div className="flex flex-col items-end gap-2">
          <TogglePanel id="robotDetail">
            {(close) => <RobotDetailCard client={client} onClose={close} />}
          </TogglePanel>
          <TogglePanel id="gnssDetail">
            {(close) => <GnssDetailCard client={client} onClose={close} />}
          </TogglePanel>
        </div>
      }
      notice={
        notice && (
          <Card solid className="px-3 py-1.5">
            <p className={`text-xs ${noticeIsError ? "text-error" : "text-ok"}`}>
              {notice}
            </p>
          </Card>
        )
      }
      bottomLeft={
        <div className="flex flex-col items-start gap-2">
          <OverlayCards client={client} />
          <TogglePanel id="waypointProgress">
            {(close) => (
              <WaypointProgress
                client={client}
                control={control}
                connected={connected}
                onClose={close}
              />
            )}
          </TogglePanel>
        </div>
      }
      bottomRight={
        <div className="flex flex-col items-end gap-2">
          <JoystickCard client={client} />
          <TogglePanel id="diagnostics">
            {(close) => <DiagnosticsBoard client={client} onClose={close} />}
          </TogglePanel>
        </div>
      }
    />
  );
}
