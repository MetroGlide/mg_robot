import Card from "../ui/Card";
import IconButton from "../ui/IconButton";
import {
  CrosshairIcon,
  FlagIcon,
  HomeIcon,
  MinusIcon,
  PlusIcon,
  PoseIcon,
  RotateIcon,
} from "../ui/icons";
import { MapInteractionMode } from "../../hooks/useWaypointControl";
import { MapCommand } from "./MapCameraControls";

interface Props {
  onCommand: (kind: MapCommand["kind"]) => void;
  follow: boolean;
  onToggleFollow: () => void;
  /** 姿勢・ゴールの指定ボタンを出す場合に渡す */
  interactionMode?: MapInteractionMode;
  onInteractionModeChange?: (mode: MapInteractionMode) => void;
  disabled?: boolean;
}

export default function MapToolbar({
  onCommand,
  follow,
  onToggleFollow,
  interactionMode,
  onInteractionModeChange,
  disabled = false,
}: Props) {
  const toggleMode = (mode: Exclude<MapInteractionMode, "none">) =>
    onInteractionModeChange?.(interactionMode === mode ? "none" : mode);

  return (
    <Card className="flex flex-col gap-1 p-1">
      <IconButton icon={<PlusIcon />} title="拡大" onClick={() => onCommand("zoomIn")} />
      <IconButton icon={<MinusIcon />} title="縮小" onClick={() => onCommand("zoomOut")} />
      <IconButton icon={<RotateIcon />} title="90度回転" onClick={() => onCommand("rotate")} />
      <IconButton icon={<HomeIcon />} title="表示を初期位置に戻す" onClick={() => onCommand("reset")} />
      <IconButton
        icon={<CrosshairIcon />}
        title="ロボットを追従"
        active={follow}
        onClick={onToggleFollow}
      />
      {onInteractionModeChange && (
        <>
          <div className="my-0.5 h-px bg-line" />
          <IconButton
            icon={<PoseIcon />}
            title="初期姿勢を指定(地図をドラッグ)"
            active={interactionMode === "pose_estimate"}
            disabled={disabled}
            onClick={() => toggleMode("pose_estimate")}
          />
          <IconButton
            icon={<FlagIcon />}
            title="ゴールを指定(地図をドラッグ)"
            active={interactionMode === "nav_goal"}
            disabled={disabled}
            onClick={() => toggleMode("nav_goal")}
          />
        </>
      )}
    </Card>
  );
}
