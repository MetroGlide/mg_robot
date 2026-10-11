import { ReactNode } from "react";
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
  WheelIcon,
} from "../ui/icons";
import { useWheelMode } from "../../hooks/useWheelMode";
import { MapInteractionMode } from "../../hooks/useWaypointControl";
import { MapCommand } from "./MapCameraControls";
import OpacityControl from "./OpacityControl";

interface Props {
  onCommand: (kind: MapCommand["kind"]) => void;
  follow: boolean;
  onToggleFollow: () => void;
  /** 姿勢・ゴールの指定ボタンを出す場合に渡す */
  interactionMode?: MapInteractionMode;
  onInteractionModeChange?: (mode: MapInteractionMode) => void;
  disabled?: boolean;
  /** ユースケース固有のボタン(表示の切り替えなど)。区切りの下に並べる */
  extra?: ReactNode;
}

export default function MapToolbar({
  onCommand,
  follow,
  onToggleFollow,
  interactionMode,
  onInteractionModeChange,
  disabled = false,
  extra,
}: Props) {
  const [wheelMode, setWheelMode] = useWheelMode();
  const toggleMode = (mode: Exclude<MapInteractionMode, "none">) =>
    onInteractionModeChange?.(interactionMode === mode ? "none" : mode);

  return (
    <Card solid className="flex flex-col gap-1 p-1">
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
      <IconButton
        icon={<WheelIcon />}
        title={
          wheelMode === "zoom"
            ? "ホイール: 拡大縮小(押すと移動に切り替え)"
            : "ホイール: 移動(押すと拡大縮小に切り替え。マウス向け)"
        }
        active={wheelMode === "zoom"}
        onClick={() => setWheelMode(wheelMode === "zoom" ? "pan" : "zoom")}
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
      {extra && (
        <>
          <div className="my-0.5 h-px bg-line" />
          {extra}
        </>
      )}
      <div className="my-0.5 h-px bg-line" />
      <OpacityControl />
    </Card>
  );
}

/** 地図の操作を持たないビュー(衛星画像の表示中など)で、透明度のボタンだけを出すツールバー */
export function OpacityToolbar() {
  return (
    <Card solid className="p-1">
      <OpacityControl />
    </Card>
  );
}
