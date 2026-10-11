import { ReactNode } from "react";
import Card from "../ui/Card";
import IconButton from "../ui/IconButton";
import {
  CloseIcon,
  CrosshairIcon,
  FlagIcon,
  HomeIcon,
  MinusIcon,
  PlusIcon,
  PoseIcon,
  RotateIcon,
  ToolsIcon,
} from "../ui/icons";
import { useOpsValue } from "../../hooks/useOpsValue";
import { MapInteractionMode } from "../../hooks/useWaypointControl";
import { MapCommand } from "./MapCameraControls";
import OpacityControl from "./OpacityControl";

/** ツールバーの開閉の保存キー(ビューをまたいで共有する)。閉じている間はアイコン 1 つだけにして、右端のアイコンの列と重ならないようにする */
const TOOLBAR_OPEN_KEY = "toolbar.open";

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
  const toggleMode = (mode: Exclude<MapInteractionMode, "none">) =>
    onInteractionModeChange?.(interactionMode === mode ? "none" : mode);

  const [open, setOpen] = useOpsValue<boolean>(TOOLBAR_OPEN_KEY, true);

  if (!open) {
    return (
      <Card solid className="p-1">
        <IconButton
          icon={<ToolsIcon />}
          title="地図のツールを開く"
          onClick={() => setOpen(true)}
        />
      </Card>
    );
  }

  return (
    <Card solid className="grid grid-cols-2 gap-1 p-1">
      <IconButton icon={<CloseIcon />} title="ツールを閉じる" onClick={() => setOpen(false)} />
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
      {extra}
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
