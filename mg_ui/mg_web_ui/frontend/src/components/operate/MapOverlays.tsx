import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useVisualization } from "../../contexts/VisualizationContext";
import JoystickPad from "../panels/JoystickPad";
import Card from "../ui/Card";
import IconButton from "../ui/IconButton";
import { GlobeIcon, JoystickIcon } from "../ui/icons";
import GpsMiniMap from "./GpsMiniMap";

/**
 * 地図ツールバーに置く、ジョイスティックと GPS のミニ地図の表示の切り替え。
 * 表示の有無は設定(VisualizationContext の overlays)に保存し、端末をまたいで共有する。
 */
export function OverlayToggleButtons({ disabled = false }: { disabled?: boolean }) {
  const { overlays, toggleOverlay } = useVisualization();
  return (
    <>
      <IconButton
        icon={<JoystickIcon />}
        title="ジョイスティックを表示"
        active={overlays.joystick}
        disabled={disabled}
        onClick={() => toggleOverlay("joystick")}
      />
      <IconButton
        icon={<GlobeIcon />}
        title="GPS のミニ地図を表示(衛星画像を読み込む)"
        active={overlays.gpsMap}
        onClick={() => toggleOverlay("gpsMap")}
      />
    </>
  );
}

/** 表示を有効にしたジョイスティックと GPS のミニ地図。どちらも無効なら何も描かない */
export function OverlayCards({ client }: { client: FoxgloveClientHandle }) {
  const { overlays } = useVisualization();
  return (
    <>
      {overlays.gpsMap && <GpsMiniMap client={client} />}
      {overlays.joystick && (
        <Card className="p-2">
          <JoystickPad client={client} />
        </Card>
      )}
    </>
  );
}
