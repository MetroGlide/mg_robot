import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useVisualization } from "../../contexts/VisualizationContext";
import JoystickPad from "../panels/JoystickPad";
import Card from "../ui/Card";
import IconButton from "../ui/IconButton";
import { GlobeIcon, JoystickIcon, LayersIcon } from "../ui/icons";
import { useSensorsVisible } from "../../hooks/useSensorsVisible";
import GpsMiniMap from "./GpsMiniMap";

/**
 * 地図ツールバーに置く、センサ表示・ジョイスティック・GPS のミニ地図の表示の切り替え。
 * センサ表示は、センサビューのレイヤーパネルで ON にした内容を、まとめて地図に重ねる(購読もこのときだけ)。
 * 表示の有無は設定(VisualizationContext の overlays)に保存し、端末をまたいで共有する。
 */
export function OverlayToggleButtons({ disabled = false }: { disabled?: boolean }) {
  const { overlays, toggleOverlay } = useVisualization();
  const [sensors, setSensors] = useSensorsVisible();
  return (
    <>
      <IconButton
        icon={<LayersIcon />}
        title="センサ表示をまとめて切り替え(内容はセンサビューのレイヤーで選ぶ)"
        active={sensors}
        onClick={() => setSensors(!sensors)}
      />
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

/** 表示を有効にした GPS のミニ地図。無効なら何も描かない */
export function OverlayCards({ client }: { client: FoxgloveClientHandle }) {
  const { overlays } = useVisualization();
  return overlays.gpsMap ? <GpsMiniMap client={client} /> : null;
}

/** 表示を有効にしたジョイスティック。右下の診断カードの上に置く */
export function JoystickCard({ client }: { client: FoxgloveClientHandle }) {
  const { overlays } = useVisualization();
  return overlays.joystick ? (
    <Card className="p-2">
      <JoystickPad client={client} />
    </Card>
  ) : null;
}
