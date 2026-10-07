import { useOpsValue } from "../../hooks/useOpsValue";
import { SlamOpacity, TileType } from "../ros-viewer/SatelliteOverlayViewer";
import Card from "../ui/Card";
import Disclosure from "../ui/Disclosure";

export interface SlamGnssLayers {
  poseGraphNodes: boolean;
  poseGraphSeqEdges: boolean;
  poseGraphLoopEdges: boolean;
  poseGraphGnssPrior: boolean;
  pathBefore: boolean;
}

export const DEFAULT_SLAM_GNSS_LAYERS: SlamGnssLayers = {
  poseGraphNodes: true,
  poseGraphSeqEdges: true,
  poseGraphLoopEdges: true,
  poseGraphGnssPrior: true,
  pathBefore: true,
};

export interface SatelliteSettings {
  enabled: boolean;
  tileType: TileType;
  slamOpacity: SlamOpacity;
}

export const DEFAULT_SATELLITE: SatelliteSettings = {
  enabled: false,
  tileType: "satellite",
  slamOpacity: 0.6,
};

const LAYER_ITEMS: { key: keyof SlamGnssLayers; label: string; color: string }[] = [
  { key: "poseGraphNodes", label: "グラフのノード", color: "#00bcd4" },
  { key: "poseGraphSeqEdges", label: "逐次エッジ", color: "#22c55e" },
  { key: "poseGraphLoopEdges", label: "ループエッジ", color: "#d946ef" },
  { key: "poseGraphGnssPrior", label: "GNSS の拘束", color: "#f59e0b" },
  { key: "pathBefore", label: "最適化前の経路", color: "#6b7280" },
];

const TILES: { value: TileType; label: string }[] = [
  { value: "osm", label: "OSM" },
  { value: "satellite", label: "衛星" },
];

const OPACITIES: SlamOpacity[] = [0.2, 0.4, 0.6, 0.8, 1.0];

function Segment({
  active,
  onClick,
  children,
}: {
  active: boolean;
  onClick: () => void;
  children: string;
}) {
  return (
    <button
      type="button"
      aria-pressed={active}
      onClick={onClick}
      className={`flex-1 rounded-md px-2 py-1 text-xs font-semibold ${
        active
          ? "bg-accent text-surface-elevated"
          : "bg-surface-elevated text-content hover:bg-line"
      }`}
    >
      {children}
    </button>
  );
}

/**
 * SLAM-GNSS-2D の表示の設定。ポーズグラフと経路のレイヤーの切り替えと、衛星画像への重ね表示。
 * 設定は画面の切り替えをまたいで保持する(衛星画像の重ね表示は、再読み込みでは切る。タイルの読み込みを避けるため)。
 */
export default function SlamGnssViewCard({
  layers,
  onLayersChange,
  satellite,
  onSatelliteChange,
}: {
  layers: SlamGnssLayers;
  onLayersChange: (next: SlamGnssLayers) => void;
  satellite: SatelliteSettings;
  onSatelliteChange: (next: SatelliteSettings) => void;
}) {
  const [open, setOpen] = useOpsValue<boolean>("slamGnss.viewCard", true);

  return (
    <Card className="w-64 p-3">
      <button
        type="button"
        aria-expanded={open}
        onClick={() => setOpen(!open)}
        className="flex w-full items-center justify-between text-sm font-bold"
      >
        表示
        <span className="text-xs text-muted">{open ? "閉じる" : "開く"}</span>
      </button>
      {open && (
        <div className="mt-2 max-h-[calc(100vh-14rem)] space-y-2 overflow-y-auto">
          <Disclosure title="レイヤー" storageKey="slamGnss.layers.open" defaultOpen>
            <div className="space-y-1">
              {LAYER_ITEMS.map(({ key, label, color }) => (
                <label key={key} className="flex cursor-pointer items-center gap-2 py-0.5 text-xs">
                  <input
                    type="checkbox"
                    checked={layers[key]}
                    onChange={(e) => onLayersChange({ ...layers, [key]: e.target.checked })}
                    disabled={satellite.enabled}
                    className="h-3.5 w-3.5 accent-accent"
                  />
                  <span className="h-2 w-2 rounded-full" style={{ backgroundColor: color }} />
                  {label}
                </label>
              ))}
              {satellite.enabled && (
                <p className="text-xs text-muted">衛星画像の重ね表示中は、切り替えられません。</p>
              )}
            </div>
          </Disclosure>
          <Disclosure title="衛星画像の重ね表示" storageKey="slamGnss.satellite.open">
            <div className="space-y-3">
              <label className="flex cursor-pointer items-center gap-2 text-xs">
                <input
                  type="checkbox"
                  checked={satellite.enabled}
                  onChange={(e) => onSatelliteChange({ ...satellite, enabled: e.target.checked })}
                  className="h-3.5 w-3.5 accent-accent"
                />
                重ね表示を使う(地図のタイルを読み込む)
              </label>
              <div>
                <p className="mb-1 text-xs text-muted">地図のタイル</p>
                <div className="flex gap-1">
                  {TILES.map((t) => (
                    <Segment
                      key={t.value}
                      active={satellite.tileType === t.value}
                      onClick={() => onSatelliteChange({ ...satellite, tileType: t.value })}
                    >
                      {t.label}
                    </Segment>
                  ))}
                </div>
              </div>
              <div>
                <p className="mb-1 text-xs text-muted">SLAM 地図の不透明度</p>
                <div className="flex gap-1">
                  {OPACITIES.map((op) => (
                    <Segment
                      key={op}
                      active={satellite.slamOpacity === op}
                      onClick={() => onSatelliteChange({ ...satellite, slamOpacity: op })}
                    >
                      {`${Math.round(op * 100)}%`}
                    </Segment>
                  ))}
                </div>
              </div>
              {satellite.enabled && (
                <p className="text-xs text-muted">
                  GPS で最初に取得した位置を、SLAM の原点として対応づけます。
                </p>
              )}
            </div>
          </Disclosure>
        </div>
      )}
    </Card>
  );
}
