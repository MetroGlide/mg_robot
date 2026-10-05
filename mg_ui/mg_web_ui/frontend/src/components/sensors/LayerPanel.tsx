import { useOpsValue } from "../../hooks/useOpsValue";
import { LayerKey, useVisualization } from "../../contexts/VisualizationContext";
import {
  PRESET_DESCRIPTIONS,
  PRESET_LABELS,
  VisualizationPreset,
} from "../../contexts/visualizationPresets";
import Card from "../ui/Card";

const PRESETS: VisualizationPreset[] = ["light", "standard", "full"];

interface LayerGroup {
  title: string;
  layers: { key: LayerKey; label: string; heavy?: boolean }[];
}

// heavy は点群や画像など、ブラウザと bridge の負荷が大きいもの
const GROUPS: LayerGroup[] = [
  {
    title: "地図・経路",
    layers: [
      { key: "map", label: "地図" },
      { key: "globalCostmap", label: "グローバルコストマップ", heavy: true },
      { key: "localCostmap", label: "ローカルコストマップ", heavy: true },
      { key: "planPath", label: "計画経路" },
      { key: "actualPath", label: "走行軌跡" },
      { key: "waypointMarkers", label: "ウェイポイント" },
    ],
  },
  {
    title: "ロボット・自己位置",
    layers: [
      { key: "robotPose", label: "ロボット" },
      { key: "particleCloud", label: "AMCL パーティクル" },
      { key: "collisionPolygons", label: "衝突検知エリア" },
    ],
  },
  {
    title: "センサ",
    layers: [
      { key: "lidarTop", label: "LiDAR (上)" },
      { key: "lidarFront", label: "LiDAR (前)" },
      { key: "pointCloud", label: "深度点群", heavy: true },
      { key: "colorImage", label: "カメラ画像", heavy: true },
      { key: "depthImage", label: "深度画像", heavy: true },
    ],
  },
];

/** センサビューのレイヤー切替。設定は旧 UI と共有される */
export default function LayerPanel() {
  const { layers, toggleLayer, applyPreset } = useVisualization();
  const [open, setOpen] = useOpsValue<boolean>("sensors.layerPanel", true);

  return (
    <Card className="w-64 p-3">
      <button
        type="button"
        onClick={() => setOpen(!open)}
        className="flex w-full items-center justify-between text-sm font-bold"
      >
        レイヤー
        <span className="text-xs text-muted">{open ? "閉じる" : "開く"}</span>
      </button>
      {open && (
        <div className="mt-2 space-y-3">
          <div className="flex gap-1">
            {PRESETS.map((preset) => (
              <button
                key={preset}
                type="button"
                title={PRESET_DESCRIPTIONS[preset]}
                onClick={() => applyPreset(preset)}
                className="flex-1 rounded-md bg-surface-sunken px-2 py-1 text-xs font-semibold hover:bg-line"
              >
                {PRESET_LABELS[preset]}
              </button>
            ))}
          </div>
          {GROUPS.map((group) => (
            <div key={group.title}>
              <p className="mb-1 text-[11px] font-semibold text-muted">
                {group.title}
              </p>
              <ul className="space-y-1">
                {group.layers.map(({ key, label, heavy }) => (
                  <li key={key}>
                    <label className="flex cursor-pointer items-center gap-2 text-xs">
                      <input
                        type="checkbox"
                        checked={layers[key]}
                        onChange={() => toggleLayer(key)}
                        className="accent-accent"
                      />
                      <span className="flex-1">{label}</span>
                      {heavy && (
                        <span className="text-[10px] text-warn">高負荷</span>
                      )}
                    </label>
                  </li>
                ))}
              </ul>
            </div>
          ))}
        </div>
      )}
    </Card>
  );
}
