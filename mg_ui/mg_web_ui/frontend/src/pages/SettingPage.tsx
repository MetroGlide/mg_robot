import { useEffect, useState } from "react";
import { useSimulation } from "../contexts/SimulationContext";
import { useRosbagReplay } from "../contexts/RosbagReplayContext";
import {
  useVisualization,
  LayerKey,
  OverlayKey,
  GpsMapSize,
} from "../contexts/VisualizationContext";
import { useTeleop } from "../contexts/TeleopContext";
import { ThemeMode, useTheme } from "../contexts/ThemeContext";
import { useRobotProfile } from "../contexts/RobotProfileContext";
import {
  PRESET_DESCRIPTIONS,
  PRESET_LABELS,
  VisualizationPreset,
} from "../contexts/visualizationPresets";
import ActionButton from "../components/ui/ActionButton";
import Toggle from "../components/ui/Toggle";
import {
  getSysManagerUrl,
  getSysManagerDefaultUrl,
  setSysManagerUrl,
  resetSysManagerUrl,
} from "../utils/systemManagerConfig";

const PRESETS: VisualizationPreset[] = ["light", "standard", "full"];

interface LayerGroup {
  label: string;
  items: { key: LayerKey; label: string }[];
}

const LAYER_GROUPS: LayerGroup[] = [
  {
    label: "Map",
    items: [
      { key: "map", label: "Map" },
      { key: "globalCostmap", label: "Global Costmap" },
      { key: "localCostmap", label: "Local Costmap" },
    ],
  },
  {
    label: "LiDAR",
    items: [
      { key: "lidarTop", label: "LiDAR" },
    ],
  },
  {
    label: "Robot",
    items: [
      { key: "robotPose", label: "Robot Pose (AMCL)" },
      { key: "particleCloud", label: "Particle Cloud" },
    ],
  },
  {
    label: "Navigation",
    items: [
      { key: "planPath", label: "Planned Path" },
      { key: "actualPath", label: "Actual Path" },
      { key: "waypointMarkers", label: "Waypoint Markers" },
      { key: "collisionPolygons", label: "Collision Polygons" },
    ],
  },
  {
    label: "Sensors",
    items: [
      { key: "pointCloud", label: "Point Cloud (Depth)" },
      { key: "colorImage", label: "Color Image" },
      { key: "depthImage", label: "Depth Image" },
    ],
  },
];

interface OverlayItem {
  key: OverlayKey;
  label: string;
  description: string;
}

const OVERLAY_CONFIG: OverlayItem[] = [
  {
    key: "joystick",
    label: "Joystick Pad",
    description: "操作パッド（選択式、デフォルトOFF）",
  },
  {
    key: "velocityGauge",
    label: "Velocity Gauge",
    description: "速度指令値・実測値メーター",
  },
  {
    key: "systemMetrics",
    label: "System Metrics",
    description: "CPU / メモリ使用率",
  },
  {
    key: "gpsStatus",
    label: "GPS Status",
    description: "GPS Fix状態・緯度・経度・高度",
  },
  {
    key: "gpsMap",
    label: "GPS Map",
    description: "OSMミニマップ・GPS軌跡表示",
  },
];

const THEME_MODES: { value: ThemeMode; label: string }[] = [
  { value: "light", label: "ライト" },
  { value: "dark", label: "ダーク" },
  { value: "system", label: "端末に合わせる" },
];

const TABS = [
  { id: "general", label: "全般" },
  { id: "connection", label: "Connection" },
  { id: "simulation", label: "Simulation" },
  { id: "rosbag-replay", label: "Rosbag Replay" },
  { id: "visualization", label: "Visualization" },
  { id: "overlays", label: "Viewer Overlays" },
  { id: "teleop", label: "Teleop" },
] as const;

type TabId = (typeof TABS)[number]["id"];

export default function SettingPage() {
  const { isSimulation, setIsSimulation } = useSimulation();
  const { isRosbagReplayVisible, setIsRosbagReplayVisible } = useRosbagReplay();
  const {
    enabled,
    layers,
    overlays,
    setEnabled,
    toggleLayer,
    toggleOverlay,
    gpsMapSize,
    setGpsMapSize,
    applyPreset,
  } = useVisualization();
  const {
    maxLinear,
    maxAngular,
    gaugeMaxLinear,
    gaugeMaxAngular,
    gaugeSyncWithPad,
    setMaxLinear,
    setMaxAngular,
    setGaugeMaxLinear,
    setGaugeMaxAngular,
    setGaugeSyncWithPad,
  } = useTeleop();

  const { mode: themeMode, setMode: setThemeMode, lowLoad, setLowLoad } = useTheme();

  const { name: robotName, setName: setRobotName } = useRobotProfile();
  const [robotNameDraft, setRobotNameDraft] = useState(robotName);
  useEffect(() => setRobotNameDraft(robotName), [robotName]);

  const commitRobotName = () => {
    const next = robotNameDraft.trim();
    if (next === "") setRobotNameDraft(robotName);
    else if (next !== robotName) setRobotName(next);
  };

  const [activeTab, setActiveTab] = useState<TabId>("general");

  const [sysManagerUrl, setSysManagerUrlState] = useState(() =>
    getSysManagerUrl(),
  );

  const handleSysManagerUrlBlur = () => {
    setSysManagerUrl(sysManagerUrl);
  };

  const handleSysManagerUrlReset = () => {
    resetSysManagerUrl();
    setSysManagerUrlState(getSysManagerDefaultUrl());
  };

  return (
    <div className="flex h-full">
      <nav className="w-40 flex-shrink-0 flex flex-col gap-1 p-2 border-r border-line">
        {TABS.map((tab) => (
          <button
            key={tab.id}
            onClick={() => setActiveTab(tab.id)}
            className={`text-left px-3 py-2 rounded text-sm transition-colors ${
              activeTab === tab.id
                ? "bg-surface-sunken text-content font-semibold"
                : "text-muted hover:text-content hover:bg-surface-sunken"
            }`}
          >
            {tab.label}
          </button>
        ))}
      </nav>

      <div className="flex-1 overflow-y-auto p-4">
        {activeTab === "general" && (
          <div className="space-y-4">
            <div>
              <label className="mb-1 block text-xs text-muted">
                ロボットの名前(全端末で共通。画面の見出しとタブの名前に使う)
              </label>
              <input
                value={robotNameDraft}
                onChange={(e) => setRobotNameDraft(e.target.value)}
                onBlur={commitRobotName}
                onKeyDown={(e) => {
                  if (e.key === "Enter") e.currentTarget.blur();
                }}
                className="w-full rounded border border-line bg-surface-sunken px-2 py-1.5 text-sm text-content"
              />
            </div>
            <div className="border-t border-line pt-3">
              <p className="mb-2 text-xs text-muted">テーマ(この端末だけの設定)</p>
              <div className="flex gap-2">
                {THEME_MODES.map(({ value, label }) => (
                  <button
                    key={value}
                    onClick={() => setThemeMode(value)}
                    className={`flex-1 rounded border px-2 py-2 text-xs transition-colors ${
                      themeMode === value
                        ? "border-accent bg-accent text-surface-elevated"
                        : "border-line bg-surface-sunken text-content hover:bg-line"
                    }`}
                  >
                    {label}
                  </button>
                ))}
              </div>
            </div>
            <div className="flex items-center gap-4 border-t border-line pt-3">
              <span className="text-sm text-content">低負荷モード</span>
              <Toggle value={lowLoad} onChange={() => setLowLoad(!lowLoad)} />
              <span
                className={`text-sm font-semibold ${lowLoad ? "text-accent" : "text-muted"}`}
              >
                {lowLoad ? "ON" : "OFF"}
              </span>
            </div>
            <p className="text-xs text-muted">
              影とアニメーションを切ります(この端末だけの設定)。
            </p>
          </div>
        )}

        {activeTab === "connection" && (
          <div className="space-y-3">
            <div>
              <label className="text-xs text-muted block mb-1">
                System Manager URL
              </label>
              <div className="flex gap-2">
                <input
                  type="text"
                  value={sysManagerUrl}
                  onChange={(e) => setSysManagerUrlState(e.target.value)}
                  onBlur={handleSysManagerUrlBlur}
                  className="flex-1 bg-surface-sunken text-sm text-content px-2 py-1 rounded border border-line focus:outline-none focus:border-accent"
                />
                <button
                  onClick={handleSysManagerUrlReset}
                  className="px-3 py-1 rounded text-sm font-medium bg-surface-sunken hover:bg-line text-content flex-shrink-0"
                >
                  Reset
                </button>
              </div>
              <p className="text-xs text-muted mt-1">
                Default: {getSysManagerDefaultUrl()}
              </p>
            </div>
          </div>
        )}

        {activeTab === "simulation" && (
          <div className="space-y-4">
            <div className="flex items-center gap-4">
              <span className="text-sm text-content">Simulation Mode</span>
              <Toggle
                value={isSimulation}
                onChange={() => setIsSimulation(!isSimulation)}
              />
              <span
                className={`text-sm font-semibold ${isSimulation ? "text-accent" : "text-muted"}`}
              >
                {isSimulation ? "ON" : "OFF"}
              </span>
            </div>
            <p className="text-xs text-muted">
              Enable to show simulation-related features on each page.
            </p>
          </div>
        )}

        {activeTab === "rosbag-replay" && (
          <div className="space-y-4">
            <div className="flex items-center gap-4">
              <span className="text-sm text-content">Rosbag Replay</span>
              <Toggle
                value={isRosbagReplayVisible}
                onChange={() =>
                  setIsRosbagReplayVisible(!isRosbagReplayVisible)
                }
              />
              <span
                className={`text-sm font-semibold ${isRosbagReplayVisible ? "text-accent" : "text-muted"}`}
              >
                {isRosbagReplayVisible ? "ON" : "OFF"}
              </span>
            </div>
            <p className="text-xs text-muted">
              Enable to show rosbag replay controls on each page.
            </p>
          </div>
        )}

        {activeTab === "visualization" && (
          <div className="space-y-4">
            <div className="flex items-center gap-4">
              <span className="text-sm text-content">
                Enable Visualization
              </span>
              <Toggle value={enabled} onChange={() => setEnabled(!enabled)} />
              <span
                className={`text-sm font-semibold ${enabled ? "text-accent" : "text-muted"}`}
              >
                {enabled ? "ON" : "OFF"}
              </span>
            </div>
            <p className="text-xs text-muted">
              Disable to stop all visualization topic subscriptions.
            </p>

            {enabled && (
              <div className="space-y-2 pt-2 border-t border-line">
                <p className="text-xs text-muted">
                  Preset（レイヤーとオーバーレイをまとめて切り替えます）
                </p>
                <div className="flex gap-2">
                  {PRESETS.map((preset) => (
                    <ActionButton
                      key={preset}
                      label={PRESET_LABELS[preset]}
                      size="sm"
                      onClick={() => applyPreset(preset)}
                    />
                  ))}
                </div>
                <ul className="text-xs text-muted space-y-0.5">
                  {PRESETS.map((preset) => (
                    <li key={preset}>
                      {PRESET_LABELS[preset]}: {PRESET_DESCRIPTIONS[preset]}
                    </li>
                  ))}
                </ul>
              </div>
            )}

            {enabled && (
              <div className="space-y-4 pt-2 border-t border-line">
                {LAYER_GROUPS.map((group) => (
                  <div key={group.label}>
                    <p className="text-xs text-muted mb-2">{group.label}</p>
                    <div className="space-y-2">
                      {group.items.map(({ key, label }) => (
                        <div key={key} className="flex items-center gap-3">
                          <Toggle
                            value={layers[key]}
                            onChange={() => toggleLayer(key)}
                          />
                          <span className="text-sm text-content">{label}</span>
                        </div>
                      ))}
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>
        )}

        {activeTab === "overlays" && (
          <div className="space-y-4">
            <p className="text-xs text-muted">
              可視化領域に重ねて表示する要素を設定します。
            </p>
            <div className="space-y-2">
              {OVERLAY_CONFIG.map(({ key, label, description }) => (
                <div key={key} className="flex items-center gap-3">
                  <Toggle
                    value={overlays[key]}
                    onChange={() => toggleOverlay(key)}
                  />
                  <div>
                    <span className="text-sm text-content">{label}</span>
                    <p className="text-xs text-muted">{description}</p>
                  </div>
                </div>
              ))}
            </div>
            {overlays.gpsMap && (
              <div className="pt-3 border-t border-line">
                <p className="text-xs text-muted mb-2">GPS Map サイズ</p>
                <div className="flex gap-2">
                  {(
                    [
                      { value: "default", label: "標準", sub: "192×192" },
                      {
                        value: "2x-square",
                        label: "2倍 正方形",
                        sub: "384×384",
                      },
                      { value: "2x-wide", label: "2倍 横長", sub: "384×192" },
                    ] as { value: GpsMapSize; label: string; sub: string }[]
                  ).map(({ value, label, sub }) => (
                    <button
                      key={value}
                      onClick={() => setGpsMapSize(value)}
                      className={`flex-1 px-2 py-2 rounded text-xs border transition-colors ${
                        gpsMapSize === value
                          ? "bg-accent border-accent text-surface-elevated"
                          : "bg-surface-sunken border-line text-content hover:bg-line"
                      }`}
                    >
                      <div className="font-medium">{label}</div>
                      <div className="text-muted mt-0.5">{sub}</div>
                    </button>
                  ))}
                </div>
              </div>
            )}
          </div>
        )}

        {activeTab === "teleop" && (
          <div className="space-y-4">
            <div>
              <p className="text-xs text-muted mb-2">Pad speed limit</p>
              <div className="grid grid-cols-2 gap-4">
                <label className="flex flex-col gap-1">
                  <span className="text-sm text-content">
                    Max Linear (m/s)
                  </span>
                  <input
                    type="number"
                    min={0.1}
                    max={2.0}
                    step={0.1}
                    value={maxLinear}
                    onChange={(e) => setMaxLinear(Number(e.target.value))}
                    className="w-full bg-surface-sunken rounded px-2 py-1 text-sm"
                  />
                </label>
                <label className="flex flex-col gap-1">
                  <span className="text-sm text-content">
                    Max Angular (rad/s)
                  </span>
                  <input
                    type="number"
                    min={0.1}
                    max={2.0}
                    step={0.1}
                    value={maxAngular}
                    onChange={(e) => setMaxAngular(Number(e.target.value))}
                    className="w-full bg-surface-sunken rounded px-2 py-1 text-sm"
                  />
                </label>
              </div>
            </div>
            <div className="pt-3 border-t border-line space-y-3">
              <div className="flex items-center gap-3">
                <Toggle
                  value={gaugeSyncWithPad}
                  onChange={() => setGaugeSyncWithPad(!gaugeSyncWithPad)}
                />
                <div>
                  <span className="text-sm text-content">
                    Gauge: Sync with pad limits
                  </span>
                  <p className="text-xs text-muted">
                    When ON, gauge display limit equals pad speed limit.
                  </p>
                </div>
              </div>
              {!gaugeSyncWithPad && (
                <div>
                  <p className="text-xs text-muted mb-2">
                    Gauge display limit
                  </p>
                  <div className="grid grid-cols-2 gap-4">
                    <label className="flex flex-col gap-1">
                      <span className="text-sm text-content">
                        Max Linear (m/s)
                      </span>
                      <input
                        type="number"
                        min={0.1}
                        max={5.0}
                        step={0.1}
                        value={gaugeMaxLinear}
                        onChange={(e) =>
                          setGaugeMaxLinear(Number(e.target.value))
                        }
                        className="w-full bg-surface-sunken rounded px-2 py-1 text-sm"
                      />
                    </label>
                    <label className="flex flex-col gap-1">
                      <span className="text-sm text-content">
                        Max Angular (rad/s)
                      </span>
                      <input
                        type="number"
                        min={0.1}
                        max={5.0}
                        step={0.1}
                        value={gaugeMaxAngular}
                        onChange={(e) =>
                          setGaugeMaxAngular(Number(e.target.value))
                        }
                        className="w-full bg-surface-sunken rounded px-2 py-1 text-sm"
                      />
                    </label>
                  </div>
                </div>
              )}
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
