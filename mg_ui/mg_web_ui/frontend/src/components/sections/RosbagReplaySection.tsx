import { useState, useEffect } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useServiceCaller } from "../../hooks/useServiceCaller";
import { SERVICES } from "../../ros/services";
import { loadSettings, saveSettings } from "../../utils/settingsApi";
import { getSysManagerUrl } from "../../utils/systemManagerConfig";
import { containerDisplay } from "../operate/ContainerControl";
import OpsButton from "../ui/OpsButton";
import Pill from "../ui/Pill";
import ValueConfirmDialog from "../ui/ValueConfirmDialog";

const RATE_OPTIONS = [0.5, 1.0, 1.5, 2.0] as const;

const SETTINGS_KEY = "rosbagReplayInput";

const INPUT_CLASS =
  "mt-1 block w-full rounded-md border border-line bg-surface-elevated px-2 py-1 text-xs text-content";

/** rosbag 再生コンテナの起動・停止と、再生の一時停止・速度の変更。枠は呼び出し側で付ける */
export default function RosbagReplaySection({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [file, setFile] = useState("");
  const [topics, setTopics] = useState("");
  const [rate, setRate] = useState(1.0);
  const [isPaused, setIsPaused] = useState(false);
  const [sysLoading, setSysLoading] = useState(false);
  const [sysError, setSysError] = useState<string | null>(null);
  const [dialogOpen, setDialogOpen] = useState(false);
  const [pendingFile, setPendingFile] = useState("");
  const [pendingTopics, setPendingTopics] = useState("");

  const {
    call: callRos,
    loading: rosLoading,
    error: rosError,
  } = useServiceCaller(client);

  const containerState = sysManager.containers["rosbag-replay"];
  const isRunning = containerState === "running";
  const display = containerDisplay(containerState);

  useEffect(() => {
    loadSettings().then((data) => {
      const s = data[SETTINGS_KEY] as
        | { file?: string; topics?: string }
        | undefined;
      if (s?.file !== undefined) setFile(s.file);
      if (s?.topics !== undefined) setTopics(s.topics);
    });
  }, []);

  const persistInputs = (nextFile: string, nextTopics: string) => {
    saveSettings(SETTINGS_KEY, { file: nextFile, topics: nextTopics });
  };

  const callSys = async (path: string, body?: unknown) => {
    setSysLoading(true);
    setSysError(null);
    try {
      const result = await sysManager.callApi(path, body);
      if (!result.success) setSysError(result.message);
    } catch (e) {
      setSysError(e instanceof Error ? e.message : String(e));
    } finally {
      setSysLoading(false);
    }
  };

  const handleStart = () => {
    const topicList = topics
      .split(",")
      .map((t) => t.trim())
      .filter((t) => t.length > 0);
    callSys("/rosbag-replay/start", { file, topics: topicList });
    setIsPaused(false);
  };

  const handleStop = () => {
    callSys("/rosbag-replay/stop");
    setIsPaused(false);
  };

  const handlePause = async () => {
    await callRos(SERVICES.ROSBAG_PAUSE, {});
    setIsPaused(true);
  };

  const handleResume = async () => {
    await callRos(SERVICES.ROSBAG_RESUME, {});
    setIsPaused(false);
  };

  const handleSetRate = async (r: number) => {
    await callRos(SERVICES.ROSBAG_SET_RATE, { rate: r });
    setRate(r);
  };

  const handleLoadFromEnv = async () => {
    setSysLoading(true);
    setSysError(null);
    try {
      const res = await fetch(`${getSysManagerUrl()}/rosbag-replay/env`);
      if (!res.ok) {
        throw new Error(`Failed to load rosbag replay env: ${res.status} ${res.statusText}`);
      }
      const data: unknown = await res.json();
      const payload =
        typeof data === "object" && data !== null
          ? (data as { file?: unknown; topics?: unknown })
          : {};
      const envFile = typeof payload.file === "string" ? payload.file : "";
      const envTopics = Array.isArray(payload.topics)
        ? payload.topics.filter((topic): topic is string => typeof topic === "string").join(", ")
        : "";
      setPendingFile(envFile);
      setPendingTopics(envTopics);
      setDialogOpen(true);
    } catch (e) {
      setSysError(e instanceof Error ? e.message : String(e));
    } finally {
      setSysLoading(false);
    }
  };

  const handleDialogConfirm = () => {
    setFile(pendingFile);
    setTopics(pendingTopics);
    persistInputs(pendingFile, pendingTopics);
    setDialogOpen(false);
  };

  return (
    <div className="space-y-3">
      <div className="flex items-center justify-between gap-2">
        <span className="text-xs font-semibold">Rosbag 再生</span>
        <Pill tone={display.tone}>{display.label}</Pill>
      </div>

      <div className="space-y-2">
        <label className="block text-xs text-muted">
          ファイル (コンテナ内の絶対パス)
          <input
            type="text"
            value={file}
            onChange={(e) => setFile(e.target.value)}
            onBlur={() => persistInputs(file, topics)}
            placeholder="/root/ros2_data/example.bag"
            className={INPUT_CLASS}
          />
        </label>
        <label className="block text-xs text-muted">
          トピック (カンマ区切り、空なら全部)
          <input
            type="text"
            value={topics}
            onChange={(e) => setTopics(e.target.value)}
            onBlur={() => persistInputs(file, topics)}
            placeholder="/scan, /odom, /tf"
            className={INPUT_CLASS}
          />
        </label>
        <div className="flex flex-wrap gap-2">
          <OpsButton tone="primary" disabled={sysLoading || !file} onClick={handleStart}>
            開始
          </OpsButton>
          <OpsButton tone="danger" disabled={sysLoading} onClick={handleStop}>
            停止
          </OpsButton>
          <OpsButton disabled={sysLoading} onClick={handleLoadFromEnv}>
            .env から読む
          </OpsButton>
        </div>
        {sysError && <p className="text-xs text-error">{sysError}</p>}
      </div>

      <div className="space-y-2">
        <div className="flex gap-2">
          <OpsButton
            tone="warn"
            disabled={!isRunning || isPaused || rosLoading}
            onClick={handlePause}
          >
            一時停止
          </OpsButton>
          <OpsButton
            tone="primary"
            disabled={!isRunning || !isPaused || rosLoading}
            onClick={handleResume}
          >
            再開
          </OpsButton>
        </div>
        <div>
          <p className="mb-1 text-xs text-muted">再生速度</p>
          <div className="flex gap-1">
            {RATE_OPTIONS.map((r) => (
              <button
                key={r}
                type="button"
                onClick={() => handleSetRate(r)}
                disabled={!isRunning || rosLoading}
                className={`rounded-md px-3 py-1 text-xs font-semibold disabled:opacity-40 ${
                  rate === r
                    ? "bg-accent text-surface-elevated"
                    : "bg-surface-elevated text-content hover:bg-line"
                }`}
              >
                {r}x
              </button>
            ))}
          </div>
        </div>
        {rosError && <p className="text-xs text-error">{rosError}</p>}
      </div>

      <ValueConfirmDialog
        open={dialogOpen}
        title=".env の値で上書きしますか？"
        values={[
          { label: "ファイル", value: pendingFile },
          { label: "トピック", value: pendingTopics },
        ]}
        onConfirm={handleDialogConfirm}
        onCancel={() => setDialogOpen(false)}
      />
    </div>
  );
}
