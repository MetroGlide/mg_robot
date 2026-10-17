import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useOpsValue } from "../../hooks/useOpsValue";
import { useSlamGnssMaps } from "../../hooks/useSlamGnssMaps";
import Card from "../ui/Card";
import OpsButton from "../ui/OpsButton";
import Pill from "../ui/Pill";

const INPUT_CLASS =
  "mt-1 block w-full rounded-md border border-line bg-surface-sunken px-2 py-1 font-mono text-xs text-content disabled:opacity-50";

/**
 * SLAM-GNSS-2D の地図の管理。実行中の SLAM の地図を保存し、保存済みの地図を選んで、
 * プレビューまたは再最適化を行う。プレビューと再最適化は同時に実行できない。
 */
export default function SlamGnssMapCard({
  sysManager,
  maps,
}: {
  sysManager: SystemManagerHandle;
  maps: ReturnType<typeof useSlamGnssMaps>;
}) {
  const [open, setOpen] = useOpsValue<boolean>("slamGnss.mapCard", false);
  const busy = maps.isPreviewing || maps.isReoptimizing;
  const state = sysManager.containers;
  const connected = Object.keys(state).length > 0;

  return (
    <Card className="w-[34rem] max-w-full p-4">
      <div className="flex items-center justify-between gap-2">
        <button
          type="button"
          aria-expanded={open}
          onClick={() => setOpen(!open)}
          className="flex flex-1 items-center gap-2 text-left text-sm font-bold"
        >
          SLAM-GNSS-2D の地図
          <span className="text-xs font-normal text-muted">{open ? "閉じる" : "開く"}</span>
        </button>
        <div className="flex gap-1">
          {maps.isPreviewing && <Pill tone="ok">プレビュー中</Pill>}
          {maps.isReoptimizing && <Pill tone="warn">再最適化中</Pill>}
          {!connected && <Pill tone="warn">system_manager に届いていません</Pill>}
        </div>
      </div>
      {open && (
        <div className="mt-3 max-h-[calc(100vh-14rem)] space-y-3 overflow-y-auto">
        <label className="block text-xs text-muted">
          保存先のディレクトリ
          <input
            type="text"
            value={maps.targetDirectory}
            onChange={(e) => maps.setTargetDirectory(e.target.value)}
            disabled={busy}
            className={INPUT_CLASS}
          />
        </label>
        <div className="flex gap-2">
          <OpsButton
            tone="primary"
            disabled={maps.isSaving || maps.isPreviewing}
            onClick={() => void maps.save()}
          >
            {maps.isSaving ? "保存しています…" : "SLAM 地図を保存"}
          </OpsButton>
          <OpsButton disabled={busy} onClick={() => void maps.reload()}>
            一覧を更新
          </OpsButton>
        </div>

        <div className="h-px bg-line" />

        <label className="block text-xs text-muted">
          プレビュー・再最適化する地図
          {maps.maps.length > 0 ? (
            <select
              value={maps.selectedMap}
              onChange={(e) => maps.setSelectedMap(e.target.value)}
              disabled={busy}
              className={INPUT_CLASS}
            >
              {maps.maps.map((m) => (
                <option key={m} value={m}>
                  {m}
                </option>
              ))}
            </select>
          ) : (
            <span className="mt-1 block text-xs italic">
              このディレクトリに保存済みの地図がありません
            </span>
          )}
        </label>
        <div className="flex gap-2">
          <OpsButton
            tone="primary"
            disabled={busy || !maps.selectedMap}
            onClick={() => void maps.startPreview()}
          >
            プレビュー開始
          </OpsButton>
          <OpsButton
            tone="danger"
            disabled={!maps.isPreviewing}
            onClick={() => void maps.stopPreview()}
          >
            プレビュー停止
          </OpsButton>
        </div>

        <div className="h-px bg-line" />

        <label className="block text-xs text-muted">
          再最適化に使う rosbag のパス
          <input
            type="text"
            value={maps.reoptBagPath}
            onChange={(e) => maps.setReoptBagPath(e.target.value)}
            disabled={busy}
            placeholder="/path/to/original_bag"
            className={INPUT_CLASS}
          />
        </label>
        <div className="flex gap-2">
          <OpsButton
            tone="warn"
            disabled={busy || !maps.selectedMap}
            onClick={() => void maps.startReoptimize()}
          >
            地図を再最適化
          </OpsButton>
          <OpsButton
            tone="danger"
            disabled={!maps.isReoptimizing}
            onClick={() => void maps.stopReoptimize()}
          >
            再最適化を停止
          </OpsButton>
        </div>
        </div>
      )}
    </Card>
  );
}
