import { useCallback, useEffect, useState } from "react";
import { SystemManagerHandle } from "./useSystemManagerClient";
import { useOpsValue } from "./useOpsValue";
import { getSysManagerUrl } from "../utils/systemManagerConfig";

const DEFAULT_DIRECTORY = "/root/ros2_data/slam_maps";

export interface SlamGnssNotice {
  text: string;
  ok: boolean;
}

function joinPath(dir: string, name: string): string {
  return dir.endsWith("/") ? `${dir}${name}` : `${dir}/${name}`;
}

function errorText(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

/**
 * SLAM-GNSS-2D の保存済み地図の一覧と、保存・プレビュー・再最適化の操作。
 * プレビューと再最適化の実行状態は、コンテナの実際の状態から求める。
 * 起動要求を送ってからコンテナが running になるまでの間は「起動中」として扱う。
 * 入力値は画面の切り替えをまたいで保持する。
 */
export function useSlamGnssMaps(sysManager: SystemManagerHandle) {
  const { callApi, containers } = sysManager;
  const [targetDirectory, setTargetDirectory] = useOpsValue(
    "slamGnss.targetDirectory",
    DEFAULT_DIRECTORY,
  );
  const [selectedMap, setSelectedMap] = useOpsValue("slamGnss.selectedMap", "");
  const [reoptBagPath, setReoptBagPath] = useOpsValue("slamGnss.reoptBagPath", "");
  const [maps, setMaps] = useState<string[]>([]);
  const [notice, setNotice] = useState<SlamGnssNotice | null>(null);
  const [isSaving, setIsSaving] = useState(false);
  const [previewStarting, setPreviewStarting] = useState(false);
  const [reoptStarting, setReoptStarting] = useState(false);

  const isPreviewing = previewStarting || containers["map-preview"] === "running";
  const isReoptimizing =
    reoptStarting || containers["reoptimize-slam"] === "running";

  const reload = useCallback(async () => {
    try {
      const r = await fetch(
        `${getSysManagerUrl()}/slam_gnss_2d/maps?base_dir=${encodeURIComponent(targetDirectory)}`,
      );
      const data = (await r.json()) as { success?: boolean; maps?: string[] };
      if (!data.success || !data.maps) return;
      setMaps(data.maps);
      if (!data.maps.includes(selectedMap)) setSelectedMap(data.maps[0] ?? "");
    } catch (e) {
      setNotice({ text: `地図の一覧を取得できません: ${errorText(e)}`, ok: false });
    }
  }, [targetDirectory, selectedMap, setSelectedMap]);

  useEffect(() => {
    void reload();
    // 一覧は保存先を変えたときだけ取り直す(選択の変更では取り直さない)
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [targetDirectory]);

  // 再最適化に使う bag の既定値は、.env の値を、未入力のときだけ入れる
  useEffect(() => {
    if (reoptBagPath) return;
    fetch(`${getSysManagerUrl()}/rosbag-replay/env`)
      .then((r) => r.json())
      .then((data: { file?: string }) => {
        if (data.file) setReoptBagPath(data.file);
      })
      .catch((e) =>
        setNotice({ text: `bag の既定値を取得できません: ${errorText(e)}`, ok: false }),
      );
    // 初回だけ取得する
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const run = useCallback(
    async (path: string, body: unknown, successText: string) => {
      try {
        const res = await callApi(path, body);
        setNotice({
          text: res.success ? successText : res.message,
          ok: res.success,
        });
        return res.success;
      } catch (e) {
        setNotice({ text: errorText(e), ok: false });
        return false;
      }
    },
    [callApi],
  );

  const save = useCallback(async () => {
    setIsSaving(true);
    const ok = await run(
      "/slam_gnss_2d/map/save",
      { output_dir: targetDirectory },
      "SLAM 地図を保存しました",
    );
    setIsSaving(false);
    if (ok) await reload();
  }, [run, targetDirectory, reload]);

  const startPreview = useCallback(async () => {
    if (!selectedMap) return;
    setPreviewStarting(true);
    await run(
      "/slam_gnss_2d/preview/start",
      { slam_map_path: joinPath(targetDirectory, selectedMap) },
      "プレビューを開始しました",
    );
    setPreviewStarting(false);
  }, [run, targetDirectory, selectedMap]);

  const stopPreview = useCallback(
    () => run("/slam_gnss_2d/preview/stop", {}, "プレビューを停止しました"),
    [run],
  );

  const startReoptimize = useCallback(async () => {
    if (!selectedMap) return;
    setReoptStarting(true);
    const fullPath = joinPath(targetDirectory, selectedMap);
    await run(
      "/slam_gnss_2d/reoptimize/start",
      { input_dir: fullPath, bag_path: reoptBagPath, save_dir: fullPath },
      "再最適化を開始しました",
    );
    setReoptStarting(false);
  }, [run, targetDirectory, selectedMap, reoptBagPath]);

  const stopReoptimize = useCallback(
    () => run("/slam_gnss_2d/reoptimize/stop", {}, "再最適化を停止しました"),
    [run],
  );

  return {
    targetDirectory,
    setTargetDirectory,
    selectedMap,
    setSelectedMap,
    reoptBagPath,
    setReoptBagPath,
    maps,
    notice,
    clearNotice: () => setNotice(null),
    isSaving,
    isPreviewing,
    isReoptimizing,
    reload,
    save,
    startPreview,
    stopPreview,
    startReoptimize,
    stopReoptimize,
  };
}
