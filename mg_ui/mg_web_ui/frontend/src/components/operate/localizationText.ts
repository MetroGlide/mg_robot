import { LOCALIZATION_CHECK_STATUS, LocalizationStatus } from "../../types";

/**
 * 自己位置の状態の補足を 1 行にする。判定はせず、自己位置監視が決めた値を並べるだけ。
 * 疑わしい情報源 (blame)、異常の判定の名前、自己位置状況・速度の上限・センサ構成 (あるとき)、
 * 計画用地図の障害物の中にいるか (と食い込みの深さ)。
 */
export function localizationDetail(status: LocalizationStatus): string {
  const parts: string[] = [];
  if (status.blame !== "") {
    parts.push(`疑義: ${status.blame}`);
  }
  const anomalies = status.checks
    .filter((check) => check.status === LOCALIZATION_CHECK_STATUS.ANOMALY)
    .map((check) => check.name);
  if (anomalies.length > 0) {
    parts.push(anomalies.join(", "));
  }
  if (status.situation !== "" && status.situation !== "normal") {
    parts.push(`状況: ${status.situation}`);
  }
  if (status.speed_limit > 0) {
    parts.push(`上限 ${status.speed_limit.toFixed(2)} m/s`);
  }
  if (status.sensor_set !== "") {
    parts.push(`センサ: ${status.sensor_set}`);
  }
  if (status.footprint_in_lethal) {
    parts.push(
      status.lethal_depth_m >= 0 ? `障害物内 ${status.lethal_depth_m.toFixed(2)} m` : "障害物内",
    );
  }
  return parts.join(" · ");
}
