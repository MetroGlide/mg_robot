import proj4 from "proj4";
import type { NavPvt, NavSatFix } from "../types";

export type GnssTone = "ok" | "warn" | "error" | "neutral";
export type GnssSource = "navpvt" | "navsatfix";

/** 購読元によらず同じ形で扱う GNSS の測位結果。購読元にない値は null */
export interface GnssReading {
  source: GnssSource;
  /** 測位の種類の表示名(RTK Fixed、3D Fix など) */
  label: string;
  tone: GnssTone;
  lat: number | null;
  lon: number | null;
  /** 楕円体高 [m] */
  heightEllipsoid: number | null;
  /** 平均海面からの高さ [m] */
  heightMsl: number | null;
  /** 水平方向の精度 [m] */
  hAcc: number | null;
  /** 垂直方向の精度 [m] */
  vAcc: number | null;
  pDop: number | null;
  numSv: number | null;
}

// NavPVT の flags のうち、搬送波位相による解の状態(bit 6-7)
const CARRIER_PHASE_SHIFT = 6;
const CARRIER_PHASE_MASK = 0b11;
const CARRIER_PHASE_FLOAT = 1;
const CARRIER_PHASE_FIXED = 2;
const FLAGS_GNSS_FIX_OK = 1;

const MM = 1e-3;
const LAT_LON_UNIT = 1e-7;
const P_DOP_UNIT = 0.01;

const FIX_TYPE_LABEL: Record<number, { label: string; tone: GnssTone }> = {
  0: { label: "No Fix", tone: "error" },
  1: { label: "DR のみ", tone: "warn" },
  2: { label: "2D Fix", tone: "warn" },
  3: { label: "3D Fix", tone: "warn" },
  4: { label: "GNSS+DR", tone: "warn" },
  5: { label: "時刻のみ", tone: "error" },
};

function validPosition(lat: number, lon: number): boolean {
  return (
    Number.isFinite(lat) && Number.isFinite(lon) && (lat !== 0 || lon !== 0)
  );
}

/** NavPVT の測位の種類。RTK の Fixed・Float は搬送波位相の状態で決まる */
function navPvtFix(msg: NavPvt): { label: string; tone: GnssTone } {
  const carrier = (msg.flags >> CARRIER_PHASE_SHIFT) & CARRIER_PHASE_MASK;
  if (carrier === CARRIER_PHASE_FIXED) return { label: "RTK Fixed", tone: "ok" };
  if (carrier === CARRIER_PHASE_FLOAT) return { label: "RTK Float", tone: "warn" };
  const base = FIX_TYPE_LABEL[msg.fix_type] ?? {
    label: `fix ${msg.fix_type}`,
    tone: "neutral" as GnssTone,
  };
  // DOP や精度の条件を満たしていない(FIX_OK なし)測位は、警告のままにする
  const usable = (msg.flags & FLAGS_GNSS_FIX_OK) !== 0;
  return !usable && base.tone === "ok" ? { ...base, tone: "warn" } : base;
}

export function fromNavPvt(msg: NavPvt): GnssReading {
  const lat = msg.lat * LAT_LON_UNIT;
  const lon = msg.lon * LAT_LON_UNIT;
  const hasPosition = msg.fix_type >= 2 && validPosition(lat, lon);
  return {
    source: "navpvt",
    ...navPvtFix(msg),
    lat: hasPosition ? lat : null,
    lon: hasPosition ? lon : null,
    heightEllipsoid: hasPosition ? msg.height * MM : null,
    heightMsl: hasPosition ? msg.h_msl * MM : null,
    hAcc: msg.h_acc * MM,
    vAcc: msg.v_acc * MM,
    pDop: msg.p_dop > 0 ? msg.p_dop * P_DOP_UNIT : null,
    numSv: msg.num_sv,
  };
}

/** sensor_msgs/NavSatStatus の status。RTK の Fixed と Float は区別できない */
function navSatFixStatus(status: number): { label: string; tone: GnssTone } {
  switch (status) {
    case -1:
      return { label: "No Fix", tone: "error" };
    case 0:
      return { label: "GNSS", tone: "warn" };
    case 1:
      return { label: "SBAS", tone: "warn" };
    case 2:
      return { label: "RTK", tone: "ok" };
    default:
      return { label: "不明", tone: "neutral" };
  }
}

export function fromNavSatFix(msg: NavSatFix): GnssReading {
  const hasPosition =
    msg.status.status >= 0 && validPosition(msg.latitude, msg.longitude);
  // 共分散は対角が東・北・上の分散 [m^2]。type 0 は「不明」
  const cov = msg.position_covariance_type !== 0 ? msg.position_covariance : null;
  const sigma = (i: number) =>
    cov && cov[i] >= 0 ? Math.sqrt(cov[i]) : null;
  const east = sigma(0);
  const north = sigma(4);
  return {
    source: "navsatfix",
    ...navSatFixStatus(msg.status.status),
    lat: hasPosition ? msg.latitude : null,
    lon: hasPosition ? msg.longitude : null,
    heightEllipsoid: hasPosition ? msg.altitude : null,
    heightMsl: null,
    hAcc: east !== null && north !== null ? Math.max(east, north) : null,
    vAcc: sigma(8),
    pDop: null,
    numSv: null,
  };
}

export interface UtmPosition {
  /** "54N" のような、ゾーン番号と半球 */
  zone: string;
  easting: number;
  northing: number;
}

const WGS84 = "+proj=longlat +datum=WGS84 +no_defs";

/** 緯度経度を UTM に変換する。ゾーンは経度で決める(ノルウェーなどの特例は扱わない) */
export function toUtm(lat: number, lon: number): UtmPosition {
  const zoneNumber = Math.floor((lon + 180) / 6) + 1;
  const south = lat < 0;
  const utm = `+proj=utm +zone=${zoneNumber}${south ? " +south" : ""} +datum=WGS84 +units=m +no_defs`;
  const [easting, northing] = proj4(WGS84, utm, [lon, lat]);
  return { zone: `${zoneNumber}${south ? "S" : "N"}`, easting, northing };
}
