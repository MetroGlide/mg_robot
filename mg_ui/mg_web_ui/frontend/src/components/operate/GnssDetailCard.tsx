import { useMemo } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { GnssState, useGnss } from "../../hooks/useGnss";
import { useOpsValue } from "../../hooks/useOpsValue";
import { GnssReading, toUtm } from "../../utils/gnssReading";
import Card from "../ui/Card";

/** KPI の GNSS タイルと共有する、詳細カードの開閉の保存キー */
export const GNSS_DETAIL_KEY = "kpi.gnssDetail";

const NO_VALUE = "--";

function fixed(value: number | null, digits: number, unit = ""): string {
  return value === null ? NO_VALUE : `${value.toFixed(digits)}${unit ? ` ${unit}` : ""}`;
}

function Row({ label, value, hint }: { label: string; value: string; hint?: string }) {
  return (
    <div className="flex items-baseline justify-between gap-3 py-0.5 text-xs">
      <span className="shrink-0 text-muted">{label}</span>
      <span
        className="truncate text-right font-semibold tabular-nums"
        title={hint}
      >
        {value}
      </span>
    </div>
  );
}

/** 購読元にない値は、理由を添えて「--」にする */
function missing(reading: GnssReading, value: number | null, digits: number, unit: string) {
  return value === null && reading.source === "navsatfix"
    ? { value: NO_VALUE, hint: "NavSatFix には含まれない値。購読元を NavPVT にすると表示する" }
    : { value: fixed(value, digits, unit), hint: undefined };
}

/**
 * GNSS の詳細。測位の種類、衛星数、緯度経度、UTM 座標、精度、DOP、高さ。
 */
function GnssDetailBody({ gnss }: { gnss: GnssState }) {
  const { reading, freshness, topic } = gnss;
  const utm = useMemo(
    () =>
      reading && reading.lat !== null && reading.lon !== null
        ? toUtm(reading.lat, reading.lon)
        : null,
    [reading],
  );

  if (!reading || freshness.stale) {
    return (
      <Card className="w-72 p-3">
        <h2 className="mb-1 text-sm font-bold">GNSS の詳細</h2>
        <p className="text-xs text-muted">
          {topic} を受信していません
          {freshness.ageSec !== null ? `(${Math.floor(freshness.ageSec)}秒 更新なし)` : ""}
        </p>
      </Card>
    );
  }

  const sv = missing(reading, reading.numSv, 0, "");
  const pDop = missing(reading, reading.pDop, 2, "");
  const msl = missing(reading, reading.heightMsl, 2, "m");

  return (
    <Card className="w-72 p-3">
      <div className="mb-1 flex items-baseline justify-between">
        <h2 className="text-sm font-bold">GNSS の詳細</h2>
        <span className="text-[11px] text-muted">{topic}</span>
      </div>
      <Row label="測位" value={reading.label} />
      <Row label="衛星数" value={sv.value} hint={sv.hint} />
      <div className="my-1 h-px bg-line" />
      <Row label="緯度" value={fixed(reading.lat, 7, "°")} />
      <Row label="経度" value={fixed(reading.lon, 7, "°")} />
      <Row label="UTM ゾーン" value={utm ? utm.zone : NO_VALUE} />
      <Row label="UTM E" value={utm ? `${utm.easting.toFixed(3)} m` : NO_VALUE} />
      <Row label="UTM N" value={utm ? `${utm.northing.toFixed(3)} m` : NO_VALUE} />
      <div className="my-1 h-px bg-line" />
      <Row
        label="水平精度"
        value={fixed(reading.hAcc, 3, "m")}
        hint={reading.source === "navsatfix" ? "共分散から求めた値" : undefined}
      />
      <Row label="垂直精度" value={fixed(reading.vAcc, 3, "m")} />
      <Row label="PDOP" value={pDop.value} hint={pDop.hint} />
      <div className="my-1 h-px bg-line" />
      <Row label="楕円体高" value={fixed(reading.heightEllipsoid, 2, "m")} />
      <Row label="標高(海抜)" value={msl.value} hint={msl.hint} />
    </Card>
  );
}

// 詳細は人が読む速さで十分なので間引く(Hz)
const DETAIL_HZ = 2;

function GnssDetailSubscribed({ client }: { client: FoxgloveClientHandle }) {
  return <GnssDetailBody gnss={useGnss(client, DETAIL_HZ)} />;
}

/**
 * KPI の GNSS タイルで開閉する詳細カード。右側のカードの列に置く。
 * 開いている間だけ購読する(閉じると購読をやめる)。
 */
export default function GnssDetailCard({ client }: { client: FoxgloveClientHandle }) {
  const [open] = useOpsValue<boolean>(GNSS_DETAIL_KEY, false);
  return open ? <GnssDetailSubscribed client={client} /> : null;
}
