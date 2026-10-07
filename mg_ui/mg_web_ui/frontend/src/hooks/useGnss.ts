import { useEffect, useState } from "react";
import { FoxgloveClientHandle } from "./useFoxgloveClient";
import { useFreshness, Freshness } from "./useFreshness";
import { useGnssSource } from "../contexts/GnssSourceContext";
import { TOPICS } from "../ros/interfaces";
import type { NavPvt, NavSatFix } from "../types";
import {
  fromNavPvt,
  fromNavSatFix,
  GnssReading,
  GnssSource,
} from "../utils/gnssReading";

// これを超えて受信がなければ値を古いものとして扱う(秒)
const STALE_AFTER_SEC = 5;

const SOURCES: Record<
  GnssSource,
  {
    topic: string;
    schema: string;
    convert: (msg: never) => GnssReading;
  }
> = {
  navpvt: {
    topic: TOPICS.NAVPVT,
    schema: "ublox_msgs/msg/NavPVT",
    convert: fromNavPvt as (msg: never) => GnssReading,
  },
  navsatfix: {
    topic: TOPICS.GPS_FIX,
    schema: "sensor_msgs/msg/NavSatFix",
    convert: fromNavSatFix as (msg: never) => GnssReading,
  },
};

export interface GnssState {
  reading: GnssReading | null;
  topic: string;
  freshness: Freshness;
}

/**
 * 設定で選んだ購読元(NavPVT か NavSatFix)の 1 トピックだけを購読し、共通の形にして返す。
 * 人が読むだけなので、再描画は maxHz に間引く(最後の値は必ず反映する)。
 */
export function useGnss(client: FoxgloveClientHandle, maxHz: number): GnssState {
  const { source } = useGnssSource();
  const { topic, schema, convert } = SOURCES[source];
  // 購読元を切り替えた直後に、前の購読元の値を出さないよう、どの購読元の値かを持つ
  const [latest, setLatest] = useState<{
    source: GnssSource;
    reading: GnssReading;
  } | null>(null);
  const { subscribe } = client;

  useEffect(() => {
    const intervalMs = 1000 / maxHz;
    let lastEmitAt = 0;
    let pending: GnssReading | null = null;
    let timer: ReturnType<typeof setTimeout> | null = null;

    const emit = () => {
      timer = null;
      lastEmitAt = Date.now();
      if (pending) setLatest({ source, reading: pending });
    };

    const unsubscribe = subscribe(topic, schema, (msg) => {
      pending = convert(msg as never);
      if (timer !== null) return;
      const wait = lastEmitAt + intervalMs - Date.now();
      if (wait <= 0) emit();
      else timer = setTimeout(emit, wait);
    });
    return () => {
      unsubscribe();
      if (timer !== null) clearTimeout(timer);
    };
  }, [subscribe, source, topic, schema, convert, maxHz]);

  const freshness = useFreshness(client, topic, STALE_AFTER_SEC);
  return {
    reading: latest && latest.source === source ? latest.reading : null,
    topic,
    freshness,
  };
}
