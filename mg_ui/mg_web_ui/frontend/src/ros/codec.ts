import { parse } from "@foxglove/rosmsg";
import { MessageReader, MessageWriter } from "@foxglove/rosmsg2-serialization";

export interface AdvertisedSchema {
  encoding: string;
  schemaName: string;
  schema: string;
}

const readers = new Map<string, MessageReader>();
const writers = new Map<string, MessageWriter>();

function schemaKey(schema: AdvertisedSchema): string {
  return `${schema.schemaName}:${schema.schema}`;
}

function getReader(schema: AdvertisedSchema): MessageReader | undefined {
  const key = schemaKey(schema);
  const cached = readers.get(key);
  if (cached) return cached;
  try {
    const reader = new MessageReader(parse(schema.schema, { ros2: true }));
    readers.set(key, reader);
    return reader;
  } catch {
    return undefined;
  }
}

function getWriter(schema: AdvertisedSchema): MessageWriter | undefined {
  const key = schemaKey(schema);
  const cached = writers.get(key);
  if (cached) return cached;
  try {
    const writer = new MessageWriter(parse(schema.schema, { ros2: true }));
    writers.set(key, writer);
    return writer;
  } catch {
    return undefined;
  }
}

/** ペイロードをスキーマに従ってデコードする。デコードできない場合は生のバッファを返す。 */
export function decodePayload(
  payload: ArrayBuffer,
  schema: AdvertisedSchema | undefined,
): unknown {
  if (!schema) return payload;

  if (schema.encoding === "json") {
    try {
      return JSON.parse(new TextDecoder().decode(payload));
    } catch {
      return payload;
    }
  }
  if (schema.encoding !== "cdr") return payload;

  const reader = getReader(schema);
  if (!reader) return payload;
  try {
    return reader.readMessage(new Uint8Array(payload));
  } catch {
    return payload;
  }
}

/**
 * データをスキーマに従ってエンコードする。スキーマが無い場合と json の場合は JSON にする。
 * cdr でエンコードできない場合は、壊れたデータを送らないよう例外にする。
 */
export function encodePayload(
  data: unknown,
  schema: AdvertisedSchema | undefined,
): Uint8Array {
  if (!schema || schema.encoding === "json") {
    return new TextEncoder().encode(JSON.stringify(data));
  }
  const writer = getWriter(schema);
  if (!writer) {
    throw new Error(`cannot build a message writer for ${schema.schemaName}`);
  }
  return writer.writeMessage(data as Record<string, unknown>);
}
