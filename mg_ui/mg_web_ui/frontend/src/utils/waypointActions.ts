import { normalizeType, parse } from "@foxglove/rosmsg";
import type { GateArbiterState } from "../types";

type MessageDefinition = ReturnType<typeof parse>[number];

export type JsonObjectResult =
  { ok: true; value: Record<string, unknown> } | { ok: false; error: string };

/** 入力欄の JSON を解析する。オブジェクト以外 (配列・数値など) は受け付けない。 */
export function parseJsonObject(text: string): JsonObjectResult {
  const trimmed = text.trim();
  if (trimmed === "") return { ok: true, value: {} };
  let value: unknown;
  try {
    value = JSON.parse(trimmed);
  } catch (e) {
    return { ok: false, error: e instanceof Error ? e.message : String(e) };
  }
  if (typeof value !== "object" || value === null || Array.isArray(value)) {
    return { ok: false, error: "JSON object ({...}) is required" };
  }
  return { ok: true, value: value as Record<string, unknown> };
}

const TIME_TYPES = new Set(["time", "duration"]);

function primitiveDefault(type: string): unknown {
  if (type === "string" || type === "wstring") return "";
  if (type === "bool") return false;
  if (TIME_TYPES.has(type)) return { sec: 0, nanosec: 0 };
  return 0;
}

function buildFromDefinition(
  definition: MessageDefinition,
  byName: Map<string, MessageDefinition>,
): Record<string, unknown> {
  const result: Record<string, unknown> = {};
  for (const field of definition.definitions) {
    if (field.isConstant) continue;
    const nested = field.isComplex
      ? byName.get(normalizeType(field.type))
      : undefined;
    const makeOne = (): unknown =>
      nested
        ? buildFromDefinition(nested, byName)
        : primitiveDefault(field.type);
    result[field.name] = field.isArray
      ? Array.from({ length: field.arrayLength ?? 0 }, makeOne)
      : makeOne();
  }
  return result;
}

/** メッセージ定義 (スキーマの文字列) から、すべてのフィールドを初期値で埋めた JSON のひな形を作る。 */
export function buildMessageTemplate(
  schemaText: string,
): Record<string, unknown> {
  const definitions = parse(schemaText, { ros2: true });
  const byName = new Map<string, MessageDefinition>();
  for (const definition of definitions) {
    if (definition.name) byName.set(normalizeType(definition.name), definition);
  }
  const [root] = definitions;
  return root ? buildFromDefinition(root, byName) : {};
}

export interface NavigationModeInfo {
  mode: string;
  behavior_tree: string;
}

/** sequencer の ~/navigation_mode (std_msgs/String の JSON) を解釈する。形式が違えば null。 */
export function parseNavigationMode(data: string): NavigationModeInfo | null {
  let value: unknown;
  try {
    value = JSON.parse(data);
  } catch {
    return null;
  }
  if (typeof value !== "object" || value === null) return null;
  const { mode, behavior_tree: behaviorTree } = value as Record<
    string,
    unknown
  >;
  if (typeof mode !== "string" || typeof behaviorTree !== "string") return null;
  return { mode, behavior_tree: behaviorTree };
}

export interface ActionResult {
  ok: boolean;
  text: string;
}

/**
 * サービスの応答を表示用にまとめる。{success, message} 形式なら success で成否を判断し、
 * それ以外 (Empty など) は応答の JSON をそのまま見せて成功とする。
 */
export function describeServiceResponse(response: unknown): ActionResult {
  if (typeof response === "object" && response !== null) {
    const { success, message } = response as Record<string, unknown>;
    if (typeof success === "boolean") {
      return {
        ok: success,
        text:
          typeof message === "string" && message !== ""
            ? message
            : String(success),
      };
    }
  }
  return { ok: true, text: JSON.stringify(response) ?? "ok" };
}

export type GateLabel = "ON" | "OFF" | "PENDING" | "UNKNOWN";

export interface GateView {
  label: GateLabel;
  detail: string;
}

/** AMCL のゲートの状態を、表示用のラベルと補足にする。 */
export function describeAmclGate(state: GateArbiterState | null): GateView {
  if (state === null) return { label: "UNKNOWN", detail: "" };
  const holders = state.holders.join(", ");
  if (!state.applied) {
    return {
      label: "PENDING",
      detail: `${state.desired ? "ON" : "OFF"} requested, not applied to the gate yet`,
    };
  }
  if (state.desired) return { label: "ON", detail: "" };
  return { label: "OFF", detail: holders ? `held by: ${holders}` : "" };
}

/** GNSS ブリッジの配信状態 (/odom/gps) を、表示用のラベルにする。 */
export function describeGnssGate(publishing: boolean | null): GateView {
  if (publishing === null) return { label: "UNKNOWN", detail: "" };
  return { label: publishing ? "ON" : "OFF", detail: "" };
}

/** 地図の YAML のパスから、一覧に出す短い名前を作る。 */
export function mapDisplayName(path: string): string {
  return path === "" ? "—" : (path.split("/").pop() ?? path);
}
