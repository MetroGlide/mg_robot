import { getSysManagerUrl } from "./systemManagerConfig";

export async function loadSettings(): Promise<Record<string, unknown>> {
  try {
    const data = await fetch(`${getSysManagerUrl()}/settings`).then((r) => r.json());
    return typeof data === "object" && data !== null ? (data as Record<string, unknown>) : {};
  } catch {
    return {};
  }
}

/** 指定したキーだけを保存する。value が null の場合はそのキーを削除する。 */
export function saveSettings(key: string, value: unknown): void {
  fetch(`${getSysManagerUrl()}/settings`, {
    method: "PATCH",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ [key]: value }),
  }).catch(() => {});
}
