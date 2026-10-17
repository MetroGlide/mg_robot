export type ContainerStatus = 'running' | 'exited' | 'dead' | 'unknown'

export type CallApi = (
  path: string,
  body?: unknown,
) => Promise<{ success: boolean; message: string }>

/** system_manager の GET /navigation/maps の応答 */
export interface NavigationMapEntry {
  name: string
  /** map_server に渡す YAML の絶対パス */
  path: string
  /** map_list.txt にあるが、ファイルが存在しない */
  missing: boolean
}

export type NavigationMapsResponse =
  | { success: true; map_path: string; maps: NavigationMapEntry[]; skipped: string[] }
  | { success: false; message: string }

export interface ApiLog {
  id: number
  timestamp: string
  path: string
  success: boolean
  message: string
}

export interface LogEntry {
  id: number
  service: string
  line: string
}

/** system_manager が判定したコンテナの深刻度(UI は判定しない) */
export type ContainerLevel = 'ok' | 'warn' | 'error' | 'off'

export interface ContainerDetail {
  state: string
  health: string | null
  exit_code: number | null
  level: ContainerLevel
}

export interface SystemManagerHandle {
  containers: Record<string, string>
  /** `/status/detail` の内容。深刻度 `level` を含む */
  details: Record<string, ContainerDetail>
  logs: ApiLog[]
  callApi: CallApi
  /** コンテナの状態を今すぐ取得し直す */
  refresh: () => Promise<void>
}
