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

export interface SystemManagerHandle {
  containers: Record<string, string>
  logs: ApiLog[]
  callApi: CallApi
  /** コンテナの状態を今すぐ取得し直す */
  refresh: () => Promise<void>
}
