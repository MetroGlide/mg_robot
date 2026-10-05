import { useCallback, useEffect, useRef, useState } from 'react'
import type { ApiLog, CallApi, SystemManagerHandle } from '../types/api'
import { getSysManagerUrl } from '../utils/systemManagerConfig'

export type { ApiLog, CallApi, SystemManagerHandle }

const STATUS_POLL_INTERVAL_MS = 2000
const MAX_API_LOGS = 50

export function useSystemManagerClient(): SystemManagerHandle {
  const [containers, setContainers] = useState<Record<string, string>>({})
  const [logs, setLogs] = useState<ApiLog[]>([])
  const logIdRef = useRef(0)

  const refresh = useCallback(async () => {
    try {
      const r = await fetch(`${getSysManagerUrl()}/status`)
      if (r.ok) setContainers(await r.json())
    } catch {
      // system_manager が停止中でも UI は動かし続け、次の定期確認で回復を待つ
    }
  }, [])

  useEffect(() => {
    // 非表示のタブでは確認を止め、表示に戻ったときにすぐ更新する
    const tick = () => {
      if (!document.hidden) void refresh()
    }
    tick()
    const id = setInterval(tick, STATUS_POLL_INTERVAL_MS)
    document.addEventListener('visibilitychange', tick)
    return () => {
      clearInterval(id)
      document.removeEventListener('visibilitychange', tick)
    }
  }, [refresh])

  const callApi: CallApi = useCallback(
    async (path, body) => {
      let success = false
      let message = ''
      try {
        const r = await fetch(`${getSysManagerUrl()}${path}`, {
          method: 'POST',
          headers: body !== undefined ? { 'Content-Type': 'application/json' } : {},
          body: body !== undefined ? JSON.stringify(body) : undefined,
        })
        const data = await r.json()
        success = r.ok && (data.success ?? true)
        message = data.message ?? (typeof data.detail === 'string' ? data.detail : '')
        return { success, message, ...data }
      } catch (e) {
        message = e instanceof Error ? e.message : String(e)
        throw e
      } finally {
        const entry: ApiLog = {
          id: ++logIdRef.current,
          timestamp: new Date().toLocaleTimeString(),
          path,
          success,
          message,
        }
        setLogs((prev) => [entry, ...prev].slice(0, MAX_API_LOGS))
        // 操作直後の状態をすぐ反映する(定期確認を待たない)
        void refresh()
      }
    },
    [refresh],
  )

  return { containers, logs, callApi, refresh }
}
