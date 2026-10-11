import { useEffect, useMemo, useState } from 'react'
import { FoxgloveClientHandle } from './useFoxgloveClient'
import { DiagnosticArray, DiagnosticStatus } from '../types'
import { TOPICS } from '../ros/interfaces'
import { DIAG_ROOT } from '../ros/diagnosticPaths'
import { buildDiagnosticsTree, DiagNode } from '../utils/diagnosticsTree'

/**
 * 集約した診断(/diagnostics_agg)の木。aggregator は毎回、全項目を出すので、受け取るたびに置き換える
 * (届かなくなった項目は、aggregator が STALE にして出し続ける)。まだ受信していなければ null。
 */
export function useDiagnosticsAgg(client: FoxgloveClientHandle): DiagNode | null {
  const [statuses, setStatuses] = useState<DiagnosticStatus[] | null>(null)

  useEffect(() => {
    if (client.status !== 'connected') return
    return client.subscribe(
      TOPICS.DIAGNOSTICS_AGG,
      'diagnostic_msgs/msg/DiagnosticArray',
      (msg) => setStatuses((msg as DiagnosticArray).status),
    )
  }, [client, client.status])

  return useMemo(
    () => (statuses === null ? null : buildDiagnosticsTree(statuses, DIAG_ROOT)),
    [statuses],
  )
}
