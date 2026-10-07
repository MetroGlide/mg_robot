import { PoseGraphState } from '../../hooks/usePoseGraph'

interface Props {
  state: PoseGraphState
  selectedNodeIndex: number | null
  onClose: () => void
  /** 配置。旧ページは地図の上に重ねるため、既定は絶対位置。運用ビューでは "" を渡し、レイアウトの枠に任せる */
  className?: string
}

export function PoseGraphDetailPanel({
  state,
  selectedNodeIndex,
  onClose,
  className = 'absolute top-4 left-4 z-50',
}: Props) {
  if (selectedNodeIndex === null) return null
  
  const node = state.nodes.get(selectedNodeIndex)
  if (!node) return null

  const prior = state.gnssPriors.get(selectedNodeIndex)
  
  const relatedEdges = Array.from(state.seqEdges.values()).filter(
    e => e.from === selectedNodeIndex || e.to === selectedNodeIndex
  )
  const relatedLoopEdges = Array.from(state.loopEdges.values()).filter(
    e => e.from === selectedNodeIndex || e.to === selectedNodeIndex
  )

  return (
    <div className={`${className} flex w-80 flex-col overflow-hidden rounded-lg border border-line bg-surface-elevated/95 text-content shadow-card`}>
      <div className="flex items-center justify-between border-b border-line bg-surface-sunken px-4 py-2">
        <h3 className="font-semibold text-sm">Node #{selectedNodeIndex} Details</h3>
        <button onClick={onClose} className="text-muted hover:text-content">✕</button>
      </div>
      
      <div className="p-4 space-y-4 max-h-[60vh] overflow-y-auto text-sm custom-scrollbar">
        <section>
          <h4 className="text-muted text-xs font-semibold uppercase mb-1">Pose</h4>
          <div className="grid grid-cols-2 gap-1 font-mono">
            <span>X:</span><span>{node.x.toFixed(3)} m</span>
            <span>Y:</span><span>{node.y.toFixed(3)} m</span>
            <span>Yaw:</span><span>{node.yaw.toFixed(3)} rad</span>
            <span>Time:</span><span>{node.timestamp.toFixed(3)}</span>
          </div>
        </section>

        {prior && (
          <section>
            <h4 className="text-warn text-xs font-semibold uppercase mb-1">GNSS Prior</h4>
            <div className="grid grid-cols-2 gap-1 font-mono bg-surface-sunken p-2 rounded">
              <span>Sigma:</span><span>{prior.sigma_m > 0 ? `${prior.sigma_m.toFixed(3)} m` : 'N/A'}</span>
              <span>Status:</span><span>{prior.status}</span>
            </div>
          </section>
        )}

        <section>
          <h4 className="text-ok text-xs font-semibold uppercase mb-1">Seq Edges ({relatedEdges.length})</h4>
          <ul className="space-y-2">
            {relatedEdges.map(e => (
              <li key={`${e.from}-${e.to}`} className="bg-surface-sunken p-2 rounded">
                <div className="font-mono text-xs">{e.from} → {e.to}</div>
                <div className="grid grid-cols-2 gap-x-2 text-xs text-muted mt-1">
                  <span>Score:</span><span className={e.score > 0.05 ? "text-error" : ""}>{e.score > 0 ? e.score.toFixed(4) : "N/A"}</span>
                  <span>Type:</span><span className={e.type === 1 ? "text-warn" : ""}>{e.type === 1 ? 'OdomFallback' : 'ICP'}</span>
                </div>
              </li>
            ))}
          </ul>
        </section>

        {relatedLoopEdges.length > 0 && (
          <section>
            <h4 className="text-fuchsia-500 text-xs font-semibold uppercase mb-1">Loop Edges ({relatedLoopEdges.length})</h4>
            <ul className="space-y-2">
              {relatedLoopEdges.map(e => (
                <li key={`loop-${e.from}-${e.to}`} className="bg-surface-sunken p-2 rounded border border-fuchsia-500/30">
                  <div className="font-mono text-xs text-fuchsia-500">{e.from} → {e.to}</div>
                  <div className="grid grid-cols-2 gap-x-2 text-xs text-muted mt-1">
                    <span>Score:</span><span>{e.score > 0 ? e.score.toFixed(4) : "N/A"}</span>
                  </div>
                </li>
              ))}
            </ul>
          </section>
        )}

        <section className="border-t border-line pt-2">
          <h4 className="text-muted text-xs font-semibold uppercase mb-1 flex justify-between">
            <span>Graph Stats</span>
          </h4>
          <div className="grid grid-cols-2 gap-1 font-mono text-xs text-muted bg-surface-sunken p-2 rounded">
            <span>Nodes:</span><span>{state.stats.total_nodes}</span>
            <span>Seq Edges:</span><span>{state.stats.total_seq_edges}</span>
            <span>Loop Edges:</span><span>{state.stats.total_loop_edges}</span>
            <span>ICP Success:</span>
            <span className={state.stats.icp_attempt_count > 0 && (state.stats.icp_success_count/state.stats.icp_attempt_count) < 0.9 ? 'text-error' : 'text-ok'}>
              {state.stats.icp_attempt_count > 0 ? `${(state.stats.icp_success_count/state.stats.icp_attempt_count*100).toFixed(1)}%` : 'N/A'}
            </span>
            <span>Odom FB:</span><span className={state.stats.odom_fallback_count > 0 ? 'text-warn' : ''}>{state.stats.odom_fallback_count}</span>
          </div>
        </section>
      </div>
    </div>
  )
}
