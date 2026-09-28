import React from 'react';
import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { useConsoleStore } from '../stores/useConsoleStore';
import {
  explainCellResolution,
  resolutionColor,
  resolutionMeters,
} from '../utils/resolution';
import { semanticClassColor } from '../utils/semantic';
import styles from './SelectedObjectInspector.module.css';

/**
 * @file SelectedObjectInspector.tsx
 * @brief Authoritative scientific precision inspector for adaptive cells ("WHY THIS RESOLUTION?")
 *        and dynamic object clusters ("FRAME CLUSTER C-XX").
 *
 * Implements F2A-R2.1:
 * - Mathematical source-truth using frozen backend distance policy: baselineLevelForDistance()
 * - Honest refinement progression (e.g. L1 · 10cm -> L0 · 5cm for 10.82m pedestrian)
 * - Zero manufactured evidence tags (no fake UNC, GEO STABLE, DEN PASS, or DYN NO)
 * - Canonical semantic policy tier (SAFETY CRITICAL) from frozen backend taxonomy
 * - Scene frame coordinate labels (no ungrounded ODOM claims)
 * - Clear prototype disclaimer for single-frame dynamic clusters
 */
export const SelectedObjectInspector: React.FC = () => {
  const {
    isInspectorOpen,
    selectedClusterId,
    selectedCellId,
    clearSelection,
  } = useConsoleStore();

  if (!isInspectorOpen || (selectedClusterId === null && selectedCellId === null)) {
    return null;
  }

  const frame = frameRuntime.getLatestFrame();

  const cluster =
    selectedClusterId !== null
      ? frame.dynamicClusters.find((c) => c.clusterLocalId === selectedClusterId)
      : null;

  const cell =
    selectedCellId !== null
      ? frame.cells.find((c) => c.cellId === selectedCellId)
      : null;

  // Sensor origin for mathematical distance computations
  const sensorOrigin = frame.sensorOrigin;

  return (
    <aside className={styles.inspectorPanel} aria-label="Perception Explainability Inspector">
      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerLeft}>
          <span
            className={styles.badgeIndicator}
            style={{
              backgroundColor: cell
                ? resolutionColor(cell.level)
                : 'var(--dynamic-coral, #ff7557)',
            }}
          />
          <div>
            <div className={styles.headerTitle}>
              {cell && 'WHY THIS RESOLUTION?'}
              {cluster && `FRAME CLUSTER C-0${cluster.clusterLocalId}`}
            </div>
            <div className={styles.headerSubtitle}>
              {cell && `CELL KEY · ${cell.cellId}`}
              {cluster && 'SINGLE-FRAME DYNAMIC CANDIDATE'}
            </div>
          </div>
        </div>
        <button
          type="button"
          className={styles.closeBtn}
          onClick={clearSelection}
          title="Close Inspector (Esc)"
          aria-label="Close Inspector"
        >
          ✕
        </button>
      </div>

      <div className={styles.content}>
        {/* ================================================================= */}
        {/* CELL EXPLAINABILITY: "WHY THIS RESOLUTION?"                       */}
        {/* ================================================================= */}
        {cell && (() => {
          const explanation = explainCellResolution(cell, frame);
          const distanceMeters = explanation.policyDistanceMeters;
          const baselineLevel = explanation.baselineLevel;
          const refinementSteps = baselineLevel - cell.level;

          const hasSemantic = cell.resolutionReasons.includes('SEMANTIC_RELEVANCE');
          const hasDynamic = cell.resolutionReasons.includes('DYNAMIC_RELEVANCE');
          const hasGeometry = cell.resolutionReasons.includes('GEOMETRY_COMPLEXITY');

          const semanticClass = explanation.semanticClass;
          const policyTier =
            explanation.semanticPolicyTier === 'SAFETY_CRITICAL'
              ? 'SAFETY CRITICAL'
              : explanation.semanticPolicyTier === 'ELEVATED'
              ? 'ELEVATED'
              : explanation.semanticPolicyTier || null;

          return (
            <div className={styles.section}>
              {/* Refinement Progression Card */}
              <div className={styles.refinementCard}>
                <div className={styles.refineStep}>
                  <span className={styles.refineTag}>BASE</span>
                  <div className={styles.refineInfo}>
                    <div className={styles.refineTitle}>
                      L{baselineLevel} · {(resolutionMeters(baselineLevel) * 100).toFixed(0)} cm
                    </div>
                    <div className={styles.refineDesc}>
                      Distance baseline ({distanceMeters < 10 ? '< 10 m' : distanceMeters < 30 ? '10 m – 30 m' : distanceMeters < 60 ? '30 m – 60 m' : '≥ 60 m'})
                    </div>
                  </div>
                </div>

                <div className={styles.refineArrow}>
                  {refinementSteps > 0
                    ? `↓ refined ${refinementSteps} ${refinementSteps === 1 ? 'level' : 'levels'} (${(resolutionMeters(baselineLevel) * 100).toFixed(0)}cm → ${(cell.sizeMeters * 100).toFixed(0)}cm)`
                    : refinementSteps === 0 ? '= same as distance baseline' : 'Authored demo topology · coarser than distance baseline'}
                </div>

                <div className={styles.refineStepFinal}>
                  <span
                    className={styles.refineTagFinal}
                    style={{
                      color: resolutionColor(cell.level),
                      borderColor: resolutionColor(cell.level),
                    }}
                  >
                    FINAL
                  </span>
                  <div className={styles.refineInfo}>
                    <div
                      className={styles.refineTitle}
                      style={{ color: resolutionColor(cell.level) }}
                    >
                      L{cell.level} · {(cell.sizeMeters * 100).toFixed(0)} cm
                    </div>
                    <div className={styles.refineDesc}>
                      {hasSemantic && semanticClass
                        ? `Semantic relevance: ${semanticClass}`
                        : hasDynamic
                        ? 'Dynamic relevance: fixture decision region'
                        : hasGeometry
                        ? 'Geometric edge complexity'
                        : refinementSteps < 0 ? 'Illustrative adaptive leaf' : 'Distance baseline leaf'}
                    </div>
                  </div>
                </div>
              </div>

              {/* Truthful Resolution Evidence Card */}
              <div className={styles.subSectionTitle}>RESOLUTION EVIDENCE & POLICY</div>
              <div className={styles.evidenceCard}>
                <div className={styles.metaRow}>
                  <span className={styles.label}>Decision Evidence</span>
                  <span className={styles.value}>
                    {explanation.decisionEvidence || 'Distance baseline geometry'}
                  </span>
                </div>
                <div className={styles.metaRow}>
                  <span className={styles.label}>Static Fusion</span>
                  <span className={styles.routingValue}>
                    {explanation.staticFusion.replace('_', ' ')}
                  </span>
                </div>
                {hasSemantic && semanticClass && (
                  <>
                    <div className={styles.metaRow}>
                      <span className={styles.label}>Semantic Class</span>
                      <span className={styles.classValue} style={{ color: '#ed8177' }}>
                        {semanticClass}
                      </span>
                    </div>
                    <div className={styles.metaRow}>
                      <span className={styles.label}>Policy Tier</span>
                      <span className={styles.policyTierValue}>
                        {policyTier}
                      </span>
                    </div>
                  </>
                )}
                {explanation.routing && (
                  <div className={styles.metaRow}>
                    <span className={styles.label}>Routing</span>
                    <span className={styles.routingValue}>
                      {explanation.routing === 'DYNAMIC_EXCLUDED'
                        ? 'DYNAMIC / EXCLUDED FROM STATIC MAP'
                        : explanation.routing === 'STATIC_FUSABLE'
                        ? 'STATIC / FUSABLE INTO TERRAIN'
                        : 'MIXED'}
                    </span>
                  </div>
                )}
                <div className={styles.metaRow}>
                  <span className={styles.label}>Current-Frame Returns</span>
                  <span className={styles.value}>{explanation.pointCount}</span>
                </div>
                {explanation.sourceIndices && explanation.sourceIndices.length > 0 && (
                  <div className={styles.metaRow}>
                    <span className={styles.label}>Supporting Source Indices</span>
                    <span className={styles.value}>
                      {explanation.sourceIndices.join(', ')}
                    </span>
                  </div>
                )}
                {explanation.dynamicClusterId !== undefined && (
                  <div className={styles.metaRow}>
                    <span className={styles.label}>Frame-local cluster</span>
                    <span className={styles.value}>C-{String(explanation.dynamicClusterId).padStart(2, '0')}</span>
                  </div>
                )}
              </div>

              {/* Canonical Decision Reasons List */}
              <div className={styles.subSectionTitle}>CANONICAL DECISION REASONS</div>
              <div className={styles.reasonsList}>
                {cell.resolutionReasons.map((r) => (
                  <div key={r} className={styles.reasonRow}>
                    <span className={styles.reasonBullet}>▪</span>
                    <span className={styles.reasonName}>{r}</span>
                    <span className={styles.reasonNote}>
                      {r === 'SEMANTIC_RELEVANCE' && 'Semantic relevance trigger'}
                      {r === 'DYNAMIC_RELEVANCE' && 'Dynamic hazard isolation'}
                      {r === 'DISTANCE_BASELINE' && 'Distance baseline choice'}
                      {r === 'UNCERTAINTY' && 'Uncertainty reduction gate'}
                      {r === 'GEOMETRY_COMPLEXITY' && 'Elevation roughness step'}
                      {r === 'INSUFFICIENT_EVIDENCE' && 'Insufficient evidence hold'}
                      {r === 'DENSITY_GATE_BLOCKED' && 'Point density limit'}
                    </span>
                  </div>
                ))}
              </div>

              {/* Spatial Geometry & Distance */}
              <div className={styles.subSectionTitle}>SPATIAL & GEOMETRIC STATE</div>
              <div className={styles.gridParams}>
                <div className={styles.paramBox}>
                  <div className={styles.paramLabel}>POLICY DISTANCE (XY)</div>
                  <div className={styles.paramValue}>{distanceMeters.toFixed(2)} m · XY</div>
                </div>
                <div className={styles.paramBox}>
                  <div className={styles.paramLabel}>CURRENT-FRAME RETURNS</div>
                  <div className={styles.paramValue}>{explanation.pointCount}</div>
                </div>
                <div className={styles.paramBox}>
                  <div className={styles.paramLabel}>CELL EDGE</div>
                  <div className={styles.paramValue}>{(cell.sizeMeters * 100).toFixed(0)} cm</div>
                </div>
                <div className={styles.paramBox}>
                  <div className={styles.paramLabel}>CELL AREA</div>
                  <div className={styles.paramValue}>{(cell.sizeMeters * cell.sizeMeters).toFixed(4)} m²</div>
                </div>
              </div>

              {/* Elevation & Static Map Payload */}
              <div className={styles.detailBlock}>
                <div className={styles.blockTitle}>STATIC MAP PAYLOAD</div>
                {explanation.hasStaticElevationSupport && explanation.staticElevationStats ? (
                  <div className={styles.vectorRow}>
                    <span>MEAN {explanation.staticElevationStats.mean.toFixed(2)} m</span>
                    <span>MIN {explanation.staticElevationStats.min.toFixed(2)} m</span>
                    <span>MAX {explanation.staticElevationStats.max.toFixed(2)} m</span>
                  </div>
                ) : (
                  <div className={styles.staticFusionExcluded}>
                    {explanation.staticFusion === 'EXCLUDED' ? 'NO STATIC-FUSABLE ELEVATION SUPPORT' : 'Persistent elevation not supplied by this fixture.'}
                  </div>
                )}
              </div>

              <div className={styles.detailBlock}>
                <div className={styles.blockTitle}>CENTER COORDINATE (SCENE FRAME)</div>
                <div className={styles.vectorRow}>
                  <span>X: {cell.x.toFixed(3)} m</span>
                  <span>Y: {cell.y.toFixed(3)} m</span>
                </div>
              </div>
            </div>
          );
        })()}

        {/* ================================================================= */}
        {/* DYNAMIC CLUSTER INSPECTOR                                         */}
        {/* ================================================================= */}
        {cluster && (() => {
          const dist2D = Math.hypot(
            cluster.centroid.x - sensorOrigin.x,
            cluster.centroid.y - sensorOrigin.y
          );
          const dx = cluster.aabbMax.x - cluster.aabbMin.x;
          const dy = cluster.aabbMax.y - cluster.aabbMin.y;
          const dz = cluster.aabbMax.z - cluster.aabbMin.z;

          return (
            <div className={styles.section}>
              <div className={styles.metaRow}>
                <span className={styles.label}>Class</span>
                <span
                  className={styles.classValue}
                  style={{ color: semanticClassColor(cluster.dominantClass) }}
                >
                  {cluster.dominantClass}
                </span>
              </div>

              <div className={styles.metaRow}>
                <span className={styles.label}>Routing Contract</span>
                <span className={styles.routingValue}>
                  Dynamic / Excluded from Static Map
                </span>
              </div>

              <div className={styles.gridParams}>
                <div className={styles.paramBox}>
                  <div className={styles.paramLabel}>POINT RETURNS</div>
                  <div className={styles.paramValue}>{cluster.pointCount} PTS</div>
                </div>
                <div className={styles.paramBox}>
                  <div className={styles.paramLabel}>CANONICAL IDX</div>
                  <div className={styles.paramValue}>#{cluster.canonicalSourcePointIndex}</div>
                </div>
                <div className={styles.paramBox}>
                  <div className={styles.paramLabel}>SEM CONFIDENCE</div>
                  <div className={styles.paramValue}>
                    {cluster.meanSemanticConfidence.toFixed(3)}
                  </div>
                </div>
                <div className={styles.paramBox}>
                  <div className={styles.paramLabel}>MOTION CONFIDENCE</div>
                  <div className={styles.paramValue}>
                    {cluster.meanMotionConfidence.toFixed(3)}
                  </div>
                </div>
              </div>

              <div className={styles.detailBlock}>
                <div className={styles.blockTitle}>CENTROID (SCENE FRAME)</div>
                <div className={styles.vectorRow}>
                  <span>X: {cluster.centroid.x.toFixed(3)} m</span>
                  <span>Y: {cluster.centroid.y.toFixed(3)} m</span>
                  <span>Z: {cluster.centroid.z.toFixed(3)} m</span>
                </div>
                <div className={styles.sensorDistRow}>
                  DISTANCE FROM SENSOR (2D PLANAR): {dist2D.toFixed(2)} m
                </div>
              </div>

              <div className={styles.detailBlock}>
                <div className={styles.blockTitle}>BOUNDING AABB EXTENTS</div>
                <div className={styles.vectorRow}>
                  <span>WIDTH (ΔX): {dx.toFixed(2)} m</span>
                  <span>LENGTH (ΔY): {dy.toFixed(2)} m</span>
                </div>
                <div className={styles.vectorRow} style={{ marginTop: '4px' }}>
                  <span>HEIGHT (ΔZ): {dz.toFixed(2)} m</span>
                  <span>VOL: {(dx * dy * dz).toFixed(2)} m³</span>
                </div>
              </div>

              <div className={styles.note}>
                CURRENT PROTOTYPE: Frame-local cluster only. Temporal tracking, velocity and trajectory are not yet enabled.
              </div>

              <div className={styles.detailBlock}>
                <div className={styles.blockTitle}>MEMBER SOURCE INDICES / CURRENT FRAME</div>
                <div className={styles.value}>{cluster.memberSourceIndices.join(', ')}</div>
              </div>
            </div>
          );
        })()}
      </div>
    </aside>
  );
};
