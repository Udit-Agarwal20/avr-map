import React from 'react';
import { useFrameMetadata } from '../../runtime/useFrameMetadata';
import { useConsoleStore } from '../../stores/useConsoleStore';
import { SemanticClass } from '../../types/perception';
import { resolutionColor, resolutionLabel } from '../../utils/resolution';
import { semanticClassColor, semanticClassLabel } from '../../utils/semantic';
import { pipelineStatusLabel, pipelineStatusSeverity } from '../../utils/status';
import { Badge } from '../common/Badge';
import { SectionLabel } from '../common/SectionLabel';
import styles from './RightInspector.module.css';

export const RightInspector: React.FC = () => {
  const isInspectorOpen = useConsoleStore((state) => state.isInspectorOpen);
  const meta = useFrameMetadata();

  if (!isInspectorOpen) {
    return null;
  }

  const {
    frameSeq,
    scenarioTitle,
    scenarioDescription,
    pipelineStatus,
    sourceStatus,
    pointCount,
    resolutionSummary,
    semanticSummary,
    dynamicSummary,
    dynamicClustersSummary,
  } = meta;

  const severity = pipelineStatusSeverity(pipelineStatus);
  const statusColor =
    severity === 'healthy'
      ? 'var(--status-healthy)'
      : severity === 'degraded' || severity === 'rejected'
      ? 'var(--status-warning)'
      : 'var(--status-critical)';

  return (
    <aside className={styles.inspector} aria-label="Perception Inspector">
      <div className={styles.headerRow}>
        <span className={styles.title}>FRAME INSPECTOR</span>
        <Badge variant="cyan">#{frameSeq}</Badge>
      </div>

      {/* Scenario Overview */}
      <div className={styles.section}>
        <SectionLabel title="Scenario Context" />
        <div className={styles.descriptionBox}>
          <div style={{ fontWeight: 600, color: 'var(--text-primary)', marginBottom: 4 }}>
            {scenarioTitle}
          </div>
          <div>{scenarioDescription}</div>
        </div>
      </div>

      {/* Status & Pipeline Mode */}
      <div className={styles.section}>
        <SectionLabel title="Execution Status" />
        <div className={styles.grid2}>
          <div className={styles.card}>
            <span className={styles.cardLabel}>PIPELINE</span>
            <span className={styles.cardValue} style={{ color: statusColor }}>
              {pipelineStatusLabel(pipelineStatus)}
            </span>
          </div>
          <div className={styles.card}>
            <span className={styles.cardLabel}>SOURCE</span>
            <span className={styles.cardValue}>{sourceStatus}</span>
          </div>
        </div>
      </div>

      {/* Resolution Distribution */}
      <div className={styles.section}>
        <SectionLabel title="Resolution Hierarchy" extra={`${resolutionSummary.totalCells} cells`} />
        <div className={styles.distributionTable}>
          <div className={styles.distributionRow}>
            <div className={styles.distLabel}>
              <span className={styles.colorSwatch} style={{ backgroundColor: resolutionColor(0) }} />
              <span>{resolutionLabel(0)}</span>
            </div>
            <span className={styles.distVal}>{resolutionSummary.l0Count} ({resolutionSummary.l0Percent}%)</span>
          </div>
          <div className={styles.distributionRow}>
            <div className={styles.distLabel}>
              <span className={styles.colorSwatch} style={{ backgroundColor: resolutionColor(1) }} />
              <span>{resolutionLabel(1)}</span>
            </div>
            <span className={styles.distVal}>{resolutionSummary.l1Count} ({resolutionSummary.l1Percent}%)</span>
          </div>
          <div className={styles.distributionRow}>
            <div className={styles.distLabel}>
              <span className={styles.colorSwatch} style={{ backgroundColor: resolutionColor(2) }} />
              <span>{resolutionLabel(2)}</span>
            </div>
            <span className={styles.distVal}>{resolutionSummary.l2Count} ({resolutionSummary.l2Percent}%)</span>
          </div>
          <div className={styles.distributionRow}>
            <div className={styles.distLabel}>
              <span className={styles.colorSwatch} style={{ backgroundColor: resolutionColor(3) }} />
              <span>{resolutionLabel(3)}</span>
            </div>
            <span className={styles.distVal}>{resolutionSummary.l3Count} ({resolutionSummary.l3Percent}%)</span>
          </div>
        </div>
      </div>

      {/* Dynamic Clusters Summary */}
      <div className={styles.section}>
        <SectionLabel
          title="Dynamic Clusters"
          extra={`${dynamicSummary.clusterCount} candidate(s)`}
        />
        {dynamicClustersSummary.length === 0 ? (
          <div className={styles.descriptionBox} style={{ color: 'var(--text-muted)' }}>
            No confirmed dynamic objects in current scene.
          </div>
        ) : (
          <div className={styles.clusterList}>
            {dynamicClustersSummary.map((cluster) => (
              <div key={cluster.clusterLocalId} className={styles.clusterCard}>
                <div className={styles.clusterHeader}>
                  <span>[C-0{cluster.clusterLocalId}] {cluster.dominantClass}</span>
                  <Badge variant="coral">FRAME-LOCAL</Badge>
                </div>
                <div className={styles.clusterMeta}>
                  Points: {cluster.pointCount} | Conf: {(cluster.meanSemanticConfidence * 100).toFixed(0)}%
                </div>
                <div className={styles.clusterMeta}>
                  AABB: [{(cluster.aabbMax.x - cluster.aabbMin.x).toFixed(1)}m × {(cluster.aabbMax.y - cluster.aabbMin.y).toFixed(1)}m]
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Semantic Class Summary */}
      <div className={styles.section}>
        <SectionLabel title="Semantic Distribution" extra={`${pointCount} points`} />
        <div className={styles.distributionTable}>
          {(Object.keys(semanticSummary) as SemanticClass[])
            .filter((cls) => semanticSummary[cls] > 0)
            .map((cls) => (
              <div key={cls} className={styles.distributionRow}>
                <div className={styles.distLabel}>
                  <span className={styles.colorSwatch} style={{ backgroundColor: semanticClassColor(cls) }} />
                  <span>{semanticClassLabel(cls)}</span>
                </div>
                <span className={styles.distVal}>{semanticSummary[cls]}</span>
              </div>
            ))}
        </div>
      </div>
    </aside>
  );
};
