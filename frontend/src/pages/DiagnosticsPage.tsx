import React, { useEffect, useState } from 'react';
import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { PerceptionFrame } from '../types/perception';
import { pipelineStatusLabel, pipelineStatusSeverity } from '../utils/status';
import { Badge } from '../components/common/Badge';
import styles from './DiagnosticsPage.module.css';

export const DiagnosticsPage: React.FC = () => {
  const [frame, setFrame] = useState<PerceptionFrame>(() => frameRuntime.getLatestFrame());

  useEffect(() => {
    return frameRuntime.subscribeFrame((f) => setFrame(f));
  }, []);

  const { points, dynamicPoints, dynamicClusters, noiseSourceIndices, sensorOrigin } = frame;

  const staticPointsCount = points.filter((p) => p.routing === 'STATIC_FUSABLE').length;
  const isConservationValid = staticPointsCount + dynamicPoints.length === points.length;

  const severity = pipelineStatusSeverity(frame.pipelineStatus);
  const statusColor =
    severity === 'healthy'
      ? 'var(--status-healthy)'
      : severity === 'degraded' || severity === 'rejected'
      ? 'var(--status-warning)'
      : 'var(--status-critical)';

  return (
    <div className={styles.pageContainer}>
      <div className={styles.header}>
        <div className={styles.title}>SYSTEM & PERCEPTION DIAGNOSTICS</div>
        <div className={styles.subtitle}>
          Operational integrity, point conservation audits, and data source health indicators.
        </div>
      </div>

      <div className={styles.noticeBox}>
        <strong>DEMO PLAYBACK: 10 Hz</strong> (Deterministic UI playback cadence, not real-time backend hardware measurement). Backend connection mode: DETERMINISTIC FIXTURE.
      </div>

      <div className={styles.grid2}>
        {/* Pipeline Health */}
        <div className={styles.card}>
          <div className={styles.cardHeader}>
            <span className={styles.cardTitle}>Pipeline Status Contract</span>
            <Badge variant="cyan">{frame.pipelineStatus}</Badge>
          </div>

          <div className={styles.statusTable}>
            <div className={styles.statusRow}>
              <span className={styles.statusKey}>Current Frame Sequence</span>
              <span className={styles.statusVal}>#{frame.frameSeq}</span>
            </div>
            <div className={styles.statusRow}>
              <span className={styles.statusKey}>Active Scenario</span>
              <span className={styles.statusVal}>{frame.scenarioTitle}</span>
            </div>
            <div className={styles.statusRow}>
              <span className={styles.statusKey}>Execution State</span>
              <span className={styles.statusVal} style={{ color: statusColor }}>
                {pipelineStatusLabel(frame.pipelineStatus)}
              </span>
            </div>
            <div className={styles.statusRow}>
              <span className={styles.statusKey}>Data Source Provider</span>
              <span className={styles.statusVal}>FixturePerceptionSource</span>
            </div>
            <div className={styles.statusRow}>
              <span className={styles.statusKey}>Sensor Origin (x, y, z)</span>
              <span className={styles.statusVal}>
                [{sensorOrigin.x.toFixed(1)}, {sensorOrigin.y.toFixed(1)}, {sensorOrigin.z.toFixed(1)}] m
              </span>
            </div>
          </div>
        </div>

        {/* Invariant & Point Conservation Check */}
        <div className={styles.card}>
          <div className={styles.cardHeader}>
            <span className={styles.cardTitle}>Point Routing Conservation</span>
            <Badge variant={isConservationValid ? 'cyan' : 'coral'}>
              {isConservationValid ? 'INVARIANT PASS' : 'INTEGRITY FAIL'}
            </Badge>
          </div>

          <div className={styles.statusTable}>
            <div className={styles.statusRow}>
              <span className={styles.statusKey}>Total Ingested Points</span>
              <span className={styles.statusVal}>{points.length}</span>
            </div>
            <div className={styles.statusRow}>
              <span className={styles.statusKey}>STATIC_FUSABLE Points</span>
              <span className={styles.statusVal}>{staticPointsCount}</span>
            </div>
            <div className={styles.statusRow}>
              <span className={styles.statusKey}>DYNAMIC_EXCLUDED Points</span>
              <span className={styles.statusVal} style={{ color: 'var(--dynamic-coral)' }}>
                {dynamicPoints.length}
              </span>
            </div>
            <div className={styles.statusRow}>
              <span className={styles.statusKey}>Frame-Local Dynamic Clusters</span>
              <span className={styles.statusVal}>{dynamicClusters.length}</span>
            </div>
            <div className={styles.statusRow}>
              <span className={styles.statusKey}>Noise / Suppressed Indices</span>
              <span className={styles.statusVal}>{noiseSourceIndices.length}</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};
