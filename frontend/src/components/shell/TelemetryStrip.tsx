import React from 'react';
import { useFrameMetadata } from '../../runtime/useFrameMetadata';
import styles from './TelemetryStrip.module.css';

export const TelemetryStrip: React.FC = () => {
  const meta = useFrameMetadata();
  const { resolutionSummary, dynamicSummary } = meta;

  return (
    <footer className={styles.telemetryStrip} aria-label="System Telemetry">
      <div className={styles.leftGroup}>
        <div className={styles.item}>
          <span className={styles.itemLabel}>FRAME</span>
          <span className={styles.itemValue}>
            #{meta.frameSeq.toString().padStart(5, '0')}
          </span>
        </div>

        <div className={styles.item}>
          <span className={styles.itemLabel}>ACTIVE CELLS</span>
          <span className={styles.itemValue}>{resolutionSummary.totalCells}</span>
        </div>

        {/* Resolution Breakdown */}
        <div className={styles.item}>
          <span className={styles.itemLabel}>RESOLUTION:</span>
          <span className={`${styles.resPill} ${styles.resL0}`}>
            L0 (5cm): {resolutionSummary.l0Percent}%
          </span>
          <span className={`${styles.resPill} ${styles.resL1}`}>
            L1 (10cm): {resolutionSummary.l1Percent}%
          </span>
          <span className={`${styles.resPill} ${styles.resL2}`}>
            L2 (20cm): {resolutionSummary.l2Percent}%
          </span>
          <span className={`${styles.resPill} ${styles.resL3}`}>
            L3 (40cm): {resolutionSummary.l3Percent}%
          </span>
        </div>
      </div>

      <div className={styles.rightGroup}>
        <div className={styles.item}>
          <span className={styles.itemLabel}>DYNAMIC POINTS</span>
          <span className={styles.itemValue}>{dynamicSummary.dynamicPoints}</span>
        </div>

        <div className={styles.item}>
          <span className={styles.itemLabel}>FRAME-LOCAL CLUSTERS</span>
          <span
            className={`${styles.itemValue} ${
              dynamicSummary.clusterCount > 0 ? styles.dynamicAlert : ''
            }`}
          >
            {dynamicSummary.clusterCount}
          </span>
        </div>
      </div>
    </footer>
  );
};
