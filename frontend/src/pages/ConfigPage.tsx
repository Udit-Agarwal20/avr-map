import React from 'react';
import { Badge } from '../components/common/Badge';
import { resolutionLabel, resolutionMeters } from '../utils/resolution';
import styles from './ConfigPage.module.css';

export const ConfigPage: React.FC = () => {
  return (
    <div className={styles.pageContainer}>
      <div className={styles.header}>
        <div className={styles.title}>PIPELINE CONFIGURATION PARAMETERS</div>
        <div className={styles.subtitle}>
          Read-only inspection of frozen backend parameters, resolution thresholds, and dynamic cluster criteria.
        </div>
      </div>

      <div className={styles.grid2}>
        {/* Dyadic Multi-Resolution Levels */}
        <div className={styles.card}>
          <div className={styles.cardHeader}>
            <span className={styles.cardTitle}>Dyadic Resolution Bands</span>
            <Badge variant="cyan">CORE CONSTANTS</Badge>
          </div>

          <div className={styles.configTable}>
            <div className={styles.configRow}>
              <span className={styles.configKey}>{resolutionLabel(0)}</span>
              <span className={styles.configVal}>{resolutionMeters(0).toFixed(2)} m</span>
            </div>
            <div className={styles.configRow}>
              <span className={styles.configKey}>{resolutionLabel(1)}</span>
              <span className={styles.configVal}>{resolutionMeters(1).toFixed(2)} m</span>
            </div>
            <div className={styles.configRow}>
              <span className={styles.configKey}>{resolutionLabel(2)}</span>
              <span className={styles.configVal}>{resolutionMeters(2).toFixed(2)} m</span>
            </div>
            <div className={styles.configRow}>
              <span className={styles.configKey}>{resolutionLabel(3)}</span>
              <span className={styles.configVal}>{resolutionMeters(3).toFixed(2)} m</span>
            </div>
          </div>
        </div>

        {/* Dynamic Clustering Parameters */}
        <div className={styles.card}>
          <div className={styles.cardHeader}>
            <span className={styles.cardTitle}>Dynamic Clustering Engine</span>
            <Badge variant="coral">FRAME-LOCAL</Badge>
          </div>

          <div className={styles.configTable}>
            <div className={styles.configRow}>
              <span className={styles.configKey}>Spatial Neighbor Threshold (Epsilon)</span>
              <span className={styles.configVal}>0.60 m</span>
            </div>
            <div className={styles.configRow}>
              <span className={styles.configKey}>Min Points Per Cluster</span>
              <span className={styles.configVal}>4 points</span>
            </div>
            <div className={styles.configRow}>
              <span className={styles.configKey}>Max Cluster Distance</span>
              <span className={styles.configVal}>50.0 m</span>
            </div>
            <div className={styles.configRow}>
              <span className={styles.configKey}>Temporal Tracking State</span>
              <span className={styles.configVal} style={{ color: 'var(--text-muted)' }}>
                EXCLUDED (M10C2 Future)
              </span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};
