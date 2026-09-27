import React, { useEffect, useState } from 'react';
import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { PerceptionFrame } from '../types/perception';
import { resolutionLabel } from '../utils/resolution';
import { Badge } from '../components/common/Badge';
import styles from './ComparePage.module.css';

export const ComparePage: React.FC = () => {
  const [frame, setFrame] = useState<PerceptionFrame>(() => frameRuntime.getLatestFrame());

  useEffect(() => {
    return frameRuntime.subscribeFrame((f) => setFrame(f));
  }, []);

  const { cells, resolutionSummary } = frame;

  // Compute bounding box extent of active cells
  let minX = 0;
  let maxX = 0;
  let minY = 0;
  let maxY = 0;

  if (cells.length > 0) {
    minX = Math.min(...cells.map((c) => c.x - c.sizeMeters / 2));
    maxX = Math.max(...cells.map((c) => c.x + c.sizeMeters / 2));
    minY = Math.min(...cells.map((c) => c.y - c.sizeMeters / 2));
    maxY = Math.max(...cells.map((c) => c.y + c.sizeMeters / 2));
  }

  const widthM = Math.max(1, Math.round(maxX - minX));
  const heightM = Math.max(1, Math.round(maxY - minY));

  // 5 cm uniform reference grid calculation over same bounding extent
  const uniform5cmCells = Math.round((widthM / 0.05) * (heightM / 0.05));
  const adaptiveActiveCells = cells.length;

  return (
    <div className={styles.pageContainer}>
      <div className={styles.header}>
        <div className={styles.title}>TOPOLOGY COMPARISON (ILLUSTRATIVE DEMO MODEL)</div>
        <div className={styles.subtitle}>
          Comparing active adaptive tree cells against a uniform 5 cm fine grid over the active scene extent.
        </div>
      </div>

      <div className={styles.noticeBox}>
        <strong>DEMO TOPOLOGY CELL COUNT:</strong> This deterministic demo topology uses {adaptiveActiveCells} adaptive cells versus {uniform5cmCells.toLocaleString()} uniform 5 cm reference cells over a {widthM}m × {heightM}m active extent ({frame.scenarioTitle}). All values are derived deterministically from fixture topology. No generalized performance claim.
      </div>

      <div className={styles.comparisonGrid}>
        {/* Uniform Grid Reference */}
        <div className={styles.compareCard}>
          <div className={styles.cardHeader}>
            <span className={styles.cardTitle}>Uniform 5 cm Reference Grid</span>
            <Badge variant="neutral">ILLUSTRATIVE BASELINE</Badge>
          </div>

          <div className={styles.metricsRow}>
            <div className={styles.statItem}>
              <span className={styles.statLabel}>Resolution</span>
              <span className={styles.statVal}>0.05 m (Fixed)</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statLabel}>Calculated Cells</span>
              <span className={styles.statVal}>{uniform5cmCells.toLocaleString()}</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statLabel}>Hierarchy Levels</span>
              <span className={styles.statVal}>1 (Flat Array)</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statLabel}>Scene Bounds</span>
              <span className={styles.statVal}>{widthM}m × {heightM}m</span>
            </div>
          </div>

          <div className={styles.detailsList}>
            <div className={styles.detailsItem}>
              <span className={styles.bullet}>▸</span>
              <span>Allocates high resolution uniformly across open ground, benign roads, and sky.</span>
            </div>
            <div className={styles.detailsItem}>
              <span className={styles.bullet}>▸</span>
              <span>Lacks multi-scale semantic or dynamic awareness.</span>
            </div>
            <div className={styles.detailsItem}>
              <span className={styles.bullet}>▸</span>
              <span>Theoretical maximum cell count without adaptive tree pruning.</span>
            </div>
          </div>
        </div>

        {/* Adaptive Variable-Resolution Dyadic Tree */}
        <div className={styles.compareCard}>
          <div className={styles.cardHeader}>
            <span className={styles.cardTitle}>AVR Dyadic Multi-Resolution</span>
            <Badge variant="cyan">ADAPTIVE CONSOLE</Badge>
          </div>

          <div className={styles.metricsRow}>
            <div className={styles.statItem}>
              <span className={styles.statLabel}>Resolution</span>
              <span className={styles.statVal}>0.05m – 0.40m</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statLabel}>Active Demo Cells</span>
              <span className={styles.statVal} style={{ color: 'var(--accent-cyan)' }}>
                {adaptiveActiveCells}
              </span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statLabel}>Hierarchy Levels</span>
              <span className={styles.statVal}>4 (L0, L1, L2, L3)</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statLabel}>Topology</span>
              <span className={styles.statVal}>Dyadic Quadtree</span>
            </div>
          </div>

          <div className={styles.detailsList}>
            <div className={styles.detailsItem}>
              <span className={styles.bullet}>▸</span>
              <span>
                <strong>{resolutionLabel(0)}:</strong> {resolutionSummary.l0Count} cells ({resolutionSummary.l0Percent}%) — Focused on hazards and high geometry complexity.
              </span>
            </div>
            <div className={styles.detailsItem}>
              <span className={styles.bullet}>▸</span>
              <span>
                <strong>{resolutionLabel(1)}:</strong> {resolutionSummary.l1Count} cells ({resolutionSummary.l1Percent}%) — Near obstacle boundaries.
              </span>
            </div>
            <div className={styles.detailsItem}>
              <span className={styles.bullet}>▸</span>
              <span>
                <strong>{resolutionLabel(2)} & {resolutionLabel(3)}:</strong> {resolutionSummary.l2Count + resolutionSummary.l3Count} cells ({resolutionSummary.l2Percent + resolutionSummary.l3Percent}%) — Efficient coverage for benign roadway and background.
              </span>
            </div>
            <div className={styles.detailsItem}>
              <span className={styles.bullet}>▸</span>
              <span>Refinement driven by frozen backend reasons: SEMANTIC_RELEVANCE, DYNAMIC_RELEVANCE, DISTANCE_BASELINE.</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};
