import React from 'react';
import { Badge } from '../components/common/Badge';
import styles from './ArchitecturePage.module.css';

export const ArchitecturePage: React.FC = () => {
  return (
    <div className={styles.pageContainer}>
      <div className={styles.header}>
        <div className={styles.title}>AVR-MAP PIPELINE ARCHITECTURE</div>
        <div className={styles.subtitle}>
          System architecture and dataflow across perception ingestion, routing, clustering, and adaptive multi-resolution grid.
        </div>
      </div>

      <div className={styles.pipelineFlow}>
        {/* Stage 1 */}
        <div className={styles.stageCard}>
          <div className={styles.stageHeader}>
            <span className={styles.stageTitle}>
              <span className={styles.stepNumber}>01</span>
              <span>POINT INGESTION & COORDINATE TRANSFORMATION</span>
            </span>
            <Badge variant="cyan">RAW SENSOR</Badge>
          </div>
          <div className={styles.stageBody}>
            Point clouds arrive in sensor frame and undergo rigid transform to local vehicle reference frame. Coordinates normalized to 2.5D elevation and coordinate grid.
          </div>
          <div className={styles.stageMeta}>
            <span>INPUT: Raw LiDAR Point Stream</span>
            <span>CONTRACT: Point3D (x, y, z)</span>
          </div>
        </div>

        <div className={styles.flowArrow}>▼</div>

        {/* Stage 2 */}
        <div className={styles.stageCard}>
          <div className={styles.stageHeader}>
            <span className={styles.stageTitle}>
              <span className={styles.stepNumber}>02</span>
              <span>SEMANTIC PERCEPTION PROVIDER</span>
            </span>
            <Badge variant="cyan">TAXONOMY</Badge>
          </div>
          <div className={styles.stageBody}>
            Deterministic mapping of points into 8 canonical classes: UNKNOWN, GROUND, ROAD, VEGETATION, BUILDING, VEHICLE, PEDESTRIAN, OBSTACLE_OTHER. Confidences assigned per point.
          </div>
          <div className={styles.stageMeta}>
            <span>OUTPUT: SemanticTaggedPoint</span>
            <span>CONFIDENCE: [0.0, 1.0]</span>
          </div>
        </div>

        <div className={styles.flowArrow}>▼</div>

        {/* Stage 3 */}
        <div className={styles.stageCard}>
          <div className={styles.stageHeader}>
            <span className={styles.stageTitle}>
              <span className={styles.stepNumber}>03</span>
              <span>REPLAY-SAFE POINT ROUTING</span>
            </span>
            <Badge variant="coral">STRICT PARTITION</Badge>
          </div>
          <div className={styles.stageBody}>
            Strict classification into STATIC_FUSABLE (eligible for persistent static map evidence integration) and DYNAMIC_EXCLUDED (strictly excluded from static elevation grid to prevent smear artifacts).
          </div>
          <div className={styles.stageMeta}>
            <span>ROUTING: STATIC_FUSABLE | DYNAMIC_EXCLUDED</span>
            <span>GUARANTEE: Deterministic, Replay-Safe</span>
          </div>
        </div>

        <div className={styles.flowArrow}>▼</div>

        {/* Stage 4 */}
        <div className={styles.stageCard}>
          <div className={styles.stageHeader}>
            <span className={styles.stageTitle}>
              <span className={styles.stepNumber}>04</span>
              <span>DYNAMIC OBJECT CLUSTERING</span>
            </span>
            <Badge variant="coral">FRAME-LOCAL</Badge>
          </div>
          <div className={styles.stageBody}>
            DynamicClusterer performs spatial clustering on DYNAMIC_EXCLUDED points to produce frame-local object proposals [C-00..C-xx]. No temporal tracking, trajectory, or velocity estimation.
          </div>
          <div className={styles.stageMeta}>
            <span>OUTPUT: DynamicObjectCluster</span>
            <span>SCOPE: Frame-Local Bounding Proposals</span>
          </div>
        </div>

        <div className={styles.flowArrow}>▼</div>

        {/* Stage 5 */}
        <div className={styles.stageCard}>
          <div className={styles.stageHeader}>
            <span className={styles.stageTitle}>
              <span className={styles.stepNumber}>05</span>
              <span>ADAPTIVE DYADIC TREE & RESOLUTION DECISION</span>
            </span>
            <Badge variant="cyan">VARIABLE RESOLUTION</Badge>
          </div>
          <div className={styles.stageBody}>
            Quadtree structure refines from L3 (0.40m) down to L0 (0.05m) based on 7 frozen backend reasons: DISTANCE_BASELINE, SEMANTIC_RELEVANCE, DYNAMIC_RELEVANCE, UNCERTAINTY, GEOMETRY_COMPLEXITY, INSUFFICIENT_EVIDENCE, DENSITY_GATE_BLOCKED.
          </div>
          <div className={styles.stageMeta}>
            <span>LEVELS: L0 (0.05m), L1 (0.10m), L2 (0.20m), L3 (0.40m)</span>
            <span>COARSENING: Automatic Adaptive Pruning</span>
          </div>
        </div>
      </div>
    </div>
  );
};
