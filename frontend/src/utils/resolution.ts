import { containsPoint } from './topology';
import type {
  AdaptiveCell,
  PerceptionFrame,
  ResolutionExplanation,
  ResolutionLevel,
  ResolutionReason,
  SemanticClass,
} from '../types/perception';

/**
 * @file resolution.ts
 * @brief Authoritative resolution helpers aligning with frozen C++ AVR mapping core.
 *
 * Implements frozen ResolutionPolicyConfig defaults:
 *   [0, 10.0 m)   -> L0 (5 cm)
 *   [10.0, 30.0 m) -> L1 (10 cm)
 *   [30.0, 60.0 m) -> L2 (20 cm)
 *   [60.0 m, inf) -> L3 (40 cm)
 *
 * CRITICAL ARCHITECTURAL CONTRACT:
 * Strictly mirrors C++ backend EvidenceSignalBuilder::makeResolutionSignals:
 *   const auto bounds = region.bounds(); // Canonical GridOrigin2D{0, 0}
 *   const double center_x = (bounds.min_x + bounds.max_x) * 0.5;
 *   const double center_y = (bounds.min_y + bounds.max_y) * 0.5;
 *   const auto sensor_origin = evidence.sensorOrigin(); // WorldPoint2D { x, y }
 *   const double dx = center_x - sensor_origin.x;
 *   const double dy = center_y - sensor_origin.y;
 *   const double distance_m = std::hypot(dx, dy);
 *
 * The policy distance is strictly 2D planar horizontal distance (XY).
 * Z coordinate, sensor height, and cell elevation are STRICTLY INVARIANT and EXCLUDED.
 *
 * IMPORTANT ARCHITECTURAL RULE:
 * The baselineLevelForDistance helper exists EXCLUSIVELY for deterministic demo
 * explainability and fixture validation. In future LIVE deployment, AVR Console
 * MUST consume the authoritative backend-provided resolution decision rather
 * than independently recomputing it.
 */

/**
 * @brief Computes 2D horizontal planar distance (XY) between cell center and sensor origin.
 */
export function planarDistanceFromSensor(
  cell: { x: number; y: number },
  sensorOrigin: { x: number; y: number }
): number {
  const dx = cell.x - sensorOrigin.x;
  const dy = cell.y - sensorOrigin.y;
  return Math.hypot(dx, dy);
}

export const resolutionDistanceMeters = planarDistanceFromSensor;

/**
 * @brief Derives authoritative explainability DTO from current-frame perception points.
 *
 * Semantic evidence is derived strictly from fixture perception points spatially falling into
 * the cell's footprint, evaluated using frozen backend priority:
 *   SAFETY_CRITICAL (PEDESTRIAN, VEHICLE) > ELEVATED (BUILDING, VEGETATION, OBSTACLE_OTHER) > BENIGN/NONE.
 */
export function explainCellResolution(
  cell: AdaptiveCell,
  frame: PerceptionFrame
): ResolutionExplanation {
  const sensorOrigin = frame.sensorOrigin || { x: 0, y: 0, z: 0 };
  const policyDistanceMeters = planarDistanceFromSensor(cell, sensorOrigin);
  const baselineLevel = baselineLevelForDistance(policyDistanceMeters);

  // Spatially query current-frame perception points falling strictly into cell footprint
  const cellPoints = frame.points.filter(p => containsPoint(cell, p));

  // Evaluate semantic class priority
  let dominantClass: SemanticClass | undefined;
  let dominantTier: 'SAFETY_CRITICAL' | 'ELEVATED' | 'BENIGN' | undefined;

  for (const pt of cellPoints) {
    if (pt.semanticClass === 'PEDESTRIAN' || pt.semanticClass === 'VEHICLE') {
      dominantClass = pt.semanticClass;
      dominantTier = 'SAFETY_CRITICAL';
      break; // Highest possible priority reached
    } else if (
      pt.semanticClass === 'BUILDING' ||
      pt.semanticClass === 'VEGETATION' ||
      pt.semanticClass === 'OBSTACLE_OTHER'
    ) {
      if (dominantTier !== 'SAFETY_CRITICAL') {
        dominantClass = pt.semanticClass;
        dominantTier = 'ELEVATED';
      }
    } else if (!dominantClass) {
      dominantClass = pt.semanticClass;
      dominantTier = 'BENIGN';
    }
  }

  // Evaluate dynamic relevance
  const isDynamic =
    cellPoints.some((p) => p.routing === 'DYNAMIC_EXCLUDED') ||
    cell.resolutionReasons.includes('DYNAMIC_RELEVANCE');

  // Identify dynamic cluster if present
  let dynamicClusterId: number | undefined;
  if (isDynamic) {
    const matchedCluster = frame.dynamicClusters.find((cl) =>
      cellPoints.some((cp) => cl.memberSourceIndices.includes(cp.sourceIndex))
    );
    dynamicClusterId = matchedCluster?.clusterLocalId;
  }

  // Evaluate point routing and static-fusion status
  const allDynamic = cellPoints.length > 0 && cellPoints.every((p) => p.routing === 'DYNAMIC_EXCLUDED');
  const anyDynamic = cellPoints.some((p) => p.routing === 'DYNAMIC_EXCLUDED');
  const anyStatic = cellPoints.some((p) => p.routing === 'STATIC_FUSABLE');

  const routing: 'DYNAMIC_EXCLUDED' | 'STATIC_FUSABLE' | 'MIXED' | undefined =
    cellPoints.length > 0
      ? allDynamic
        ? 'DYNAMIC_EXCLUDED'
        : anyStatic && !anyDynamic
        ? 'STATIC_FUSABLE'
        : 'MIXED'
      : undefined;

  // FROZEN BACKEND RULE: Confirmed dynamic / DYNAMIC_EXCLUDED points contribute to
  // frame-local decision geometry and dynamic clustering, but are strictly excluded
  // from persistent static map elevation terrain fusion.
  const staticFusion = anyStatic ? (anyDynamic ? 'MIXED' : 'FUSABLE') : allDynamic ? 'EXCLUDED' : 'NO_EVIDENCE';

  // Only permit static elevation stats if cell has verified static-fusable support and is not dynamic excluded
  const hasStaticElevationSupport =
    staticFusion === 'FUSABLE' && cell.elevationMean !== undefined;

  const staticElevationStats = hasStaticElevationSupport && cell.elevationMean !== undefined
    ? {
        mean: cell.elevationMean,
        min: cell.elevationMin ?? cell.elevationMean,
        max: cell.elevationMax ?? cell.elevationMean,
      }
    : undefined;

  // Decision evidence description
  const hasSemantic = cell.resolutionReasons.includes('SEMANTIC_RELEVANCE') || Boolean(dominantClass);
  let decisionEvidence = 'Distance baseline geometry';
  if (hasSemantic && isDynamic) {
    decisionEvidence = dominantClass ? `Semantic + dynamic ${dominantClass.toLowerCase()} evidence` : 'Authored demo refinement region; no current-frame returns';
  } else if (hasSemantic && dominantClass) {
    decisionEvidence = `Semantic ${dominantClass.toLowerCase()} evidence`;
  } else if (isDynamic) {
    decisionEvidence = 'Dynamic hazard evidence';
  }

  return {
    cellId: cell.cellId,
    policyDistanceMeters,
    baselineLevel,
    finalLevel: cell.level,
    reasons: [...cell.resolutionReasons],
    pointCount: cellPoints.length,
    semanticClass: dominantClass,
    semanticPolicyTier: dominantTier,
    dynamicRelevance: isDynamic,
    routing,
    decisionEvidence,
    staticFusion,
    hasStaticElevationSupport,
    sourceIndices: cellPoints.length > 0 ? cellPoints.map((p) => p.sourceIndex) : undefined,
    dynamicClusterId,
    staticElevationStats,
  };
}

/**
 * @brief Computes distance baseline resolution level for a given radial distance (meters).
 * Matches frozen C++ backend ResolutionPolicyConfig defaults exactly.
 */
export function baselineLevelForDistance(distanceMeters: number): ResolutionLevel {
  if (distanceMeters < 0 || !Number.isFinite(distanceMeters)) {
    throw new RangeError(`Distance must be a finite, non-negative number: ${distanceMeters}`);
  }
  if (distanceMeters < 10.0) {
    return 0; // L0 (5 cm)
  }
  if (distanceMeters < 30.0) {
    return 1; // L1 (10 cm)
  }
  if (distanceMeters < 60.0) {
    return 2; // L2 (20 cm)
  }
  return 3; // L3 (40 cm)
}

export function resolutionMeters(level: ResolutionLevel): number {
  switch (level) {
    case 0:
      return 0.05;
    case 1:
      return 0.10;
    case 2:
      return 0.20;
    case 3:
      return 0.40;
  }
}

export function resolutionLabel(level: ResolutionLevel): string {
  switch (level) {
    case 0:
      return 'L0 (5 cm)';
    case 1:
      return 'L1 (10 cm)';
    case 2:
      return 'L2 (20 cm)';
    case 3:
      return 'L3 (40 cm)';
  }
}

export function resolutionColor(level: ResolutionLevel): string {
  switch (level) {
    case 0:
      return '#62D3C6'; // Turquoise
    case 1:
      return '#70B8E8'; // Blue
    case 2:
      return '#D6B45F'; // Amber / Sand
    case 3:
      return '#A18AC8'; // Lavender
  }
}

export function resolutionReasonLabel(reason: ResolutionReason): string {
  switch (reason) {
    case 'DISTANCE_BASELINE':
      return 'Distance Baseline';
    case 'SEMANTIC_RELEVANCE':
      return 'Semantic Relevance';
    case 'DYNAMIC_RELEVANCE':
      return 'Dynamic Relevance';
    case 'UNCERTAINTY':
      return 'Uncertainty';
    case 'GEOMETRY_COMPLEXITY':
      return 'Geometry Complexity';
    case 'INSUFFICIENT_EVIDENCE':
      return 'Insufficient Evidence';
    case 'DENSITY_GATE_BLOCKED':
      return 'Density Gate';
  }
}
