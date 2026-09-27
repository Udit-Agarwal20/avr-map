/**
 * @file perception.ts
 * @brief Authoritative frontend data contracts mirroring AVR-Map frozen C++ core (HEAD: cc50675).
 *
 * Strict Data-Truth Contract:
 * - NO TrackId, velocity, trajectory, Kalman, or Hungarian tracking fields.
 * - Clusters are frame-local single-frame candidates only.
 * - Canonical resolution reasons match backend C++ enum names and bit values.
 * - Pipeline status enum strictly matches backend uint8_t 0..21.
 */

export type PointRouting = 'STATIC_FUSABLE' | 'DYNAMIC_EXCLUDED';

/**
 * @brief Categorical execution outcome of processing a frame (Frozen C++ AdaptivePipelineStatus : uint8_t).
 */
export enum AdaptivePipelineStatus {
  SUCCESS = 0,
  SUCCESS_REPLAY_PENDING = 1,
  SUCCESS_CACHE_DEGRADED = 2,
  FRAME_REJECTED_INPUT = 3,
  FRAME_REJECTED_SEQUENCE = 4,
  FRAME_REJECTED_DECISION = 5,
  FRAME_REJECTED_POLICY = 6,
  REFINEMENT_INTERNAL_ERROR = 7,
  REPLAY_INTERNAL_ERROR = 8,
  FUSION_FAILED = 9,
  FUSION_FAILED_AFTER_STRUCTURAL_CHANGE = 10,
  POLICY_COMMIT_INTERNAL_ERROR = 11,
  POLICY_COMMIT_FATAL = 12,
  CACHE_COMMIT_INTERNAL_ERROR = 13,
  GENERATION_EXHAUSTED = 14,
  POLICY_REVISION_EXHAUSTED = 15,
  INVARIANT_FAILURE = 16,
  PIPELINE_FATAL_LATCHED = 17,
  COARSENING_INTERNAL_ERROR = 18,
  FRAME_REJECTED_SEMANTIC_MALFORMED = 19,
  FRAME_REJECTED_SEMANTIC_ERROR = 20,
  FRAME_REJECTED_SEMANTIC_RESOURCE = 21,
}

export type AdaptivePipelineStatusName = keyof typeof AdaptivePipelineStatus;

// Compile-time contract locks asserting exact frozen numeric mappings (0..21)
// Literal-type locks: value drifts immediately fail TypeScript compilation.
export const _LOCK_STATUS_0: 0 = AdaptivePipelineStatus.SUCCESS;
export const _LOCK_STATUS_1: 1 = AdaptivePipelineStatus.SUCCESS_REPLAY_PENDING;
export const _LOCK_STATUS_2: 2 = AdaptivePipelineStatus.SUCCESS_CACHE_DEGRADED;
export const _LOCK_STATUS_3: 3 = AdaptivePipelineStatus.FRAME_REJECTED_INPUT;
export const _LOCK_STATUS_4: 4 = AdaptivePipelineStatus.FRAME_REJECTED_SEQUENCE;
export const _LOCK_STATUS_5: 5 = AdaptivePipelineStatus.FRAME_REJECTED_DECISION;
export const _LOCK_STATUS_6: 6 = AdaptivePipelineStatus.FRAME_REJECTED_POLICY;
export const _LOCK_STATUS_7: 7 = AdaptivePipelineStatus.REFINEMENT_INTERNAL_ERROR;
export const _LOCK_STATUS_8: 8 = AdaptivePipelineStatus.REPLAY_INTERNAL_ERROR;
export const _LOCK_STATUS_9: 9 = AdaptivePipelineStatus.FUSION_FAILED;
export const _LOCK_STATUS_10: 10 = AdaptivePipelineStatus.FUSION_FAILED_AFTER_STRUCTURAL_CHANGE;
export const _LOCK_STATUS_11: 11 = AdaptivePipelineStatus.POLICY_COMMIT_INTERNAL_ERROR;
export const _LOCK_STATUS_12: 12 = AdaptivePipelineStatus.POLICY_COMMIT_FATAL;
export const _LOCK_STATUS_13: 13 = AdaptivePipelineStatus.CACHE_COMMIT_INTERNAL_ERROR;
export const _LOCK_STATUS_14: 14 = AdaptivePipelineStatus.GENERATION_EXHAUSTED;
export const _LOCK_STATUS_15: 15 = AdaptivePipelineStatus.POLICY_REVISION_EXHAUSTED;
export const _LOCK_STATUS_16: 16 = AdaptivePipelineStatus.INVARIANT_FAILURE;
export const _LOCK_STATUS_17: 17 = AdaptivePipelineStatus.PIPELINE_FATAL_LATCHED;
export const _LOCK_STATUS_18: 18 = AdaptivePipelineStatus.COARSENING_INTERNAL_ERROR;
export const _LOCK_STATUS_19: 19 = AdaptivePipelineStatus.FRAME_REJECTED_SEMANTIC_MALFORMED;
export const _LOCK_STATUS_20: 20 = AdaptivePipelineStatus.FRAME_REJECTED_SEMANTIC_ERROR;
export const _LOCK_STATUS_21: 21 = AdaptivePipelineStatus.FRAME_REJECTED_SEMANTIC_RESOURCE;

// Reverse lock
export const _REV_LOCK_STATUS_0: AdaptivePipelineStatus.SUCCESS = 0;
export const _REV_LOCK_STATUS_21: AdaptivePipelineStatus.FRAME_REJECTED_SEMANTIC_RESOURCE = 21;

/**
 * @brief Canonical resolution decision reasons (Frozen C++ ResolutionReason).
 */
export type ResolutionReason =
  | 'DISTANCE_BASELINE'
  | 'SEMANTIC_RELEVANCE'
  | 'DYNAMIC_RELEVANCE'
  | 'UNCERTAINTY'
  | 'GEOMETRY_COMPLEXITY'
  | 'INSUFFICIENT_EVIDENCE'
  | 'DENSITY_GATE_BLOCKED';

/**
 * @brief Bitmask values matching backend C++ ResolutionReason : uint16_t.
 */
export enum ResolutionReasonBit {
  NONE = 0,
  DISTANCE_BASELINE = 1 << 0,
  SEMANTIC_RELEVANCE = 1 << 1,
  DYNAMIC_RELEVANCE = 1 << 2,
  UNCERTAINTY = 1 << 3,
  GEOMETRY_COMPLEXITY = 1 << 4,
  INSUFFICIENT_EVIDENCE = 1 << 5,
  DENSITY_GATE_BLOCKED = 1 << 6,
}

// Compile-time bitmask literal-type locks
export const _LOCK_BIT_NONE: 0 = ResolutionReasonBit.NONE;
export const _LOCK_BIT_DISTANCE: 1 = ResolutionReasonBit.DISTANCE_BASELINE;
export const _LOCK_BIT_SEMANTIC: 2 = ResolutionReasonBit.SEMANTIC_RELEVANCE;
export const _LOCK_BIT_DYNAMIC: 4 = ResolutionReasonBit.DYNAMIC_RELEVANCE;
export const _LOCK_BIT_UNCERTAINTY: 8 = ResolutionReasonBit.UNCERTAINTY;
export const _LOCK_BIT_GEOMETRY: 16 = ResolutionReasonBit.GEOMETRY_COMPLEXITY;
export const _LOCK_BIT_INSUFFICIENT: 32 = ResolutionReasonBit.INSUFFICIENT_EVIDENCE;
export const _LOCK_BIT_DENSITY: 64 = ResolutionReasonBit.DENSITY_GATE_BLOCKED;

/**
 * @brief Canonical 8 semantic perception classes (Frozen C++ SemanticClass).
 */
export type SemanticClass =
  | 'UNKNOWN'
  | 'GROUND'
  | 'ROAD'
  | 'VEGETATION'
  | 'BUILDING'
  | 'VEHICLE'
  | 'PEDESTRIAN'
  | 'OBSTACLE_OTHER';

/**
 * @brief Dyadic spatial resolution levels:
 * L0 = 0.05 m (5 cm)
 * L1 = 0.10 m (10 cm)
 * L2 = 0.20 m (20 cm)
 * L3 = 0.40 m (40 cm)
 */
export type ResolutionLevel = 0 | 1 | 2 | 3;

export interface PointXYZ {
  x: number;
  y: number;
  z: number;
}

/**
 * @brief Single LiDAR point representation for visualization DTO.
 */
export interface PerceptionPoint {
  sourceIndex: number;
  x: number;
  y: number;
  z: number;
  semanticClass: SemanticClass;
  routing: PointRouting;
}

/**
 * @brief Confirmed dynamic point return (Frozen C++ DynamicPoint).
 */
export interface DynamicPoint {
  sourceIndex: number;
  position: PointXYZ;
  semanticClass: SemanticClass;
  semanticConfidence: number;
  motionConfidence: number;
}

/**
 * @brief Frame-local 2.5D dynamic cluster candidate (Frozen C++ DynamicObjectCluster).
 * Note: Frame-local only. No persistent TrackId, velocity, or trajectories.
 */
export interface DynamicObjectCluster {
  clusterLocalId: number;
  canonicalSourcePointIndex: number;
  pointCount: number;
  memberSourceIndices: number[];
  centroid: PointXYZ;
  aabbMin: PointXYZ;
  aabbMax: PointXYZ;
  dominantClass: SemanticClass;
  meanSemanticConfidence: number;
  meanMotionConfidence: number;
}

/**
 * @brief Frontend visualization DTO for an adaptive 2.5D map cell.
 */
export interface AdaptiveCell {
  cellId: string;
  level: ResolutionLevel;
  x: number;
  y: number;
  sizeMeters: number;
  pointCount: number;
  elevationMean: number;
  elevationMin: number;
  elevationMax: number;
  resolutionReasons: ResolutionReason[];
}

export interface ResolutionSummary {
  totalCells: number;
  l0Count: number;
  l1Count: number;
  l2Count: number;
  l3Count: number;
  l0Percent: number;
  l1Percent: number;
  l2Percent: number;
  l3Percent: number;
}

export type SemanticSummary = Record<SemanticClass, number>;

export interface DynamicSummary {
  dynamicPoints: number;
  clusterCount: number;
  noiseCount: number;
}

export type ScenarioId = 'A' | 'B' | 'C' | 'D';

/**
 * @brief Authoritative perception frame data object.
 * Note: Raw point arrays and cell trees are owned exclusively by the runtime/source
 * and are not placed into global React / Zustand state.
 */
export interface PerceptionFrame {
  frameSeq: number;
  pipelineStatus: AdaptivePipelineStatusName;
  mode: string;
  scenarioId: ScenarioId;
  scenarioTitle: string;
  scenarioDescription: string;
  sensorOrigin: PointXYZ;
  points: PerceptionPoint[];
  cells: AdaptiveCell[];
  dynamicPoints: DynamicPoint[];
  dynamicClusters: DynamicObjectCluster[];
  noiseSourceIndices: number[];
  resolutionSummary: ResolutionSummary;
  semanticSummary: SemanticSummary;
  dynamicSummary: DynamicSummary;
}
