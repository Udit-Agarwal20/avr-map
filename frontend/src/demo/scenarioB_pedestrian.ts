import { AdaptiveCell, PerceptionFrame, PerceptionPoint } from '../types/perception';
import { calculateDynamicSummary, calculateResolutionSummary, calculateSemanticSummary } from '../utils/summaries';

const points: PerceptionPoint[] = [
  // Road surface
  { sourceIndex: 1, x: -2.0, y: 5.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 2, x: 0.0, y: 5.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 3, x: 2.0, y: 5.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 4, x: -2.0, y: 12.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 5, x: 0.0, y: 12.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 6, x: 2.0, y: 12.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  // Pedestrian Hazard points (STATIC_FUSABLE obstacle on curb)
  { sourceIndex: 7, x: 2.45, y: 8.0, z: 0.2, semanticClass: 'PEDESTRIAN', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 8, x: 2.50, y: 8.05, z: 0.6, semanticClass: 'PEDESTRIAN', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 9, x: 2.52, y: 7.95, z: 1.1, semanticClass: 'PEDESTRIAN', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 10, x: 2.48, y: 8.02, z: 1.5, semanticClass: 'PEDESTRIAN', routing: 'STATIC_FUSABLE' },
  // Vegetation
  { sourceIndex: 11, x: -4.0, y: 8.0, z: 0.8, semanticClass: 'VEGETATION', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 12, x: -4.2, y: 12.0, z: 1.2, semanticClass: 'VEGETATION', routing: 'STATIC_FUSABLE' },
];

const cells: AdaptiveCell[] = [
  // L0 (5 cm) localized fine patches centered on pedestrian hazard
  { cellId: 'B_L0_01', level: 0, x: 2.45, y: 7.95, sizeMeters: 0.05, pointCount: 6, elevationMean: 0.4, elevationMin: 0.1, elevationMax: 1.5, resolutionReasons: ['SEMANTIC_RELEVANCE', 'UNCERTAINTY'] },
  { cellId: 'B_L0_02', level: 0, x: 2.50, y: 7.95, sizeMeters: 0.05, pointCount: 5, elevationMean: 0.5, elevationMin: 0.1, elevationMax: 1.6, resolutionReasons: ['SEMANTIC_RELEVANCE', 'UNCERTAINTY'] },
  { cellId: 'B_L0_03', level: 0, x: 2.45, y: 8.00, sizeMeters: 0.05, pointCount: 7, elevationMean: 0.6, elevationMin: 0.2, elevationMax: 1.7, resolutionReasons: ['SEMANTIC_RELEVANCE', 'UNCERTAINTY'] },
  { cellId: 'B_L0_04', level: 0, x: 2.50, y: 8.00, sizeMeters: 0.05, pointCount: 8, elevationMean: 0.7, elevationMin: 0.2, elevationMax: 1.7, resolutionReasons: ['SEMANTIC_RELEVANCE', 'UNCERTAINTY'] },
  // L1 (10 cm) buffer surrounding hazard
  { cellId: 'B_L1_01', level: 1, x: 2.30, y: 7.80, sizeMeters: 0.10, pointCount: 3, elevationMean: 0.05, elevationMin: -0.1, elevationMax: 0.2, resolutionReasons: ['GEOMETRY_COMPLEXITY'] },
  { cellId: 'B_L1_02', level: 1, x: 2.60, y: 8.10, sizeMeters: 0.10, pointCount: 4, elevationMean: 0.05, elevationMin: -0.1, elevationMax: 0.2, resolutionReasons: ['GEOMETRY_COMPLEXITY'] },
  // L2 (20 cm) road area
  { cellId: 'B_L2_01', level: 2, x: -1.0, y: 6.0, sizeMeters: 0.20, pointCount: 5, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'B_L2_02', level: 2, x: 1.0, y: 6.0, sizeMeters: 0.20, pointCount: 6, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'B_L2_03', level: 2, x: -1.0, y: 10.0, sizeMeters: 0.20, pointCount: 5, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'B_L2_04', level: 2, x: 1.0, y: 10.0, sizeMeters: 0.20, pointCount: 6, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  // L3 (40 cm) distant road
  { cellId: 'B_L3_01', level: 3, x: -2.0, y: 18.0, sizeMeters: 0.40, pointCount: 4, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'B_L3_02', level: 3, x: 2.0, y: 18.0, sizeMeters: 0.40, pointCount: 4, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
];

export const scenarioB: PerceptionFrame = {
  frameSeq: 102,
  pipelineStatus: 'SUCCESS',
  mode: 'CROSSWALK_HAZARD',
  scenarioId: 'B',
  scenarioTitle: 'Scenario B: Crosswalk Pedestrian Hazard',
  scenarioDescription: 'Vulnerable road user on crosswalk. Triggers safety-driven 5 cm (L0) refinement patch while preserving coarse L2/L3 allocation on open road.',
  sensorOrigin: { x: 0.0, y: 0.0, z: 0.0 },
  points,
  cells,
  dynamicPoints: [],
  dynamicClusters: [],
  noiseSourceIndices: [],
  resolutionSummary: calculateResolutionSummary(cells),
  semanticSummary: calculateSemanticSummary(points),
  dynamicSummary: calculateDynamicSummary([], [], []),
};
