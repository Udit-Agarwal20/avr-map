import { AdaptiveCell, PerceptionFrame, PerceptionPoint } from '../types/perception';
import { calculateDynamicSummary, calculateResolutionSummary, calculateSemanticSummary } from '../utils/summaries';

const points: PerceptionPoint[] = [
  // Road surface points (STATIC_FUSABLE)
  { sourceIndex: 1, x: -2.0, y: 5.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 2, x: 0.0, y: 5.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 3, x: 2.0, y: 5.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 4, x: -2.0, y: 10.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 5, x: 0.0, y: 10.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 6, x: 2.0, y: 10.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 7, x: -2.0, y: 15.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 8, x: 0.0, y: 15.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 9, x: 2.0, y: 15.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 10, x: -2.0, y: 20.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 11, x: 0.0, y: 20.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 12, x: 2.0, y: 20.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  // Ground & Curb points
  { sourceIndex: 13, x: -3.5, y: 8.0, z: 0.0, semanticClass: 'GROUND', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 14, x: -3.5, y: 12.0, z: 0.0, semanticClass: 'GROUND', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 15, x: 3.5, y: 8.0, z: 0.0, semanticClass: 'GROUND', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 16, x: 3.5, y: 12.0, z: 0.0, semanticClass: 'GROUND', routing: 'STATIC_FUSABLE' },
  // Building facade points
  { sourceIndex: 17, x: -6.0, y: 10.0, z: 1.5, semanticClass: 'BUILDING', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 18, x: -6.0, y: 15.0, z: 2.0, semanticClass: 'BUILDING', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 19, x: 6.0, y: 10.0, z: 1.8, semanticClass: 'BUILDING', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 20, x: 6.0, y: 15.0, z: 2.2, semanticClass: 'BUILDING', routing: 'STATIC_FUSABLE' },
];

const cells: AdaptiveCell[] = [
  // L3 (40 cm) cells on distant / open road
  { cellId: 'A_L3_01', level: 3, x: -2.0, y: 16.0, sizeMeters: 0.40, pointCount: 4, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'A_L3_02', level: 3, x: 0.0, y: 16.0, sizeMeters: 0.40, pointCount: 5, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'A_L3_03', level: 3, x: 2.0, y: 16.0, sizeMeters: 0.40, pointCount: 4, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'A_L3_04', level: 3, x: -2.0, y: 20.0, sizeMeters: 0.40, pointCount: 3, elevationMean: -0.1, elevationMin: -0.13, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'A_L3_05', level: 3, x: 0.0, y: 20.0, sizeMeters: 0.40, pointCount: 4, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'A_L3_06', level: 3, x: 2.0, y: 20.0, sizeMeters: 0.40, pointCount: 3, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  // L2 (20 cm) cells on mid-range corridor road
  { cellId: 'A_L2_01', level: 2, x: -2.0, y: 8.0, sizeMeters: 0.20, pointCount: 6, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'A_L2_02', level: 2, x: 0.0, y: 8.0, sizeMeters: 0.20, pointCount: 7, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'A_L2_03', level: 2, x: 2.0, y: 8.0, sizeMeters: 0.20, pointCount: 6, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'A_L2_04', level: 2, x: -2.0, y: 12.0, sizeMeters: 0.20, pointCount: 5, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'A_L2_05', level: 2, x: 0.0, y: 12.0, sizeMeters: 0.20, pointCount: 6, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'A_L2_06', level: 2, x: 2.0, y: 12.0, sizeMeters: 0.20, pointCount: 5, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  // L2 (20 cm) on building facades
  { cellId: 'A_L2_07', level: 2, x: -6.0, y: 10.0, sizeMeters: 0.20, pointCount: 8, elevationMean: 1.5, elevationMin: 0.2, elevationMax: 2.8, resolutionReasons: ['GEOMETRY_COMPLEXITY'] },
  { cellId: 'A_L2_08', level: 2, x: 6.0, y: 10.0, sizeMeters: 0.20, pointCount: 8, elevationMean: 1.8, elevationMin: 0.2, elevationMax: 3.1, resolutionReasons: ['GEOMETRY_COMPLEXITY'] },
  // L1 (10 cm) curb edges
  { cellId: 'A_L1_01', level: 1, x: -3.5, y: 8.0, sizeMeters: 0.10, pointCount: 8, elevationMean: 0.05, elevationMin: -0.1, elevationMax: 0.18, resolutionReasons: ['GEOMETRY_COMPLEXITY'] },
  { cellId: 'A_L1_02', level: 1, x: 3.5, y: 8.0, sizeMeters: 0.10, pointCount: 8, elevationMean: 0.05, elevationMin: -0.1, elevationMax: 0.18, resolutionReasons: ['GEOMETRY_COMPLEXITY'] },
];

export const scenarioA: PerceptionFrame = {
  frameSeq: 101,
  pipelineStatus: 'SUCCESS',
  mode: 'URBAN_CORRIDOR',
  scenarioId: 'A',
  scenarioTitle: 'Scenario A: Urban Corridor Baseline',
  scenarioDescription: 'Straight asphalt road flanked by buildings. Static baseline with coarse L2/L3 allocation and zero dynamic objects.',
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
