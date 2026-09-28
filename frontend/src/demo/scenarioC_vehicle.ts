import { withCurrentReturns } from '../utils/topology';
import { AdaptiveCell, DynamicObjectCluster, DynamicPoint, PerceptionFrame, PerceptionPoint } from '../types/perception';
import { calculateDynamicSummary, calculateResolutionSummary, calculateSemanticSummary } from '../utils/summaries';

const points: PerceptionPoint[] = [
  // Static Road
  { sourceIndex: 1, x: -2.0, y: 8.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 2, x: 0.0, y: 8.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 3, x: 2.0, y: 8.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 4, x: -2.0, y: 14.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 5, x: 0.0, y: 14.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 6, x: 2.0, y: 14.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  // Confirmed Dynamic Moving Vehicle Points (DYNAMIC_EXCLUDED)
  { sourceIndex: 50, x: -3.2, y: 13.5, z: 0.3, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 51, x: -2.8, y: 13.5, z: 0.4, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 52, x: -3.2, y: 14.5, z: 0.5, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 53, x: -2.8, y: 14.5, z: 0.6, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 54, x: -3.2, y: 15.5, z: 0.7, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 55, x: -2.8, y: 15.5, z: 0.8, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 56, x: -3.0, y: 14.0, z: 1.2, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 57, x: -3.0, y: 15.0, z: 1.3, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
];

const dynamicPoints: DynamicPoint[] = [
  { sourceIndex: 50, position: { x: -3.2, y: 13.5, z: 0.3 }, semanticClass: 'VEHICLE', semanticConfidence: 0.91, motionConfidence: 0.94 },
  { sourceIndex: 51, position: { x: -2.8, y: 13.5, z: 0.4 }, semanticClass: 'VEHICLE', semanticConfidence: 0.92, motionConfidence: 0.95 },
  { sourceIndex: 52, position: { x: -3.2, y: 14.5, z: 0.5 }, semanticClass: 'VEHICLE', semanticConfidence: 0.93, motionConfidence: 0.96 },
  { sourceIndex: 53, position: { x: -2.8, y: 14.5, z: 0.6 }, semanticClass: 'VEHICLE', semanticConfidence: 0.90, motionConfidence: 0.93 },
  { sourceIndex: 54, position: { x: -3.2, y: 15.5, z: 0.7 }, semanticClass: 'VEHICLE', semanticConfidence: 0.94, motionConfidence: 0.97 },
  { sourceIndex: 55, position: { x: -2.8, y: 15.5, z: 0.8 }, semanticClass: 'VEHICLE', semanticConfidence: 0.92, motionConfidence: 0.95 },
  { sourceIndex: 56, position: { x: -3.0, y: 14.0, z: 1.2 }, semanticClass: 'VEHICLE', semanticConfidence: 0.91, motionConfidence: 0.94 },
  { sourceIndex: 57, position: { x: -3.0, y: 15.0, z: 1.3 }, semanticClass: 'VEHICLE', semanticConfidence: 0.93, motionConfidence: 0.96 },
];

const dynamicClusters: DynamicObjectCluster[] = [
  {
    clusterLocalId: 0,
    canonicalSourcePointIndex: 50,
    pointCount: 8,
    memberSourceIndices: [50, 51, 52, 53, 54, 55, 56, 57],
    centroid: { x: -3.0, y: 14.5, z: 0.725 },
    aabbMin: { x: -3.2, y: 13.5, z: 0.3 },
    aabbMax: { x: -2.8, y: 15.5, z: 1.3 },
    dominantClass: 'VEHICLE',
    meanSemanticConfidence: 0.92,
    meanMotionConfidence: 0.95,
  },
];

const cells: AdaptiveCell[] = [
  // L2 (20 cm) road cells underneath
  { cellId: 'C_L2_01', level: 2, x: -3.0, y: 14.0, sizeMeters: 0.20, pointCount: 6, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DYNAMIC_RELEVANCE', 'DISTANCE_BASELINE'] },
  { cellId: 'C_L2_02', level: 2, x: -1.0, y: 14.0, sizeMeters: 0.20, pointCount: 5, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'C_L2_03', level: 2, x: 1.0, y: 14.0, sizeMeters: 0.20, pointCount: 5, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  // L3 (40 cm) cells
  { cellId: 'C_L3_01', level: 3, x: -2.0, y: 8.0, sizeMeters: 0.40, pointCount: 4, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'C_L3_02', level: 3, x: 2.0, y: 8.0, sizeMeters: 0.40, pointCount: 4, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'C_L3_03', level: 3, x: -2.0, y: 20.0, sizeMeters: 0.40, pointCount: 3, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'C_L3_04', level: 3, x: 2.0, y: 20.0, sizeMeters: 0.40, pointCount: 3, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
];

const activeCells = withCurrentReturns(cells, points);

export const scenarioC: PerceptionFrame = {
  frameSeq: 103,
  pipelineStatus: 'SUCCESS',
  mode: 'DYNAMIC_OVERTAKE',
  scenarioId: 'C',
  scenarioTitle: 'Scenario C: Dynamic Vehicle Overtake',
  scenarioDescription: 'Confirmed dynamic vehicle moving in left lane. Dynamic points are excluded from static map and clustered into [C-00] VEHICLE.',
  sensorOrigin: { x: 0.0, y: 0.0, z: 0.0 },
  points,
  cells: activeCells,
  dynamicPoints,
  dynamicClusters,
  noiseSourceIndices: [],
  resolutionSummary: calculateResolutionSummary(activeCells),
  semanticSummary: calculateSemanticSummary(points),
  dynamicSummary: calculateDynamicSummary(dynamicPoints, dynamicClusters, []),
};
