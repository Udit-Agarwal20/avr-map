import { AdaptiveCell, DynamicObjectCluster, DynamicPoint, PerceptionFrame, PerceptionPoint } from '../types/perception';
import { calculateDynamicSummary, calculateResolutionSummary, calculateSemanticSummary } from '../utils/summaries';

const points: PerceptionPoint[] = [
  // Road & Static Ground
  { sourceIndex: 1, x: -2.0, y: 6.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 2, x: 0.0, y: 6.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 3, x: 2.0, y: 6.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 4, x: -2.0, y: 12.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 5, x: 0.0, y: 12.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 6, x: 2.0, y: 12.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 7, x: -2.0, y: 18.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 8, x: 0.0, y: 18.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 9, x: 2.0, y: 18.0, z: -0.1, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' },
  // Buildings & Vegetation
  { sourceIndex: 20, x: -6.0, y: 12.0, z: 2.0, semanticClass: 'BUILDING', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 21, x: -6.0, y: 16.0, z: 2.2, semanticClass: 'BUILDING', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 22, x: 5.5, y: 15.0, z: 1.5, semanticClass: 'VEGETATION', routing: 'STATIC_FUSABLE' },
  { sourceIndex: 23, x: 5.5, y: 18.0, z: 1.8, semanticClass: 'VEGETATION', routing: 'STATIC_FUSABLE' },
  // Confirmed Dynamic Moving Vehicle Points (Cluster 0)
  { sourceIndex: 101, x: -3.2, y: 14.5, z: 0.3, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 102, x: -2.8, y: 14.5, z: 0.4, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 103, x: -3.2, y: 15.5, z: 0.5, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 104, x: -2.8, y: 15.5, z: 0.6, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 105, x: -3.2, y: 16.5, z: 0.7, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 106, x: -2.8, y: 16.5, z: 0.8, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 107, x: -3.0, y: 15.0, z: 1.2, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 108, x: -3.0, y: 16.0, z: 1.3, semanticClass: 'VEHICLE', routing: 'DYNAMIC_EXCLUDED' },
  // Confirmed Dynamic Crossing Pedestrians Points (Cluster 1)
  { sourceIndex: 201, x: 3.4, y: 9.8, z: 0.2, semanticClass: 'PEDESTRIAN', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 202, x: 3.6, y: 9.9, z: 0.6, semanticClass: 'PEDESTRIAN', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 203, x: 3.5, y: 10.1, z: 1.1, semanticClass: 'PEDESTRIAN', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 204, x: 3.5, y: 10.2, z: 1.5, semanticClass: 'PEDESTRIAN', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 205, x: 3.8, y: 10.3, z: 0.2, semanticClass: 'PEDESTRIAN', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 206, x: 3.9, y: 10.4, z: 0.6, semanticClass: 'PEDESTRIAN', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 207, x: 3.8, y: 10.5, z: 1.1, semanticClass: 'PEDESTRIAN', routing: 'DYNAMIC_EXCLUDED' },
  { sourceIndex: 208, x: 3.9, y: 10.6, z: 1.5, semanticClass: 'PEDESTRIAN', routing: 'DYNAMIC_EXCLUDED' },
];

const dynamicPoints: DynamicPoint[] = [
  // Vehicle points
  { sourceIndex: 101, position: { x: -3.2, y: 14.5, z: 0.3 }, semanticClass: 'VEHICLE', semanticConfidence: 0.92, motionConfidence: 0.95 },
  { sourceIndex: 102, position: { x: -2.8, y: 14.5, z: 0.4 }, semanticClass: 'VEHICLE', semanticConfidence: 0.93, motionConfidence: 0.96 },
  { sourceIndex: 103, position: { x: -3.2, y: 15.5, z: 0.5 }, semanticClass: 'VEHICLE', semanticConfidence: 0.94, motionConfidence: 0.97 },
  { sourceIndex: 104, position: { x: -2.8, y: 15.5, z: 0.6 }, semanticClass: 'VEHICLE', semanticConfidence: 0.91, motionConfidence: 0.94 },
  { sourceIndex: 105, position: { x: -3.2, y: 16.5, z: 0.7 }, semanticClass: 'VEHICLE', semanticConfidence: 0.95, motionConfidence: 0.98 },
  { sourceIndex: 106, position: { x: -2.8, y: 16.5, z: 0.8 }, semanticClass: 'VEHICLE', semanticConfidence: 0.93, motionConfidence: 0.95 },
  { sourceIndex: 107, position: { x: -3.0, y: 15.0, z: 1.2 }, semanticClass: 'VEHICLE', semanticConfidence: 0.92, motionConfidence: 0.94 },
  { sourceIndex: 108, position: { x: -3.0, y: 16.0, z: 1.3 }, semanticClass: 'VEHICLE', semanticConfidence: 0.94, motionConfidence: 0.96 },
  // Pedestrian points
  { sourceIndex: 201, position: { x: 3.4, y: 9.8, z: 0.2 }, semanticClass: 'PEDESTRIAN', semanticConfidence: 0.88, motionConfidence: 0.90 },
  { sourceIndex: 202, position: { x: 3.6, y: 9.9, z: 0.6 }, semanticClass: 'PEDESTRIAN', semanticConfidence: 0.89, motionConfidence: 0.91 },
  { sourceIndex: 203, position: { x: 3.5, y: 10.1, z: 1.1 }, semanticClass: 'PEDESTRIAN', semanticConfidence: 0.90, motionConfidence: 0.92 },
  { sourceIndex: 204, position: { x: 3.5, y: 10.2, z: 1.5 }, semanticClass: 'PEDESTRIAN', semanticConfidence: 0.87, motionConfidence: 0.89 },
  { sourceIndex: 205, position: { x: 3.8, y: 10.3, z: 0.2 }, semanticClass: 'PEDESTRIAN', semanticConfidence: 0.89, motionConfidence: 0.91 },
  { sourceIndex: 206, position: { x: 3.9, y: 10.4, z: 0.6 }, semanticClass: 'PEDESTRIAN', semanticConfidence: 0.91, motionConfidence: 0.93 },
  { sourceIndex: 207, position: { x: 3.8, y: 10.5, z: 1.1 }, semanticClass: 'PEDESTRIAN', semanticConfidence: 0.88, motionConfidence: 0.90 },
  { sourceIndex: 208, position: { x: 3.9, y: 10.6, z: 1.5 }, semanticClass: 'PEDESTRIAN', semanticConfidence: 0.90, motionConfidence: 0.92 },
];

const dynamicClusters: DynamicObjectCluster[] = [
  {
    clusterLocalId: 0,
    canonicalSourcePointIndex: 101,
    pointCount: 8,
    memberSourceIndices: [101, 102, 103, 104, 105, 106, 107, 108],
    centroid: { x: -3.0, y: 15.5, z: 0.725 },
    aabbMin: { x: -3.3, y: 14.3, z: 0.2 },
    aabbMax: { x: -2.7, y: 16.7, z: 1.4 },
    dominantClass: 'VEHICLE',
    meanSemanticConfidence: 0.93,
    meanMotionConfidence: 0.95,
  },
  {
    clusterLocalId: 1,
    canonicalSourcePointIndex: 201,
    pointCount: 8,
    memberSourceIndices: [201, 202, 203, 204, 205, 206, 207, 208],
    centroid: { x: 3.68, y: 10.23, z: 0.85 },
    aabbMin: { x: 3.3, y: 9.7, z: 0.1 },
    aabbMax: { x: 4.1, y: 10.8, z: 1.6 },
    dominantClass: 'PEDESTRIAN',
    meanSemanticConfidence: 0.89,
    meanMotionConfidence: 0.91,
  },
];

const cells: AdaptiveCell[] = [
  // L0 (5 cm) cells near crossing pedestrians
  { cellId: 'D_L0_01', level: 0, x: 3.40, y: 9.80, sizeMeters: 0.05, pointCount: 6, elevationMean: 0.4, elevationMin: 0.1, elevationMax: 1.6, resolutionReasons: ['SEMANTIC_RELEVANCE', 'UNCERTAINTY'] },
  { cellId: 'D_L0_02', level: 0, x: 3.45, y: 9.80, sizeMeters: 0.05, pointCount: 5, elevationMean: 0.5, elevationMin: 0.1, elevationMax: 1.5, resolutionReasons: ['SEMANTIC_RELEVANCE', 'UNCERTAINTY'] },
  { cellId: 'D_L0_03', level: 0, x: 3.40, y: 9.85, sizeMeters: 0.05, pointCount: 7, elevationMean: 0.6, elevationMin: 0.2, elevationMax: 1.7, resolutionReasons: ['SEMANTIC_RELEVANCE', 'UNCERTAINTY'] },
  { cellId: 'D_L0_04', level: 0, x: 3.45, y: 9.85, sizeMeters: 0.05, pointCount: 8, elevationMean: 0.7, elevationMin: 0.2, elevationMax: 1.7, resolutionReasons: ['SEMANTIC_RELEVANCE', 'UNCERTAINTY'] },
  // L1 (10 cm) curb edges & road interface
  { cellId: 'D_L1_01', level: 1, x: 3.20, y: 9.60, sizeMeters: 0.10, pointCount: 5, elevationMean: 0.05, elevationMin: -0.1, elevationMax: 0.2, resolutionReasons: ['GEOMETRY_COMPLEXITY'] },
  { cellId: 'D_L1_02', level: 1, x: 3.60, y: 10.0, sizeMeters: 0.10, pointCount: 5, elevationMean: 0.05, elevationMin: -0.1, elevationMax: 0.2, resolutionReasons: ['GEOMETRY_COMPLEXITY'] },
  { cellId: 'D_L1_03', level: 1, x: -3.5, y: 12.0, sizeMeters: 0.10, pointCount: 4, elevationMean: 0.05, elevationMin: -0.1, elevationMax: 0.2, resolutionReasons: ['GEOMETRY_COMPLEXITY'] },
  // L2 (20 cm) vehicle lane & mid-road
  { cellId: 'D_L2_01', level: 2, x: -3.0, y: 15.0, sizeMeters: 0.20, pointCount: 6, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DYNAMIC_RELEVANCE', 'DISTANCE_BASELINE'] },
  { cellId: 'D_L2_02', level: 2, x: -1.0, y: 10.0, sizeMeters: 0.20, pointCount: 7, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'D_L2_03', level: 2, x: 1.0, y: 10.0, sizeMeters: 0.20, pointCount: 6, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'D_L2_04', level: 2, x: -1.0, y: 14.0, sizeMeters: 0.20, pointCount: 5, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'D_L2_05', level: 2, x: 1.0, y: 14.0, sizeMeters: 0.20, pointCount: 6, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  // L3 (40 cm) distant road & open terrain
  { cellId: 'D_L3_01', level: 3, x: -2.0, y: 20.0, sizeMeters: 0.40, pointCount: 4, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'D_L3_02', level: 3, x: 2.0, y: 20.0, sizeMeters: 0.40, pointCount: 4, elevationMean: -0.1, elevationMin: -0.11, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'D_L3_03', level: 3, x: -2.0, y: 24.0, sizeMeters: 0.40, pointCount: 3, elevationMean: -0.1, elevationMin: -0.13, elevationMax: -0.08, resolutionReasons: ['DISTANCE_BASELINE'] },
  { cellId: 'D_L3_04', level: 3, x: 2.0, y: 24.0, sizeMeters: 0.40, pointCount: 3, elevationMean: -0.1, elevationMin: -0.12, elevationMax: -0.09, resolutionReasons: ['DISTANCE_BASELINE'] },
];

export const scenarioD: PerceptionFrame = {
  frameSeq: 104,
  pipelineStatus: 'SUCCESS',
  mode: 'MIXED_INTERSECTION',
  scenarioId: 'D',
  scenarioTitle: 'Scenario D: Complex Mixed Intersection (Hero)',
  scenarioDescription: 'Complex multi-hazard scene. Simultaneous moving vehicle and pedestrian hazards, showing wide hierarchical resolution allocation and frame-local cluster extraction.',
  sensorOrigin: { x: 0.0, y: 0.0, z: 0.0 },
  points,
  cells,
  dynamicPoints,
  dynamicClusters,
  noiseSourceIndices: [],
  resolutionSummary: calculateResolutionSummary(cells),
  semanticSummary: calculateSemanticSummary(points),
  dynamicSummary: calculateDynamicSummary(dynamicPoints, dynamicClusters, []),
};
