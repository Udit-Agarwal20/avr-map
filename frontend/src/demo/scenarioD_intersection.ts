import { buildActiveLeaves } from './activeLeaves';
import { withCurrentReturns } from '../utils/topology';
import type { AdaptiveCell, DynamicObjectCluster, DynamicPoint, PerceptionFrame, PerceptionPoint } from '../types/perception';
import { calculateDynamicSummary, calculateResolutionSummary, calculateSemanticSummary } from '../utils/summaries';

// ============================================================================
// 1. DYNAMIC POINTS & CLUSTERS (Confirmed Hazards)
// ============================================================================

// Cluster 0: Moving Oncoming Vehicle (x: ~ -3.0, y: ~ 15.5, z: ~ 0.7)
const vehicleRawPoints: { dx: number; dy: number; dz: number }[] = [
  // Wheels (4)
  { dx: -0.75, dy: -1.2, dz: 0.25 },
  { dx: 0.75, dy: -1.2, dz: 0.25 },
  { dx: -0.75, dy: 1.2, dz: 0.25 },
  { dx: 0.75, dy: 1.2, dz: 0.25 },
  // Lower chassis (8)
  { dx: -0.85, dy: -1.8, dz: 0.45 },
  { dx: 0.85, dy: -1.8, dz: 0.45 },
  { dx: -0.85, dy: 0.0, dz: 0.45 },
  { dx: 0.85, dy: 0.0, dz: 0.45 },
  { dx: -0.85, dy: 1.8, dz: 0.45 },
  { dx: 0.85, dy: 1.8, dz: 0.45 },
  { dx: 0.0, dy: -1.85, dz: 0.45 },
  { dx: 0.0, dy: 1.85, dz: 0.45 },
  // Hood / Beltline (8)
  { dx: -0.8, dy: -1.4, dz: 0.85 },
  { dx: 0.8, dy: -1.4, dz: 0.85 },
  { dx: -0.8, dy: 0.0, dz: 0.85 },
  { dx: 0.8, dy: 0.0, dz: 0.85 },
  { dx: -0.8, dy: 1.4, dz: 0.85 },
  { dx: 0.8, dy: 1.4, dz: 0.85 },
  { dx: 0.0, dy: -1.5, dz: 0.85 },
  { dx: 0.0, dy: 1.5, dz: 0.85 },
  // Cabin Roof (4)
  { dx: -0.6, dy: -0.5, dz: 1.35 },
  { dx: 0.6, dy: -0.5, dz: 1.35 },
  { dx: -0.6, dy: 0.5, dz: 1.35 },
  { dx: 0.6, dy: 0.5, dz: 1.35 },
];

const vehicleBase = { x: -3.0, y: 15.5, z: 0.0 };
const vehicleDynamicPoints: DynamicPoint[] = vehicleRawPoints.map((p, idx) => ({
  sourceIndex: 101 + idx,
  position: {
    x: Number((vehicleBase.x + p.dx).toFixed(3)),
    y: Number((vehicleBase.y + p.dy).toFixed(3)),
    z: Number((vehicleBase.z + p.dz).toFixed(3)),
  },
  semanticClass: 'VEHICLE',
  semanticConfidence: Number((0.92 + (idx % 5) * 0.01).toFixed(2)),
  motionConfidence: Number((0.94 + (idx % 4) * 0.01).toFixed(2)),
}));

// Cluster 1: Crossing Pedestrian (x: ~ 3.60, y: ~ 10.20, z: ~ 0.88)
const pedRawPoints: { dx: number; dy: number; dz: number }[] = [
  // 6 core returns inside D_L0_P_127 footprint [3.575, 3.625] x [10.175, 10.225]
  { dx: -0.015, dy: -0.010, dz: 0.90 }, // Hip L
  { dx: 0.015, dy: 0.010, dz: 0.90 },  // Hip R
  { dx: -0.015, dy: -0.012, dz: 1.20 }, // Torso L
  { dx: 0.015, dy: 0.012, dz: 1.20 },  // Torso R
  { dx: -0.010, dy: 0.010, dz: 1.45 },  // Chest L
  { dx: 0.010, dy: -0.010, dz: 1.45 },  // Chest R
  // 10 peripheral returns in adjacent cells of halo
  { dx: -0.10, dy: -0.07, dz: 0.15 },  // Foot L
  { dx: 0.10, dy: 0.07, dz: 0.15 },   // Foot R
  { dx: -0.08, dy: -0.05, dz: 0.50 },  // Knee L
  { dx: 0.08, dy: 0.05, dz: 0.50 },   // Knee R
  { dx: -0.12, dy: -0.02, dz: 1.35 },  // Arm L
  { dx: 0.12, dy: 0.02, dz: 1.35 },   // Arm R
  { dx: -0.08, dy: -0.06, dz: 1.68 },  // Head L
  { dx: 0.08, dy: -0.06, dz: 1.68 },   // Head R
  { dx: -0.08, dy: 0.06, dz: 1.72 },   // Head top L
  { dx: 0.08, dy: 0.06, dz: 1.72 },    // Head top R
];

const pedBase = { x: 3.60, y: 10.20, z: 0.0 };
const pedDynamicPoints: DynamicPoint[] = pedRawPoints.map((p, idx) => ({
  sourceIndex: 201 + idx,
  position: {
    x: Number((pedBase.x + p.dx).toFixed(3)),
    y: Number((pedBase.y + p.dy).toFixed(3)),
    z: Number((pedBase.z + p.dz).toFixed(3)),
  },
  semanticClass: 'PEDESTRIAN',
  semanticConfidence: Number((0.89 + (idx % 4) * 0.01).toFixed(2)),
  motionConfidence: Number((0.91 + (idx % 3) * 0.01).toFixed(2)),
}));

const dynamicPoints: DynamicPoint[] = [...vehicleDynamicPoints, ...pedDynamicPoints];

// Build cluster bounding boxes & centroids deterministically from members
function computeCluster(
  id: number,
  pts: DynamicPoint[],
  dominantClass: 'VEHICLE' | 'PEDESTRIAN'
): DynamicObjectCluster {
  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  let minZ = Infinity;
  let maxZ = -Infinity;
  let sumX = 0;
  let sumY = 0;
  let sumZ = 0;
  let sumSem = 0;
  let sumMot = 0;

  for (const pt of pts) {
    const { x, y, z } = pt.position;
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
    sumX += x;
    sumY += y;
    sumZ += z;
    sumSem += pt.semanticConfidence;
    sumMot += pt.motionConfidence;
  }

  const n = pts.length;
  return {
    clusterLocalId: id,
    canonicalSourcePointIndex: pts[0].sourceIndex,
    pointCount: n,
    memberSourceIndices: pts.map((p) => p.sourceIndex),
    centroid: {
      x: sumX / n,
      y: sumY / n,
      z: sumZ / n,
    },
    aabbMin: {
      x: minX,
      y: minY,
      z: minZ,
    },
    aabbMax: {
      x: maxX,
      y: maxY,
      z: maxZ,
    },
    dominantClass,
    meanSemanticConfidence: sumSem / n,
    meanMotionConfidence: sumMot / n,
  };
}

const dynamicClusters: DynamicObjectCluster[] = [
  computeCluster(0, vehicleDynamicPoints, 'VEHICLE'),
  computeCluster(1, pedDynamicPoints, 'PEDESTRIAN'),
];

// ============================================================================
// 2. STATIC POINTS (Road, Curbs, Buildings, Trees, Lane Markings)
// ============================================================================

const staticPoints: PerceptionPoint[] = [];
let staticIdx = 1001;

// A. Road surface points
for (let y = -4; y <= 32; y += 3) {
  for (let x = -4; x <= 4; x += 2) {
    staticPoints.push({
      sourceIndex: staticIdx++,
      x: Number(x.toFixed(2)),
      y: Number(y.toFixed(2)),
      z: -0.05,
      semanticClass: 'ROAD',
      routing: 'STATIC_FUSABLE',
    });
  }
}

// B. Curbs (x = -4.8, x = +4.8, z = 0.15)
for (let y = -4; y <= 32; y += 2) {
  staticPoints.push({
    sourceIndex: staticIdx++,
    x: -4.8,
    y: Number(y.toFixed(2)),
    z: 0.15,
    semanticClass: 'GROUND',
    routing: 'STATIC_FUSABLE',
  });
  staticPoints.push({
    sourceIndex: staticIdx++,
    x: 4.8,
    y: Number(y.toFixed(2)),
    z: 0.15,
    semanticClass: 'GROUND',
    routing: 'STATIC_FUSABLE',
  });
}

// C. Buildings (left and right urban blocks)
const buildingLocations = [
  { x: -7.5, y: 6.0, z: 3.5 },
  { x: -7.5, y: 10.0, z: 4.2 },
  { x: -7.5, y: 20.0, z: 5.0 },
  { x: -7.5, y: 26.0, z: 4.0 },
  { x: 7.5, y: 5.0, z: 3.8 },
  { x: 7.5, y: 22.0, z: 4.5 },
  { x: 7.5, y: 28.0, z: 5.2 },
];
for (const b of buildingLocations) {
  staticPoints.push({
    sourceIndex: staticIdx++,
    x: b.x,
    y: b.y,
    z: b.z,
    semanticClass: 'BUILDING',
    routing: 'STATIC_FUSABLE',
  });
  staticPoints.push({
    sourceIndex: staticIdx++,
    x: b.x,
    y: b.y,
    z: b.z * 0.5,
    semanticClass: 'BUILDING',
    routing: 'STATIC_FUSABLE',
  });
}

// D. Trees / Vegetation
const treeLocations = [
  { x: -6.0, y: 4.0 },
  { x: -6.0, y: 22.0 },
  { x: 6.0, y: 4.0 },
  { x: 6.0, y: 18.0 },
];
for (const t of treeLocations) {
  for (let z = 1.0; z <= 2.5; z += 0.5) {
    staticPoints.push({
      sourceIndex: staticIdx++,
      x: t.x,
      y: t.y,
      z,
      semanticClass: 'VEGETATION',
      routing: 'STATIC_FUSABLE',
    });
  }
}

// Deterministic surface samples for the synthetic LiDAR demonstration.
// Keep actor footprints occluded: the golden pedestrian region retains its six dynamic returns.
let surfaceIndex = 20000;
for (let iy = -16; iy <= 120; iy++) {
  for (let ix = -18; ix <= 18; ix++) {
    const x = ix * .25, y = iy * .25;
    if (dynamicClusters.some(c => x >= c.aabbMin.x - .1 && x <= c.aabbMax.x + .1 && y >= c.aabbMin.y - .1 && y <= c.aabbMax.y + .1)) continue;
    staticPoints.push({ sourceIndex: surfaceIndex++, x, y, z: .014, semanticClass: 'ROAD', routing: 'STATIC_FUSABLE' });
  }
}
for (const x of [-4.8, 4.8]) {
  for (let iy = -16; iy <= 120; iy++) {
    staticPoints.push({sourceIndex:surfaceIndex++,x,y:iy*.25,z:.16,semanticClass:'GROUND',routing:'STATIC_FUSABLE'});
  }
}

// Combine static and dynamic points into unified points array
const points: PerceptionPoint[] = [
  ...staticPoints,
  ...dynamicPoints.map((dp) => ({
    sourceIndex: dp.sourceIndex,
    x: dp.position.x,
    y: dp.position.y,
    z: dp.position.z,
    semanticClass: dp.semanticClass,
    routing: 'DYNAMIC_EXCLUDED' as const,
  })),
];

// ============================================================================
// 3. ADAPTIVE RESOLUTION CELLS (Deterministic Dyadic Hierarchy)
// ============================================================================

const cells: AdaptiveCell[] = [];
let cellSeq = 1;

// L0: 5 cm (0.05 m)
// Hazard Halo 1: Pedestrian at Crosswalk (x: 3.3 to 4.0, y: 9.8 to 10.7)
for (let y = 9.80; y <= 10.70; y += 0.05) {
  for (let x = 3.30; x <= 4.00; x += 0.05) {
    const isCoreHazardCell = Math.abs(x - 3.60) < 0.02 && Math.abs(y - 10.20) < 0.02;
    cells.push({
      cellId: `D_L0_P_${cellSeq++}`,
      level: 0,
      x: Number(x.toFixed(3)),
      y: Number(y.toFixed(3)),
      sizeMeters: 0.05,
      pointCount: 6,
      resolutionReasons: isCoreHazardCell
        ? ['SEMANTIC_RELEVANCE', 'DYNAMIC_RELEVANCE']
        : ['SEMANTIC_RELEVANCE'],
    });
  }
}

// Hazard Halo 2: Vehicle Footprint Core (x: -3.5 to -2.5, y: 14.6 to 16.4)
for (let y = 14.60; y <= 16.40; y += 0.05) {
  for (let x = -3.50; x <= -2.50; x += 0.05) {
    cells.push({
      cellId: `D_L0_V_${cellSeq++}`,
      level: 0,
      x: Number(x.toFixed(3)),
      y: Number(y.toFixed(3)),
      sizeMeters: 0.05,
      pointCount: 5,
      resolutionReasons: ['DYNAMIC_RELEVANCE'],
    });
  }
}

// L1: 10 cm (0.10 m)
// Pedestrian Transition Zone & Crosswalk Interface
for (let y = 9.30; y <= 11.20; y += 0.10) {
  for (let x = 2.90; x <= 4.40; x += 0.10) {
    // Avoid overlap with L0 region
    if (x >= 3.25 && x <= 4.05 && y >= 9.75 && y <= 10.75) continue;
    cells.push({
      cellId: `D_L1_PT_${cellSeq++}`,
      level: 1,
      x: Number(x.toFixed(3)),
      y: Number(y.toFixed(3)),
      sizeMeters: 0.10,
      pointCount: 4,
      elevationMean: 0.04,
      elevationMin: 0.0,
      elevationMax: 0.3,
      resolutionReasons: ['SEMANTIC_RELEVANCE', 'DISTANCE_BASELINE'],
    });
  }
}

// Vehicle Transition Zone
for (let y = 13.80; y <= 17.20; y += 0.10) {
  for (let x = -3.90; x <= -2.10; x += 0.10) {
    // Avoid overlap with L0 region
    if (x >= -3.55 && x <= -2.45 && y >= 14.55 && y <= 16.45) continue;
    cells.push({
      cellId: `D_L1_VT_${cellSeq++}`,
      level: 1,
      x: Number(x.toFixed(3)),
      y: Number(y.toFixed(3)),
      sizeMeters: 0.10,
      pointCount: 4,
      elevationMean: 0.02,
      elevationMin: -0.05,
      elevationMax: 0.2,
      resolutionReasons: ['DYNAMIC_RELEVANCE', 'DISTANCE_BASELINE'],
    });
  }
}

// Curb edges (right and left curb interfaces)
for (let y = 6.0; y <= 18.0; y += 0.20) {
  cells.push({
    cellId: `D_L1_CR_${cellSeq++}`,
    level: 1,
    x: 4.65,
    y: Number(y.toFixed(3)),
    sizeMeters: 0.10,
    pointCount: 3,
    elevationMean: 0.12,
    elevationMin: 0.0,
    elevationMax: 0.15,
    resolutionReasons: ['GEOMETRY_COMPLEXITY'],
  });
  cells.push({
    cellId: `D_L1_CL_${cellSeq++}`,
    level: 1,
    x: -4.65,
    y: Number(y.toFixed(3)),
    sizeMeters: 0.10,
    pointCount: 3,
    elevationMean: 0.12,
    elevationMin: 0.0,
    elevationMax: 0.15,
    resolutionReasons: ['GEOMETRY_COMPLEXITY'],
  });
}

// L2: 20 cm (0.20 m)
// Driving Lanes & Approach Corridor
for (let y = 4.0; y <= 21.0; y += 0.40) {
  for (let x = -4.0; x <= 4.0; x += 0.40) {
    // Avoid dense vehicle and pedestrian zones
    if (x >= -4.0 && x <= -2.0 && y >= 13.6 && y <= 17.4) continue;
    if (x >= 2.8 && x <= 4.5 && y >= 9.2 && y <= 11.4) continue;
    cells.push({
      cellId: `D_L2_${cellSeq++}`,
      level: 2,
      x: Number(x.toFixed(3)),
      y: Number(y.toFixed(3)),
      sizeMeters: 0.20,
      pointCount: 5,
      elevationMean: 0.0,
      elevationMin: -0.05,
      elevationMax: 0.05,
      resolutionReasons: ['DISTANCE_BASELINE'],
    });
  }
}

// L3: 40 cm (0.40 m)
// Distant road and wide open intersection perimeter
for (let y = -4.0; y <= 3.6; y += 0.80) {
  for (let x = -4.0; x <= 4.0; x += 0.80) {
    cells.push({
      cellId: `D_L3_NEAR_${cellSeq++}`,
      level: 3,
      x: Number(x.toFixed(3)),
      y: Number(y.toFixed(3)),
      sizeMeters: 0.40,
      pointCount: 4,
      elevationMean: 0.0,
      elevationMin: -0.05,
      elevationMax: 0.05,
      resolutionReasons: ['DISTANCE_BASELINE'],
    });
  }
}

for (let y = 21.4; y <= 32.0; y += 0.80) {
  for (let x = -4.0; x <= 4.0; x += 0.80) {
    cells.push({
      cellId: `D_L3_FAR_${cellSeq++}`,
      level: 3,
      x: Number(x.toFixed(3)),
      y: Number(y.toFixed(3)),
      sizeMeters: 0.40,
      pointCount: 3,
      elevationMean: 0.0,
      elevationMin: -0.05,
      elevationMax: 0.05,
      resolutionReasons: ['DISTANCE_BASELINE'],
    });
  }
}

// Cross street extensions (X: -14 to -4 and +4 to +14, Y: 12 to 17)
for (let x = -14.0; x <= -4.4; x += 0.80) {
  for (let y = 12.0; y <= 17.0; y += 0.80) {
    cells.push({
      cellId: `D_L3_CROSS_L_${cellSeq++}`,
      level: 3,
      x: Number(x.toFixed(3)),
      y: Number(y.toFixed(3)),
      sizeMeters: 0.40,
      pointCount: 3,
      elevationMean: 0.0,
      elevationMin: -0.05,
      elevationMax: 0.05,
      resolutionReasons: ['DISTANCE_BASELINE'],
    });
  }
}
for (let x = 4.4; x <= 14.0; x += 0.80) {
  for (let y = 12.0; y <= 17.0; y += 0.80) {
    cells.push({
      cellId: `D_L3_CROSS_R_${cellSeq++}`,
      level: 3,
      x: Number(x.toFixed(3)),
      y: Number(y.toFixed(3)),
      sizeMeters: 0.40,
      pointCount: 3,
      elevationMean: 0.0,
      elevationMin: -0.05,
      elevationMax: 0.05,
      resolutionReasons: ['DISTANCE_BASELINE'],
    });
  }
}

// ============================================================================
// 4. EXPORT AUTHORITATIVE SCENARIO D FRAME
// ============================================================================

const activeCells = withCurrentReturns(buildActiveLeaves(cells), points);

export const scenarioD: PerceptionFrame = {
  frameSeq: 104,
  pipelineStatus: 'SUCCESS',
  mode: 'MIXED_INTERSECTION',
  scenarioId: 'D',
  scenarioTitle: 'Scenario D · Mixed Intersection',
  scenarioDescription:
    'Complex multi-hazard scene. Simultaneous moving vehicle and pedestrian hazards, showing wide hierarchical resolution allocation and frame-local cluster extraction.',
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
