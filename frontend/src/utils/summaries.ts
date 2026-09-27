import {
  AdaptiveCell,
  DynamicObjectCluster,
  DynamicPoint,
  DynamicSummary,
  PerceptionPoint,
  ResolutionSummary,
  SemanticSummary,
} from '../types/perception';

export function calculateResolutionSummary(cells: AdaptiveCell[]): ResolutionSummary {
  const total = cells.length;
  if (total === 0) {
    return {
      totalCells: 0,
      l0Count: 0,
      l1Count: 0,
      l2Count: 0,
      l3Count: 0,
      l0Percent: 0,
      l1Percent: 0,
      l2Percent: 0,
      l3Percent: 0,
    };
  }

  let l0 = 0;
  let l1 = 0;
  let l2 = 0;
  let l3 = 0;

  for (const c of cells) {
    if (c.level === 0) l0++;
    else if (c.level === 1) l1++;
    else if (c.level === 2) l2++;
    else if (c.level === 3) l3++;
  }

  return {
    totalCells: total,
    l0Count: l0,
    l1Count: l1,
    l2Count: l2,
    l3Count: l3,
    l0Percent: Math.round((l0 / total) * 100),
    l1Percent: Math.round((l1 / total) * 100),
    l2Percent: Math.round((l2 / total) * 100),
    l3Percent: Math.round((l3 / total) * 100),
  };
}

export function calculateSemanticSummary(points: PerceptionPoint[]): SemanticSummary {
  const summary: SemanticSummary = {
    UNKNOWN: 0,
    GROUND: 0,
    ROAD: 0,
    VEGETATION: 0,
    BUILDING: 0,
    VEHICLE: 0,
    PEDESTRIAN: 0,
    OBSTACLE_OTHER: 0,
  };

  for (const pt of points) {
    if (pt.semanticClass in summary) {
      summary[pt.semanticClass]++;
    }
  }

  return summary;
}

export function calculateDynamicSummary(
  dynamicPoints: DynamicPoint[],
  clusters: DynamicObjectCluster[],
  noise: number[]
): DynamicSummary {
  return {
    dynamicPoints: dynamicPoints.length,
    clusterCount: clusters.length,
    noiseCount: noise.length,
  };
}
