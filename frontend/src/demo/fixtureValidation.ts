import {
  AdaptivePipelineStatus,
  PerceptionFrame,
} from '../types/perception';
import {
  calculateDynamicSummary,
  calculateResolutionSummary,
  calculateSemanticSummary,
} from '../utils/summaries';

export interface ValidationResult {
  valid: boolean;
  errors: string[];
}

/**
 * @brief Strictly validate the internal consistency and conservation laws of a perception frame.
 */
export function validatePerceptionFrame(frame: PerceptionFrame): ValidationResult {
  const errors: string[] = [];

  const sourceById = new Map(frame.points.map(p => [p.sourceIndex, p]));
  if (sourceById.size !== frame.points.length) errors.push('Duplicate perception source index');
  if (new Set(frame.dynamicPoints.map(p=>p.sourceIndex)).size !== frame.dynamicPoints.length) errors.push('Duplicate dynamic source index');
  if (new Set(frame.noiseSourceIndices).size !== frame.noiseSourceIndices.length) errors.push('Duplicate noise source index');
  for (const p of frame.dynamicPoints) {
    const source = sourceById.get(p.sourceIndex);
    if (!source || source.routing !== 'DYNAMIC_EXCLUDED' || source.x !== p.position.x || source.y !== p.position.y || source.z !== p.position.z) errors.push(`Dynamic source ${p.sourceIndex} does not match perception observation`);
  }

  // 1. Resolution summary equality
  const expectedRes = calculateResolutionSummary(frame.cells);
  if (
    frame.resolutionSummary.totalCells !== expectedRes.totalCells ||
    frame.resolutionSummary.l0Count !== expectedRes.l0Count ||
    frame.resolutionSummary.l1Count !== expectedRes.l1Count ||
    frame.resolutionSummary.l2Count !== expectedRes.l2Count ||
    frame.resolutionSummary.l3Count !== expectedRes.l3Count
  ) {
    errors.push(
      `Scenario ${frame.scenarioId}: Resolution summary mismatch. Declared ${frame.resolutionSummary.totalCells} cells vs derived ${expectedRes.totalCells}`
    );
  }

  // 2. Semantic summary equality
  const expectedSem = calculateSemanticSummary(frame.points);
  for (const [k, v] of Object.entries(expectedSem)) {
    if (frame.semanticSummary[k as keyof typeof expectedSem] !== v) {
      errors.push(
        `Scenario ${frame.scenarioId}: Semantic summary mismatch for class ${k}. Declared ${frame.semanticSummary[k as keyof typeof expectedSem]} vs derived ${v}`
      );
    }
  }

  // 3. Dynamic counts equality
  const expectedDyn = calculateDynamicSummary(
    frame.dynamicPoints,
    frame.dynamicClusters,
    frame.noiseSourceIndices
  );
  if (
    frame.dynamicSummary.dynamicPoints !== expectedDyn.dynamicPoints ||
    frame.dynamicSummary.clusterCount !== expectedDyn.clusterCount ||
    frame.dynamicSummary.noiseCount !== expectedDyn.noiseCount
  ) {
    errors.push(
      `Scenario ${frame.scenarioId}: Dynamic summary mismatch. Declared ${frame.dynamicSummary.dynamicPoints} dynamic points vs derived ${expectedDyn.dynamicPoints}`
    );
  }

  // 4. Point routing invariant:
  // - All DYNAMIC_EXCLUDED points in points array must be present in dynamicPoints
  // - No STATIC_FUSABLE point may be in dynamicPoints
  const dynExcludedPoints = frame.points.filter((p) => p.routing === 'DYNAMIC_EXCLUDED');
  const dynPointsSourceIndices = new Set(frame.dynamicPoints.map((p) => p.sourceIndex));

  for (const p of dynExcludedPoints) {
    if (!dynPointsSourceIndices.has(p.sourceIndex)) {
      errors.push(
        `Scenario ${frame.scenarioId}: DYNAMIC_EXCLUDED point sourceIndex ${p.sourceIndex} missing from dynamicPoints set`
      );
    }
  }

  const staticPointsSourceIndices = new Set(
    frame.points.filter((p) => p.routing === 'STATIC_FUSABLE').map((p) => p.sourceIndex)
  );

  for (const dp of frame.dynamicPoints) {
    if (staticPointsSourceIndices.has(dp.sourceIndex)) {
      errors.push(
        `Scenario ${frame.scenarioId}: STATIC_FUSABLE point sourceIndex ${dp.sourceIndex} illegally present in dynamicPoints set`
      );
    }
  }

  // 5. Cluster member verification and uniqueness
  const dynamicById = new Map(frame.dynamicPoints.map(p => [p.sourceIndex, p]));
  const seenClusterMembers = new Set<number>();
  for (const cluster of frame.dynamicClusters) {
    if (cluster.pointCount !== cluster.memberSourceIndices.length) {
      errors.push(
        `Scenario ${frame.scenarioId}: Cluster ${cluster.clusterLocalId} pointCount (${cluster.pointCount}) does not match member array length (${cluster.memberSourceIndices.length})`
      );
    }

    for (const srcIdx of cluster.memberSourceIndices) {
      if (!dynPointsSourceIndices.has(srcIdx)) {
        errors.push(
          `Scenario ${frame.scenarioId}: Cluster ${cluster.clusterLocalId} references non-existent dynamic source index ${srcIdx}`
        );
      }
      if (seenClusterMembers.has(srcIdx)) {
        errors.push(
          `Scenario ${frame.scenarioId}: Duplicate cluster membership detected for source index ${srcIdx}`
        );
      }
      seenClusterMembers.add(srcIdx);
    }

    const members = cluster.memberSourceIndices.flatMap(id => {
      const point = dynamicById.get(id);
      return point ? [point] : [];
    });
    if (members.length === 0) errors.push(`Cluster ${cluster.clusterLocalId} has no source members`);
    else if (members.length === cluster.memberSourceIndices.length) {
      const same = (a: number, b: number) => Number.isFinite(a) && Math.abs(a - b) < 1e-9;
      if (cluster.canonicalSourcePointIndex !== Math.min(...cluster.memberSourceIndices)) errors.push(`Cluster ${cluster.clusterLocalId} canonical index mismatch`);
      for (const axis of ['x', 'y', 'z'] as const) {
        const values = members.map(p => p.position[axis]);
        if (!same(cluster.aabbMin[axis], Math.min(...values)) || !same(cluster.aabbMax[axis], Math.max(...values))) errors.push(`Cluster ${cluster.clusterLocalId} ${axis} bounds do not match source members`);
        if (!same(cluster.centroid[axis], values.reduce((sum, value) => sum + value, 0) / values.length)) errors.push(`Cluster ${cluster.clusterLocalId} ${axis} centroid does not match source members`);
      }
      if (!same(cluster.meanSemanticConfidence, members.reduce((sum, p) => sum + p.semanticConfidence, 0) / members.length) || !same(cluster.meanMotionConfidence, members.reduce((sum, p) => sum + p.motionConfidence, 0) / members.length)) errors.push(`Cluster ${cluster.clusterLocalId} confidence means do not match source members`);
    }
  }

  // 6. Cluster Conservation: Clustered dynamic points + noise points must equal total dynamic points
  const noiseSet = new Set(frame.noiseSourceIndices);
  for (const noiseIdx of frame.noiseSourceIndices) {
    if (!dynPointsSourceIndices.has(noiseIdx)) {
      errors.push(
        `Scenario ${frame.scenarioId}: Noise source index ${noiseIdx} does not exist in dynamicPoints`
      );
    }
    if (seenClusterMembers.has(noiseIdx)) {
      errors.push(
        `Scenario ${frame.scenarioId}: Point ${noiseIdx} is partitioned as both clustered member and noise`
      );
    }
  }

  if (seenClusterMembers.size + noiseSet.size !== frame.dynamicPoints.length) {
    errors.push(
      `Scenario ${frame.scenarioId}: Dynamic point conservation violation: ${seenClusterMembers.size} clustered + ${noiseSet.size} noise != ${frame.dynamicPoints.length} total dynamic points`
    );
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * @brief Assert that AdaptivePipelineStatus numeric table exactly matches C++ backend 0..21.
 */
export function validateBackendStatusContract(): void {
  const expectedCount = 22;
  for (let i = 0; i < expectedCount; ++i) {
    const name = AdaptivePipelineStatus[i];
    if (typeof name !== 'string') {
      throw new Error(`AdaptivePipelineStatus drift: index ${i} is not defined in enum`);
    }
    const backCode = AdaptivePipelineStatus[name as keyof typeof AdaptivePipelineStatus];
    if (backCode !== i) {
      throw new Error(
        `AdaptivePipelineStatus bidirectional mismatch for ${name}: code ${backCode} vs expected ${i}`
      );
    }
  }
}
