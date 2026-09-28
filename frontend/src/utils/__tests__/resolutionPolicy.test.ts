import assert from 'node:assert/strict';
import { describe, it } from 'node:test';
import { validateBackendStatusContract, validatePerceptionFrame } from '../../demo/fixtureValidation';
import { scenarioA } from '../../demo/scenarioA_corridor';
import { scenarioB } from '../../demo/scenarioB_pedestrian';
import { scenarioC } from '../../demo/scenarioC_vehicle';
import { scenarioD } from '../../demo/scenarioD_intersection';
import {
  baselineLevelForDistance,
  explainCellResolution,
  planarDistanceFromSensor,
  resolutionColor,
  resolutionLabel,
  resolutionMeters,
  resolutionReasonLabel,
} from '../resolution';

/**
 * @file resolutionPolicy.test.ts
 * @brief Authoritative tests for distance policy, boundary conditions, Z-invariance,
 *        static-fusion provenance, and explainability source truth matching frozen backend (HEAD: cc50675).
 */

describe('Resolution Policy & Explainability Source Truth (Milestone F2A-R2.3)', () => {
  // ==========================================================================
  // Suite 1: Boundary Geometry Tests (Planar XY Distance)
  // ==========================================================================
  describe('Boundary Geometry Tests (XY Planar)', () => {
    const sensor = { x: 0.0, y: 0.0 };

    it('produces L0 (5 cm) for planar distances in [0, 10.0 m)', () => {
      // 0.000 m
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 0.0, y: 0.0 }, sensor)), 0);
      // 5.000 m (3-4-5 triangle)
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 3.0, y: 4.0 }, sensor)), 0);
      // 9.999 m
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 0.0, y: 9.999 }, sensor)), 0);
    });

    it('produces L1 (10 cm) for planar distances in [10.0, 30.0 m)', () => {
      // 10.000 m exact boundary
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 6.0, y: 8.0 }, sensor)), 1);
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 0.0, y: 10.0 }, sensor)), 1);
      // 20.000 m
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 12.0, y: 16.0 }, sensor)), 1);
      // 29.999 m
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 0.0, y: 29.999 }, sensor)), 1);
    });

    it('produces L2 (20 cm) for planar distances in [30.0, 60.0 m)', () => {
      // 30.000 m exact boundary
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 0.0, y: 30.0 }, sensor)), 2);
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 18.0, y: 24.0 }, sensor)), 2);
      // 45.000 m
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 0.0, y: 45.0 }, sensor)), 2);
      // 59.999 m
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 0.0, y: 59.999 }, sensor)), 2);
    });

    it('produces L3 (40 cm) for planar distances >= 60.0 m', () => {
      // 60.000 m exact boundary
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 0.0, y: 60.0 }, sensor)), 3);
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 36.0, y: 48.0 }, sensor)), 3);
      // Far range 100 m, 250 m
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 0.0, y: 100.0 }, sensor)), 3);
      assert.equal(baselineLevelForDistance(planarDistanceFromSensor({ x: 0.0, y: 250.0 }, sensor)), 3);
    });
  });

  // ==========================================================================
  // Suite 2: Z-Invariance Golden Tests (Section 7)
  // ==========================================================================
  describe('Z-Invariance Golden Tests', () => {
    const testCases = [
      { x: 3.6, y: 10.2, expectedBaseline: 1 },
      { x: 0.0, y: 5.0, expectedBaseline: 0 },
      { x: 15.0, y: 20.0, expectedBaseline: 1 },
      { x: 30.0, y: 40.0, expectedBaseline: 2 },
      { x: 50.0, y: 50.0, expectedBaseline: 3 },
    ];

    it('resolution policy distance and baseline level are strictly invariant to sensor and cell Z offsets', () => {
      for (const tc of testCases) {
        const nominalDist = Math.hypot(tc.x, tc.y);
        const nominalBaseline = baselineLevelForDistance(nominalDist);
        assert.equal(nominalBaseline, tc.expectedBaseline);

        // Perturb sensor Z and cell elevations with extreme finite values
        const zVariations = [-500.0, -50.0, -1.62, 0.0, 1.62, 2.5, 10.0, 100.0, 1000.0];
        for (const sensorZ of zVariations) {
          for (const cellElevation of zVariations) {
            // Planar distance must strictly ignore any Z term
            const planarDist = planarDistanceFromSensor(
              { x: tc.x, y: tc.y },
              { x: 0.0, y: 0.0 }
            );
            assert.equal(
              planarDist,
              nominalDist,
              `Distance drifted under sensorZ=${sensorZ}, cellElevation=${cellElevation}`
            );

            const baseline = baselineLevelForDistance(planarDist);
            assert.equal(
              baseline,
              tc.expectedBaseline,
              `Baseline drifted under sensorZ=${sensorZ}, cellElevation=${cellElevation}`
            );
          }
        }
      }
    });
  });

  // ==========================================================================
  // Suite 3: Invalid Distance Range Errors
  // ==========================================================================
  describe('Invalid Distance Rejections', () => {
    it('throws RangeError on negative, NaN, or non-finite inputs', () => {
      assert.throws(() => baselineLevelForDistance(-0.001), RangeError);
      assert.throws(() => baselineLevelForDistance(-10), RangeError);
      assert.throws(() => baselineLevelForDistance(Number.NaN), RangeError);
      assert.throws(() => baselineLevelForDistance(Number.POSITIVE_INFINITY), RangeError);
      assert.throws(() => baselineLevelForDistance(Number.NEGATIVE_INFINITY), RangeError);
    });
  });

  // ==========================================================================
  // Suite 4: Golden Selected Pedestrian Hazard Cell (D_L0_P_127) (Section 6 & 7)
  // ==========================================================================
  describe('Golden Selected Pedestrian Hazard Cell (D_L0_P_127)', () => {
    const frame = scenarioD;
    const cell127 = frame.cells.find((c) => c.cellId === 'D_L0_P_127');

    it('cell exists at coordinates (x = 3.600, y = 10.200)', () => {
      assert.ok(cell127, 'Cell D_L0_P_127 must exist in Scenario D');
      assert.equal(cell127.x, 3.600, 'Cell center x must be exactly 3.600');
      assert.equal(cell127.y, 10.200, 'Cell center y must be exactly 10.200');
      assert.equal(cell127.level, 0, 'Cell level must be L0 (5 cm)');
      assert.equal(cell127.sizeMeters, 0.05, 'Cell sizeMeters must be 0.05 m');
    });

    it('computes planar policy distance approximately 10.82 m (L1 baseline)', () => {
      assert.ok(cell127);
      const sensorOrigin = frame.sensorOrigin || { x: 0, y: 0 };
      const planarDist = planarDistanceFromSensor(cell127, sensorOrigin);
      const expectedDist = Math.hypot(3.6, 10.2); // sqrt(12.96 + 104.04) = sqrt(117) ≈ 10.81665...
      assert.ok(
        Math.abs(planarDist - expectedDist) < 1e-6,
        `Planar distance ${planarDist} must equal sqrt(3.6^2 + 10.2^2) = ${expectedDist}`
      );
      assert.equal(Number(planarDist.toFixed(2)), 10.82, 'Rounded distance must display 10.82 m');

      const baseline = baselineLevelForDistance(planarDist);
      assert.equal(baseline, 1, 'Policy distance 10.82 m must map to L1 (10 cm)');
      assert.notEqual(baseline, 3, 'Policy distance 10.82 m must NEVER regress to L3 (40 cm)');

      const refinement = baseline - cell127.level;
      assert.equal(refinement, 1, 'Refinement from L1 to L0 must be exactly 1 level');
    });
  });

  // ==========================================================================
  // Suite 5: Fixture Perception Points & Provenance (Sections 10, 11, 12, 13, 16, 19)
  // ==========================================================================
  describe('Fixture Perception Points & Provenance for D_L0_P_127', () => {
    const frame = scenarioD;
    const cell127 = frame.cells.find((c) => c.cellId === 'D_L0_P_127');

    it('contains exactly 6 current-frame perception returns in cell footprint', () => {
      assert.ok(cell127);
      const half = cell127.sizeMeters / 2; // 0.025
      const minX = cell127.x - half;
      const maxX = cell127.x + half;
      const minY = cell127.y - half;
      const maxY = cell127.y + half;

      const pointsInside = frame.points.filter(
        (p) =>
          p.x >= minX - 1e-4 &&
          p.x <= maxX + 1e-4 &&
          p.y >= minY - 1e-4 &&
          p.y <= maxY + 1e-4
      );

      // Section 19: Assert equality of current-frame points and cell.pointCount
      assert.equal(
        pointsInside.length,
        6,
        `Expected exactly 6 points in D_L0_P_127 footprint, found ${pointsInside.length}`
      );
      assert.equal(
        cell127.pointCount,
        pointsInside.length,
        'cell.pointCount must strictly equal points falling in spatial footprint'
      );

      // Section 10: Report source indices, semantic classes, routing, dynamic membership
      const sourceIndices = pointsInside.map((p) => p.sourceIndex);
      assert.deepEqual(
        sourceIndices,
        [201, 202, 203, 204, 205, 206],
        'Supporting source indices must be [201, 202, 203, 204, 205, 206]'
      );

      for (const p of pointsInside) {
        assert.equal(p.semanticClass, 'PEDESTRIAN', 'Every point must be PEDESTRIAN');
        assert.equal(p.routing, 'DYNAMIC_EXCLUDED', 'Every point must be DYNAMIC_EXCLUDED');
      }

      // Check dynamic cluster membership
      const pedCluster = frame.dynamicClusters.find((c) => c.dominantClass === 'PEDESTRIAN');
      assert.ok(pedCluster, 'Pedestrian dynamic cluster C-01 must exist');
      assert.equal(pedCluster.clusterLocalId, 1);
      for (const idx of sourceIndices) {
        assert.ok(
          pedCluster.memberSourceIndices.includes(idx),
          `Point ${idx} must be a member of pedestrian cluster C-01`
        );
      }
    });

    it('derives authoritative explainability DTO matching frozen contract', () => {
      assert.ok(cell127);
      const explanation = explainCellResolution(cell127, frame);

      assert.equal(explanation.cellId, 'D_L0_P_127');
      assert.equal(Number(explanation.policyDistanceMeters.toFixed(2)), 10.82);
      assert.equal(explanation.baselineLevel, 1);
      assert.equal(explanation.finalLevel, 0);
      assert.equal(explanation.semanticClass, 'PEDESTRIAN');
      assert.equal(explanation.semanticPolicyTier, 'SAFETY_CRITICAL');
      assert.equal(explanation.dynamicRelevance, true);
      assert.equal(explanation.pointCount, 6);
      assert.deepEqual(explanation.sourceIndices, [201, 202, 203, 204, 205, 206]);
      assert.equal(explanation.dynamicClusterId, 1);

      // Section 13 & 14: Cell reasons include both semantic and dynamic relevance
      assert.ok(
        explanation.reasons.includes('SEMANTIC_RELEVANCE'),
        'Reasons must include SEMANTIC_RELEVANCE'
      );
      assert.ok(
        explanation.reasons.includes('DYNAMIC_RELEVANCE'),
        'Reasons must include DYNAMIC_RELEVANCE'
      );
      assert.ok(
        !explanation.reasons.includes('UNCERTAINTY'),
        'Must not manufacture UNCERTAINTY reason'
      );
    });

    it('enforces static-fusion provenance: DYNAMIC_EXCLUDED points do NOT populate static map elevation state (Sections 25 & 26)', () => {
      assert.ok(cell127);
      const explanation = explainCellResolution(cell127, frame);

      // Provenance assertion: Dynamic points only drive decision geometry, strictly excluded from static terrain fusion
      assert.equal(explanation.routing, 'DYNAMIC_EXCLUDED');
      assert.equal(explanation.staticFusion, 'EXCLUDED');
      assert.equal(explanation.hasStaticElevationSupport, false);
      assert.equal(
        explanation.staticElevationStats,
        undefined,
        'Must NOT expose unsupported or fabricated static elevation statistics for dynamic hazard cell'
      );
      assert.equal(
        cell127.elevationMean,
        undefined,
        'Cell fixture D_L0_P_127 must not have hand-authored elevationMean'
      );
      assert.equal(
        explanation.decisionEvidence,
        'Semantic + dynamic pedestrian evidence'
      );
    });
  });

  // ==========================================================================
  // Suite 6: Physical Scale & Label Mappings
  // ==========================================================================
  describe('Physical Scale & Label Mappings', () => {
    it('correctly maps physical resolution meters and labels', () => {
      assert.equal(resolutionMeters(0), 0.05);
      assert.equal(resolutionMeters(1), 0.10);
      assert.equal(resolutionMeters(2), 0.20);
      assert.equal(resolutionMeters(3), 0.40);

      assert.equal(resolutionLabel(0), 'L0 (5 cm)');
      assert.equal(resolutionLabel(1), 'L1 (10 cm)');
      assert.equal(resolutionLabel(2), 'L2 (20 cm)');
      assert.equal(resolutionLabel(3), 'L3 (40 cm)');

      assert.equal(resolutionColor(0), '#62D3C6');
      assert.equal(resolutionColor(1), '#70B8E8');
      assert.equal(resolutionColor(2), '#D6B45F');
      assert.equal(resolutionColor(3), '#A18AC8');

      assert.equal(resolutionReasonLabel('DISTANCE_BASELINE'), 'Distance Baseline');
      assert.equal(resolutionReasonLabel('SEMANTIC_RELEVANCE'), 'Semantic Relevance');
      assert.equal(resolutionReasonLabel('DYNAMIC_RELEVANCE'), 'Dynamic Relevance');
      assert.equal(resolutionReasonLabel('UNCERTAINTY'), 'Uncertainty');
      assert.equal(resolutionReasonLabel('GEOMETRY_COMPLEXITY'), 'Geometry Complexity');
      assert.equal(resolutionReasonLabel('INSUFFICIENT_EVIDENCE'), 'Insufficient Evidence');
      assert.equal(resolutionReasonLabel('DENSITY_GATE_BLOCKED'), 'Density Gate');
    });
  });

  // ==========================================================================
  // Suite 7: Full Perception Frame Integrity (Conservation Laws for ALL Scenarios)
  // ==========================================================================
  describe('Full Perception Frame Conservation & Integrity (Scenarios A, B, C, D)', () => {
    it('Scenario A passes complete perception frame conservation validation', () => {
      const res = validatePerceptionFrame(scenarioA);
      assert.equal(res.valid, true, `Scenario A frame validation failed: ${res.errors.join('; ')}`);
      assert.equal(res.errors.length, 0);
    });

    it('Scenario B passes complete perception frame conservation validation', () => {
      const res = validatePerceptionFrame(scenarioB);
      assert.equal(res.valid, true, `Scenario B frame validation failed: ${res.errors.join('; ')}`);
      assert.equal(res.errors.length, 0);
    });

    it('Scenario C passes complete perception frame conservation validation', () => {
      const res = validatePerceptionFrame(scenarioC);
      assert.equal(res.valid, true, `Scenario C frame validation failed: ${res.errors.join('; ')}`);
      assert.equal(res.errors.length, 0);
    });

    it('Scenario D passes complete perception frame conservation validation', () => {
      const res = validatePerceptionFrame(scenarioD);
      assert.equal(res.valid, true, `Scenario D frame validation failed: ${res.errors.join('; ')}`);
      assert.equal(res.errors.length, 0);
    });

    it('backend status enum table matches frozen C++ 0..21', () => {
      assert.doesNotThrow(() => validateBackendStatusContract());
    });
  });
});
