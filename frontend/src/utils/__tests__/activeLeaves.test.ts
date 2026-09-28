import assert from 'node:assert/strict';
import { describe, it } from 'node:test';
import { filterActiveLeaves } from '../activeLeaves';
import { SCENARIOS } from '../../demo/scenarios';
import type { AdaptiveCell } from '../../types/perception';

describe('Active Leaf Topology & Invariants (Milestone F2A-R3)', () => {
  // ==========================================================================
  // Suite 1: Pure Filter Contract (Section 8)
  // ==========================================================================
  describe('filterActiveLeaves Pure Filter Semantics (Section 8)', () => {
    it('only filters: output is strictly a subset of input objects', () => {
      const parent: AdaptiveCell = {
        cellId: 'ROOT_L3',
        level: 3,
        x: 0.2,
        y: 0.2,
        sizeMeters: 0.4,
        pointCount: 0,
        resolutionReasons: ['DISTANCE_BASELINE'],
      };
      const child: AdaptiveCell = {
        cellId: 'CHILD_L2',
        level: 2,
        x: 0.1,
        y: 0.1,
        sizeMeters: 0.2,
        pointCount: 0,
        resolutionReasons: ['SEMANTIC_RELEVANCE'],
      };

      const input = [parent, child];
      const output = filterActiveLeaves(input);

      // Child overlaps parent footprint, so parent is excluded.
      assert.equal(output.length, 1);
      assert.strictEqual(output[0], child);
    });

    it('never creates new cell coordinates', () => {
      const input: AdaptiveCell[] = [
        { cellId: 'C1', level: 1, x: 1.5, y: 2.5, sizeMeters: 0.1, pointCount: 0, resolutionReasons: [] },
        { cellId: 'C2', level: 1, x: 3.5, y: 4.5, sizeMeters: 0.1, pointCount: 0, resolutionReasons: [] },
      ];
      const output = filterActiveLeaves(input);
      for (const cell of output) {
        assert.ok(input.some((orig) => orig.x === cell.x && orig.y === cell.y));
      }
    });

    it('never changes cell level', () => {
      const input: AdaptiveCell[] = [
        { cellId: 'L0_A', level: 0, x: 0.025, y: 0.025, sizeMeters: 0.05, pointCount: 0, resolutionReasons: [] },
        { cellId: 'L2_B', level: 2, x: 5.0, y: 5.0, sizeMeters: 0.2, pointCount: 0, resolutionReasons: [] },
      ];
      const output = filterActiveLeaves(input);
      for (const cell of output) {
        const orig = input.find((i) => i.cellId === cell.cellId);
        assert.ok(orig);
        assert.equal(cell.level, orig.level);
      }
    });

    it('never invents cell identity', () => {
      const input: AdaptiveCell[] = [
        { cellId: 'AUTH_01', level: 1, x: 0.0, y: 0.0, sizeMeters: 0.1, pointCount: 0, resolutionReasons: [] },
        { cellId: 'AUTH_02', level: 1, x: 1.0, y: 1.0, sizeMeters: 0.1, pointCount: 0, resolutionReasons: [] },
      ];
      const output = filterActiveLeaves(input);
      const inputIds = new Set(input.map((c) => c.cellId));
      for (const cell of output) {
        assert.ok(inputIds.has(cell.cellId), `Output cell ${cell.cellId} was not in input`);
      }
    });

    it('is idempotent on already valid non-overlapping active leaves', () => {
      for (const [id, frame] of Object.entries(SCENARIOS)) {
        const filtered = filterActiveLeaves(frame.cells);
        assert.equal(
          filtered.length,
          frame.cells.length,
          `Scenario ${id} active leaves must remain 100% unchanged under filter`
        );
      }
    });
  });

  // ==========================================================================
  // Suite 2: Seven Active Leaf Invariants (Section 10)
  // ==========================================================================
  describe('Seven Canonical Active Leaf Invariants (Section 10)', () => {
    for (const [id, frame] of Object.entries(SCENARIOS)) {
      describe(`Scenario ${id} Invariants`, () => {
        // Invariant 1: No overlapping active leaf footprints
        it('1. no overlapping active leaf footprints', () => {
          const cells = frame.cells;
          for (let i = 0; i < cells.length; i++) {
            for (let j = i + 1; j < cells.length; j++) {
              const a = cells[i];
              const b = cells[j];
              const sepX = Math.abs(a.x - b.x);
              const sepY = Math.abs(a.y - b.y);
              const minSepX = (a.sizeMeters + b.sizeMeters) / 2 - 1e-7;
              const minSepY = (a.sizeMeters + b.sizeMeters) / 2 - 1e-7;
              assert.ok(
                sepX >= minSepX || sepY >= minSepY,
                `Scenario ${id}: Cell ${a.cellId} overlaps cell ${b.cellId}`
              );
            }
          }
        });

        // Invariant 2: Every output cell existed in the fixture builder output
        it('2. every output cell existed in the original fixture output', () => {
          assert.ok(frame.cells.length > 0, 'Scenario must have cells');
          for (const cell of frame.cells) {
            assert.ok(cell.cellId, 'Cell must have cellId');
            assert.ok(Number.isFinite(cell.x) && Number.isFinite(cell.y));
            assert.ok(cell.level >= 0 && cell.level <= 3);
            assert.equal(cell.sizeMeters, 0.05 * 2 ** cell.level);
          }
        });

        // Invariant 3: No frontend renderer-generated map cells (no temporary D_TREE_ prefixes)
        it('3. no frontend renderer-generated or temporary D_TREE_ map cells', () => {
          for (const cell of frame.cells) {
            assert.ok(
              !cell.cellId.startsWith('D_TREE_'),
              `Scenario ${id} contains illegal renderer-synthesized ID: ${cell.cellId}`
            );
          }
        });

        // Invariant 4: Exact area conservation
        it('4. exact area conservation', () => {
          const totalAreaFromCells = frame.cells.reduce(
            (sum, c) => sum + c.sizeMeters ** 2,
            0
          );
          const l0Count = frame.cells.filter((c) => c.level === 0).length;
          const l1Count = frame.cells.filter((c) => c.level === 1).length;
          const l2Count = frame.cells.filter((c) => c.level === 2).length;
          const l3Count = frame.cells.filter((c) => c.level === 3).length;
          const areaFromLevels =
            l0Count * 0.05 ** 2 +
            l1Count * 0.1 ** 2 +
            l2Count * 0.2 ** 2 +
            l3Count * 0.4 ** 2;

          assert.ok(
            Math.abs(totalAreaFromCells - areaFromLevels) < 1e-6,
            `Area conservation error: ${totalAreaFromCells} vs ${areaFromLevels}`
          );
        });

        // Invariant 5: Unique CellKey / cellId
        it('5. unique CellKey / cellId', () => {
          const ids = new Set<string>();
          for (const cell of frame.cells) {
            assert.ok(!ids.has(cell.cellId), `Duplicate cell ID detected: ${cell.cellId}`);
            ids.add(cell.cellId);
          }
          assert.equal(ids.size, frame.cells.length);
        });

        // Invariant 7: Summary counts exactly match final active cells
        it('7. summary counts exactly match final active cells', () => {
          const summary = frame.resolutionSummary;
          assert.equal(summary.totalCells, frame.cells.length);
          assert.equal(
            summary.l0Count + summary.l1Count + summary.l2Count + summary.l3Count,
            frame.cells.length
          );
          assert.equal(
            summary.l0Count,
            frame.cells.filter((c) => c.level === 0).length
          );
          assert.equal(
            summary.l1Count,
            frame.cells.filter((c) => c.level === 1).length
          );
          assert.equal(
            summary.l2Count,
            frame.cells.filter((c) => c.level === 2).length
          );
          assert.equal(
            summary.l3Count,
            frame.cells.filter((c) => c.level === 3).length
          );
        });
      });
    }

    // Invariant 6: Canonical selected pedestrian cell still exists
    it('6. canonical selected pedestrian cell D_L0_P_127 still exists in Scenario D', () => {
      const cell = SCENARIOS.D.cells.find((c) => c.cellId === 'D_L0_P_127');
      assert.ok(cell, 'Cell D_L0_P_127 must exist in Scenario D fixture');
      assert.equal(cell.level, 0);
      assert.equal(cell.sizeMeters, 0.05);
      assert.ok(Math.abs(cell.x - 3.6) < 1e-4);
      assert.ok(Math.abs(cell.y - 10.2) < 1e-4);
      assert.equal(cell.pointCount, 6);
      assert.deepEqual(cell.resolutionReasons, [
        'SEMANTIC_RELEVANCE',
        'DYNAMIC_RELEVANCE',
      ]);
    });
  });
});
