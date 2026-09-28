import type { AdaptiveCell, ResolutionLevel } from '../types/perception';

/**
 * @file activeLeaves.ts
 * @brief Authoritative active-leaf spatial topology utilities.
 *
 * Invariant:
 * Exactly one active leaf owns any occupied 2D spatial coordinate.
 */

/**
 * @brief Pure filter that selects only active leaf cells from a spatial hierarchy.
 *
 * A cell is considered an active leaf if and only if NO finer child cell (level < cell.level)
 * in the collection overlaps its 2D spatial footprint.
 *
 * STRICT PURE-FILTER GUARANTEES (Milestone F2A-R3 Section 8):
 * - It NEVER creates new cell coordinates.
 * - It NEVER changes any cell level.
 * - It NEVER invents cell identities or synthesis IDs.
 * - It ONLY filters: output is strictly a subset of the input array.
 */
export function filterActiveLeaves(cells: AdaptiveCell[]): AdaptiveCell[] {
  return cells.filter((candidate) => {
    const half = candidate.sizeMeters / 2;
    const minX = candidate.x - half;
    const maxX = candidate.x + half;
    const minY = candidate.y - half;
    const maxY = candidate.y + half;

    for (const other of cells) {
      if (other.level < candidate.level) {
        const otherHalf = other.sizeMeters / 2;
        const oMinX = other.x - otherHalf;
        const oMaxX = other.x + otherHalf;
        const oMinY = other.y - otherHalf;
        const oMaxY = other.y + otherHalf;

        // Bounding box overlap with 1e-7 epsilon for floating-point tolerance
        const overlaps =
          oMinX < maxX - 1e-7 &&
          oMaxX > minX + 1e-7 &&
          oMinY < maxY - 1e-7 &&
          oMaxY > minY + 1e-7;

        if (overlaps) {
          // A finer child exists within this cell's footprint; candidate is an internal node
          return false;
        }
      }
    }
    return true;
  });
}

/**
 * @brief Deterministic fixture topology builder for Scenario D.
 *
 * NOTE: Used EXCLUSIVELY during fixture generation. NEVER invoked by the runtime
 * or Three.js renderer. All active leaves emitted are deterministic, non-overlapping,
 * and uniquely identified.
 */
export function buildActiveLeaves(regions: AdaptiveCell[]): AdaptiveCell[] {
  const origin = -0.025;
  const leaves: AdaptiveCell[] = [];
  const roots = new Set<string>();

  for (const c of regions) {
    for (
      let ix = Math.floor((c.x - c.sizeMeters / 2 - origin + 1e-8) / 0.4);
      ix <= Math.floor((c.x + c.sizeMeters / 2 - origin - 1e-8) / 0.4);
      ix++
    ) {
      for (
        let iy = Math.floor((c.y - c.sizeMeters / 2 - origin + 1e-8) / 0.4);
        iy <= Math.floor((c.y + c.sizeMeters / 2 - origin - 1e-8) / 0.4);
        iy++
      ) {
        roots.add(`${ix},${iy}`);
      }
    }
  }

  function visit(x: number, y: number, level: ResolutionLevel, candidates: AdaptiveCell[]) {
    const size = 0.05 * 2 ** level;
    const touching = candidates.filter(
      (c) =>
        Math.abs(c.x - (x + size / 2)) < (c.sizeMeters + size) / 2 - 1e-8 &&
        Math.abs(c.y - (y + size / 2)) < (c.sizeMeters + size) / 2 - 1e-8
    );

    if (level > 0 && touching.some((c) => c.level < level)) {
      for (const dx of [0, size / 2]) {
        for (const dy of [0, size / 2]) {
          visit(x + dx, y + dy, (level - 1) as ResolutionLevel, touching);
        }
      }
      return;
    }

    const cx = Number((x + size / 2).toFixed(3));
    const cy = Number((y + size / 2).toFixed(3));
    const exact = touching.find(
      (c) => c.level === level && Math.abs(c.x - cx) < 1e-8 && Math.abs(c.y - cy) < 1e-8
    );
    const source = exact ?? touching.sort((a, b) => a.level - b.level)[0];
    const canonicalId =
      exact?.cellId ??
      `D_L${level}_${cx < 0 ? 'N' : 'P'}${Math.abs(Math.round(cx * 1000))}_${cy < 0 ? 'N' : 'P'}${Math.abs(Math.round(cy * 1000))}`;

    leaves.push({
      cellId: canonicalId,
      x: cx,
      y: cy,
      level,
      sizeMeters: size,
      pointCount: 0,
      resolutionReasons: source?.resolutionReasons ?? ['DISTANCE_BASELINE'],
    });
  }

  for (const key of roots) {
    const [ix, iy] = key.split(',').map(Number);
    visit(origin + ix * 0.4, origin + iy * 0.4, 3, regions);
  }

  return leaves;
}
