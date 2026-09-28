import type { AdaptiveCell, PerceptionPoint } from '../types/perception';
/** Half-open ownership, stabilized to micrometers to avoid binary boundary drift. */
export function containsPoint(cell: AdaptiveCell, point: Pick<PerceptionPoint, 'x' | 'y'>): boolean {
    const q = (n: number) => Math.round(n * 1e6);
    const half = cell.sizeMeters / 2;
    return q(point.x) >= q(cell.x - half) && q(point.x) < q(cell.x + half)
        && q(point.y) >= q(cell.y - half) && q(point.y) < q(cell.y + half);
}
/** Same occupied footprint on both sides; sparse unmapped space stays unmapped. */
export function compareTopology(cells: AdaptiveCell[]) {
    const extent = cells.reduce((e, c) => ({
        minX: Math.min(e.minX, c.x - c.sizeMeters / 2), maxX: Math.max(e.maxX, c.x + c.sizeMeters / 2),
        minY: Math.min(e.minY, c.y - c.sizeMeters / 2), maxY: Math.max(e.maxY, c.y + c.sizeMeters / 2),
    }), { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity });
    const uniformCount = cells.reduce((sum, c) => sum + 4 ** c.level, 0);
    return { extent: cells.length ? extent : { minX: 0, maxX: 1, minY: 0, maxY: 1 },
        adaptiveCount: cells.length, uniformCount, area: uniformCount * 0.05 ** 2,
        changedCount: cells.filter(c => c.level > 0).length };
}
/** Demo normalization only. Remove synthetic payload statistics and derive actual returns. */
export function withCurrentReturns(cells: AdaptiveCell[], points: PerceptionPoint[]): AdaptiveCell[] {
    return cells.map(cell => ({ cellId: cell.cellId, level: cell.level, x: cell.x, y: cell.y,
        sizeMeters: cell.sizeMeters, resolutionReasons: cell.resolutionReasons,
        pointCount: points.filter(point => containsPoint(cell, point)).length }));
}
