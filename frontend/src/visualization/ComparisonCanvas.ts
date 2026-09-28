import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { compareTopology } from '../utils/topology';
import { resolutionColor } from '../utils/resolution';
import type { PerceptionFrame } from '../types/perception';
export type CompareMode = 'SPLIT' | 'WIPE' | 'DIFFERENCE';
/** One camera and one frame for both representations. Draws only on input/resize. */
export class ComparisonCanvas {
    private frame: PerceptionFrame = frameRuntime.getLatestFrame();
    private mode: CompareMode = 'SPLIT';
    private divider = .5;
    private scale = 20;
    private center = { x: 0, y: 14 };
    private width = 1;
    private height = 1;
    private observer: ResizeObserver;
    private unsubscribe: () => void;
    private drag: {
        x: number;
        y: number;
    } | null = null;
    constructor(private canvas: HTMLCanvasElement) {
        this.observer = new ResizeObserver(() => this.resize());
        this.observer.observe(canvas);
        this.unsubscribe = frameRuntime.subscribeFrame(frame => { this.frame = frame; this.reset(true); });
        canvas.addEventListener('pointerdown', this.down);
        canvas.addEventListener('pointermove', this.move);
        canvas.addEventListener('pointerup', this.up);
        canvas.addEventListener('pointercancel', this.up);
        canvas.addEventListener('wheel', this.wheel, { passive: false });
    }
    setMode(mode: CompareMode) { this.mode = mode; this.reset(true); }
    setDivider(divider: number) { this.divider = divider; this.draw(); }
    zoom(factor: number) { this.scale = Math.max(3, Math.min(400, this.scale * factor)); this.draw(); }
    focusHazard() { const cell = this.frame.cells.find(c => c.cellId === 'D_L0_P_127') ?? this.frame.cells[0]; if (cell) {
        this.center = { x: cell.x, y: cell.y };
        this.scale = Math.min(this.width / (this.mode === 'SPLIT' ? 2 : 1) / 4, this.height / 4);
        this.draw();
    } }
    reset(intersection = false) { const e = compareTopology(this.frame.cells).extent; this.center = { x: (e.minX + e.maxX) / 2, y: (e.minY + e.maxY) / 2 }; this.scale = Math.min((this.width / (this.mode === 'SPLIT' ? 2 : 1) - 100) / Math.max(1, e.maxX - e.minX), (this.height - 100) / Math.max(1, e.maxY - e.minY)); this.scale = Math.max(3, this.scale); if (intersection && this.mode !== 'SPLIT' && this.frame.scenarioId === 'D') {
        this.center = { x: 0, y: 13 };
        this.scale *= 2.2;
    } this.draw(); }
    private resize() { this.width = this.canvas.clientWidth; this.height = this.canvas.clientHeight; const dpr = Math.min(devicePixelRatio || 1, 1.5); this.canvas.width = this.width * dpr; this.canvas.height = this.height * dpr; this.reset(true); }
    private down = (e: PointerEvent) => { this.drag = { x: e.clientX, y: e.clientY }; this.canvas.setPointerCapture(e.pointerId); };
    private move = (e: PointerEvent) => { if (!this.drag)
        return; this.center.x -= (e.clientX - this.drag.x) / this.scale; this.center.y += (e.clientY - this.drag.y) / this.scale; this.drag = { x: e.clientX, y: e.clientY }; this.draw(); };
    private up = () => { this.drag = null; };
    private wheel = (e: WheelEvent) => { e.preventDefault(); this.zoom(Math.exp(-e.deltaY * .001)); };
    private draw() {
        const ctx = this.canvas.getContext('2d');
        if (!ctx)
            return;
        const dpr = this.canvas.width / this.width;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.fillStyle = '#070A0D';
        ctx.fillRect(0, 0, this.width, this.height);
        if (this.mode === 'SPLIT') {
            this.panel(ctx, 0, this.width / 2, false, 0, this.width / 2);
            this.panel(ctx, this.width / 2, this.width / 2, true, this.width / 2, this.width / 2);
            ctx.strokeStyle = '#30404C';
            ctx.beginPath();
            ctx.moveTo(this.width / 2, 0);
            ctx.lineTo(this.width / 2, this.height);
            ctx.stroke();
        }
        else if (this.mode === 'WIPE') {
            this.panel(ctx, 0, this.width, false, 0, this.width * this.divider);
            this.panel(ctx, 0, this.width, true, this.width * this.divider, this.width * (1 - this.divider));
        }
        else
            this.panel(ctx, 0, this.width, true, 0, this.width);
    }
    private panel(ctx: CanvasRenderingContext2D, left: number, width: number, adaptive: boolean, clipX: number, clipW: number) {
        ctx.save();
        ctx.beginPath();
        ctx.rect(clipX, 0, clipW, this.height);
        ctx.clip();
        const sx = (x: number) => left + width / 2 + (x - this.center.x) * this.scale;
        const sy = (y: number) => this.height / 2 - (y - this.center.y) * this.scale;
        // Meter grid is tied to scene coordinates, never a decorative screen ruler.
        ctx.lineWidth = 1;
        ctx.strokeStyle = '#151E27';
        ctx.beginPath();
        for (let x = Math.floor(this.center.x - width / this.scale); x < this.center.x + width / this.scale; x++) {
            ctx.moveTo(sx(x), 0);
            ctx.lineTo(sx(x), this.height);
        }
        for (let y = Math.floor(this.center.y - this.height / this.scale); y < this.center.y + this.height / this.scale; y++) {
            ctx.moveTo(left, sy(y));
            ctx.lineTo(left + width, sy(y));
        }
        ctx.stroke();
        for (const c of this.frame.cells) {
            const x = sx(c.x - c.sizeMeters / 2), y = sy(c.y + c.sizeMeters / 2), size = c.sizeMeters * this.scale;
            if (x + size < left || x > left + width || y + size < 0 || y > this.height)
                continue;
            const color = adaptive ? resolutionColor(c.level) : '#769299';
            ctx.fillStyle = color;
            ctx.globalAlpha = this.mode === 'DIFFERENCE' ? (c.level > 0 ? .3 : .04) : adaptive ? .2 : .06;
            ctx.fillRect(x, y, size, size);
            ctx.globalAlpha = 1;
            ctx.strokeStyle = this.mode === 'DIFFERENCE' && c.level === 0 ? '#30404C' : color;
            ctx.lineWidth = .65;
            if (adaptive)
                ctx.strokeRect(x, y, size, size);
            else {
                const n = 2 ** c.level;
                ctx.beginPath();
                for (let i = 0; i <= n; i++) {
                    const d = size * i / n;
                    ctx.moveTo(x + d, y);
                    ctx.lineTo(x + d, y + size);
                    ctx.moveTo(x, y + d);
                    ctx.lineTo(x + size, y + d);
                }
                ctx.stroke();
            }
        }
        for (const p of this.frame.points) {
            ctx.fillStyle = p.routing === 'DYNAMIC_EXCLUDED' ? '#FF7557' : '#E6EDF3';
            ctx.globalAlpha = .85;
            ctx.fillRect(sx(p.x) - 1, sy(p.y) - 1, 2, 2);
        }
        ctx.globalAlpha = 1;
        for (const cl of this.frame.dynamicClusters) {
            ctx.strokeStyle = '#FF7557';
            ctx.lineWidth = 1.5;
            ctx.strokeRect(sx(cl.aabbMin.x), sy(cl.aabbMax.y), (cl.aabbMax.x - cl.aabbMin.x) * this.scale, (cl.aabbMax.y - cl.aabbMin.y) * this.scale);
            ctx.fillStyle = '#FFAA96';
            ctx.font = '11px "IBM Plex Mono"';
            ctx.fillText(`C-${String(cl.clusterLocalId).padStart(2, '0')} / ${cl.dominantClass}`, sx(cl.aabbMin.x), sy(cl.aabbMax.y) - 8);
        }
        const ox = sx(this.frame.sensorOrigin.x), oy = sy(this.frame.sensorOrigin.y);
        ctx.strokeStyle = '#E6EDF3';
        ctx.beginPath();
        ctx.moveTo(ox - 6, oy);
        ctx.lineTo(ox + 6, oy);
        ctx.moveTo(ox, oy - 6);
        ctx.lineTo(ox, oy + 6);
        ctx.stroke();
        const meters = this.scale > 70 ? 1 : 5;
        ctx.strokeStyle = '#A5B1BA';
        ctx.beginPath();
        ctx.moveTo(left + 24, this.height - 52);
        ctx.lineTo(left + 24 + meters * this.scale, this.height - 52);
        ctx.stroke();
        ctx.fillStyle = '#A5B1BA';
        ctx.font = '11px "IBM Plex Mono"';
        ctx.fillText(`${meters} m`, left + 24, this.height - 60);
        ctx.restore();
    }
    dispose() { this.unsubscribe(); this.observer.disconnect(); this.canvas.removeEventListener('pointerdown', this.down); this.canvas.removeEventListener('pointermove', this.move); this.canvas.removeEventListener('pointerup', this.up); this.canvas.removeEventListener('pointercancel', this.up); this.canvas.removeEventListener('wheel', this.wheel); }
}
