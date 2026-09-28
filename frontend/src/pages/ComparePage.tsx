import { useEffect, useRef, useState } from 'react';
import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { useFrameMetadata } from '../runtime/useFrameMetadata';
import { useConsoleStore } from '../stores/useConsoleStore';
import { compareTopology } from '../utils/topology';
import { ComparisonCanvas, type CompareMode } from '../visualization/ComparisonCanvas';
import s from './Workbench.module.css';
import styles from './ComparePage.module.css';

export function ComparePage() {
  const meta = useFrameMetadata();
  const presentation = useConsoleStore((state) => state.isPresentationMode);
  const [mode, setMode] = useState<CompareMode>(presentation ? 'WIPE' : 'SPLIT');
  const [divider, setDivider] = useState(50);
  const canvas = useRef<HTMLCanvasElement>(null);
  const renderer = useRef<ComparisonCanvas>();
  const topology = compareTopology(frameRuntime.getLatestFrame().cells);

  useEffect(() => {
    if (presentation) setMode('WIPE');
  }, [presentation]);

  useEffect(() => {
    if (!canvas.current) return;
    const view = new ComparisonCanvas(canvas.current);
    renderer.current = view;
    return () => {
      view.dispose();
      renderer.current = undefined;
    };
  }, []);

  useEffect(() => {
    renderer.current?.setMode(mode);
  }, [mode]);

  useEffect(() => {
    renderer.current?.setDivider(divider / 100);
  }, [divider]);

  const cellReductionPercent = (
    ((topology.uniformCount - topology.adaptiveCount) / topology.uniformCount) *
    100
  ).toFixed(1);
  const uniformLogicalKb = ((topology.uniformCount * 32) / 1024).toFixed(1);
  const adaptiveLogicalKb = ((topology.adaptiveCount * 32) / 1024).toFixed(1);

  return (
    <div className={s.page}>
      <header className={s.header}>
        <div>
          <div className={s.eyebrow}>02 / SPATIAL REPRESENTATION</div>
          <h1>The same world. Detail allocated differently.</h1>
          <p>
            Uniform 5 cm and adaptive 5–40 cm · same frame, occupied footprint and
            synchronized top-down camera.
          </p>
        </div>
        <span className={s.tag}>DERIVED DEMO TOPOLOGY</span>
      </header>

      <div className={styles.toolbar}>
        <div className={s.actions} role="group" aria-label="Comparison mode">
          {(['SPLIT', 'WIPE', 'DIFFERENCE'] as const).map((m) => (
            <button
              className={s.button}
              aria-pressed={m === mode}
              key={m}
              onClick={() => setMode(m)}
            >
              {m}
            </button>
          ))}
        </div>
        <div className={s.actions}>
          <span className={styles.sync}>
            SAME FRAME #{meta.frameSeq} · CAMERA LINKED
          </span>
          <button
            className={s.button}
            onClick={() => renderer.current?.zoom(1.3)}
            aria-label="Zoom in"
          >
            +
          </button>
          <button
            className={s.button}
            onClick={() => renderer.current?.zoom(1 / 1.3)}
            aria-label="Zoom out"
          >
            −
          </button>
          <button className={s.button} onClick={() => renderer.current?.reset()}>
            Fit footprint
          </button>
          <button
            className={s.button}
            onClick={() => renderer.current?.focusHazard()}
          >
            Inspect hazard
          </button>
        </div>
      </div>

      <div className={styles.scene}>
        <canvas
          ref={canvas}
          aria-label="Synchronized uniform and adaptive topology. Drag to pan; scroll to zoom. Cell counts and dimensions are available in the table below."
        />
        <div className={styles.leftLabel}>
          {mode === 'DIFFERENCE' ? 'REPRESENTATION DIFFERENCE' : 'UNIFORM / 5 cm'}
        </div>
        {mode !== 'DIFFERENCE' && (
          <div className={styles.rightLabel}>ADAPTIVE / 5–40 cm</div>
        )}
        {mode === 'WIPE' && (
          <div
            className={styles.wipeLine}
            style={{ left: `${divider}%` }}
            role="slider"
            tabIndex={0}
            aria-label="Drag comparison divider"
            aria-valuemin={0}
            aria-valuemax={100}
            aria-valuenow={divider}
            onPointerDown={(e) => e.currentTarget.setPointerCapture(e.pointerId)}
            onPointerMove={(e) => {
              if (!e.currentTarget.hasPointerCapture(e.pointerId)) return;
              const r = e.currentTarget.parentElement!.getBoundingClientRect();
              setDivider(
                Math.max(
                  0,
                  Math.min(100, Math.round(((e.clientX - r.left) / r.width) * 100))
                )
              );
            }}
            onKeyDown={(e) => {
              if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') {
                e.preventDefault();
                setDivider((n) =>
                  Math.max(0, Math.min(100, n + (e.key === 'ArrowRight' ? 1 : -1)))
                );
              }
            }}
          >
            <span>↔</span>
          </div>
        )}
        <div className={styles.sceneNote}>
          SCENE FRAME · XY / meters ·{' '}
          {mode === 'DIFFERENCE'
            ? 'Representation difference: adaptive cell allocation vs 5 cm baseline; hazard-region refinement preserved'
            : 'Every fine reference cell covers the same occupied adaptive footprint'}
        </div>
      </div>

      {mode === 'WIPE' && (
        <label className={styles.slider}>
          UNIFORM{' '}
          <input
            aria-label="Comparison wipe position"
            type="range"
            min="0"
            max="100"
            value={divider}
            onChange={(e) => setDivider(Number(e.target.value))}
          />{' '}
          ADAPTIVE
        </label>
      )}

      <div className={styles.metricRail}>
        <table className={s.table}>
          <thead>
            <tr>
              <th>DEMO TOPOLOGY / SAME FOOTPRINT</th>
              <th>Uniform 5 cm</th>
              <th>Adaptive</th>
              <th>Provenance</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>Active cells</td>
              <td>{topology.uniformCount.toLocaleString()}</td>
              <td>{topology.adaptiveCount.toLocaleString()}</td>
              <td>
                Σ 4^level reference cells · {topology.area.toFixed(2)} m² occupied
              </td>
            </tr>
            <tr>
              <td>Active cell reduction</td>
              <td
                colSpan={2}
                style={{ color: 'var(--status-healthy)', fontWeight: 600 }}
              >
                {cellReductionPercent}% cell-state reduction
              </td>
              <td>DERIVED DEMO TOPOLOGY (from active cell count)</td>
            </tr>
            <tr>
              <td>Modelled logical map bytes</td>
              <td>{uniformLogicalKb} kB</td>
              <td>{adaptiveLogicalKb} kB</td>
              <td>
                MODELLED LOGICAL MAP BYTES (32 B/cell descriptor · not process RSS)
              </td>
            </tr>
            <tr>
              <td>Hardware process memory / RSS</td>
              <td>—</td>
              <td>—</td>
              <td style={{ color: 'var(--status-warning)' }}>
                Awaiting measured backend run
              </td>
            </tr>
            <tr>
              <td>Per-frame update latency</td>
              <td>—</td>
              <td>—</td>
              <td style={{ color: 'var(--status-warning)' }}>
                Awaiting measured backend run
              </td>
            </tr>
            <tr>
              <td>Small object preservation</td>
              <td>—</td>
              <td>—</td>
              <td style={{ color: 'var(--status-warning)' }}>
                Awaiting benchmark evaluation
              </td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>
  );
}
