import { useState } from 'react';
import { Link } from 'react-router-dom';
import { NestedSquareLogo } from '../components/common/NestedSquareLogo';
import { SCENARIOS } from '../demo/scenarios';
import { compareTopology } from '../utils/topology';
import type { ScenarioId } from '../types/perception';
import s from './Workbench.module.css';
import styles from './BenchmarksPage.module.css';

interface MetricDef {
  name: string;
  unit: string;
  targetOrMethod: string;
  description: string;
}

const BENCHMARK_METRICS: MetricDef[] = [
  {
    name: 'Active Cells',
    unit: 'COUNT',
    targetOrMethod: 'Active leaf count across spatial tree',
    description: 'Number of active spatial leaves currently stored and updated in the map.',
  },
  {
    name: 'Logical Map Bytes',
    unit: 'BYTES',
    targetOrMethod: '32 B / active leaf + tile pool index',
    description: 'Analytically modelled descriptor storage footprint in memory.',
  },
  {
    name: 'Process RSS',
    unit: 'MEGABYTES',
    targetOrMethod: 'OS resident set size instrumentation',
    description: 'Physical memory allocated to mapping process during sustained replay.',
  },
  {
    name: 'Mean Update Time',
    unit: 'MILLISECONDS',
    targetOrMethod: 'Monotonic clock per-frame average',
    description: 'Mean runtime elapsed between frame ingestion and hysteresis commit.',
  },
  {
    name: 'p50 Update Time',
    unit: 'MILLISECONDS',
    targetOrMethod: '50th percentile frame latency',
    description: 'Median per-frame execution time under typical spatial density.',
  },
  {
    name: 'p95 Update Time',
    unit: 'MILLISECONDS',
    targetOrMethod: '95th percentile latency threshold',
    description: 'Upper-bound execution latency under burst obstacle observations.',
  },
  {
    name: 'Maximum Update Time',
    unit: 'MILLISECONDS',
    targetOrMethod: 'Worst-case execution time (WCET)',
    description: 'Maximum single-frame latency recorded over entire benchmark run.',
  },
  {
    name: 'Throughput',
    unit: 'HZ / KPTS/SEC',
    targetOrMethod: 'Sustained frame & point intake rate',
    description: 'Rate of continuous point-cloud ingestion without queue drop.',
  },
  {
    name: 'Small-Object Preservation',
    unit: 'PERCENTAGE',
    targetOrMethod: 'Recall on hazards < 0.30 m',
    description: 'Retention rate of critical obstacles compared to fine uniform baseline.',
  },
  {
    name: 'Elevation Error / RMSE',
    unit: 'METERS',
    targetOrMethod: 'Vertical surface RMSE vs 5 cm reference',
    description: 'Root-mean-square elevation error of coarsen-merged regions.',
  },
];

const SCENARIO_NAMES: Record<ScenarioId, string> = {
  A: 'Scenario A · Urban Corridor',
  B: 'Scenario B · Pedestrian Hazard',
  C: 'Scenario C · Dynamic Vehicle',
  D: 'Scenario D · Mixed Intersection',
};

export function BenchmarksPage() {
  const [selectedScenario, setSelectedScenario] = useState<ScenarioId>('D');

  const topologyRows = (['A', 'B', 'C', 'D'] as ScenarioId[]).map((id) => {
    const frame = SCENARIOS[id];
    const top = compareTopology(frame.cells);
    const reductionPercent = (
      ((top.uniformCount - top.adaptiveCount) / top.uniformCount) *
      100
    ).toFixed(1);
    const uniformLogicalKb = ((top.uniformCount * 32) / 1024).toFixed(1);
    const adaptiveLogicalKb = ((top.adaptiveCount * 32) / 1024).toFixed(1);

    return {
      id,
      name: SCENARIO_NAMES[id],
      occupiedAreaM2: top.area.toFixed(2),
      uniformCells: top.uniformCount,
      adaptiveCells: top.adaptiveCount,
      cellReductionPercent: reductionPercent,
      uniformLogicalKb,
      adaptiveLogicalKb,
    };
  });

  return (
    <div className={s.page}>
      <header className={s.header}>
        <div>
          <div className={s.eyebrow}>03 / EXPERIMENTAL BENCHMARK PROTOCOL</div>
          <h1>Benchmarks & Empirical Methodology</h1>
          <p>
            Deterministic topology models vs future hardware measurements. All
            empirical performance numbers require repeatable hardware execution.
          </p>
        </div>
        <div className={styles.tagGroup}>
          <span className={styles.warningBadge}>
            AWAITING MEASURED BACKEND RUN
          </span>
          <span className={s.tag}>DEMO TOPOLOGY AVAILABLE</span>
        </div>
      </header>

      <div className={s.body} tabIndex={0} aria-label="Benchmarks overview">
        <div className={styles.container}>
          {/* SECTION A: BENCHMARK METHODOLOGY */}
          <section className={styles.protocolCard}>
            <div className={styles.protocolHeader}>
              <span className={styles.protocolTitle}>
                Section A · Benchmark Protocol & Evaluation Methodology
              </span>
              <span className={styles.provenanceBadge}>
                REPEATABLE THREE-WAY PIPELINE PROTOCOL
              </span>
            </div>
            <p className={s.footnote} style={{ marginTop: 0 }}>
              The authoritative benchmark evaluates identical point-cloud sequences
              through three comparative mapping pipelines under identical hardware budgets:
            </p>

            <div className={styles.pipelineFlow}>
              <div className={styles.flowStep}>
                <div className={styles.flowStepNumber}>PIPELINE 01</div>
                <div className={styles.flowStepTitle}>Uniform 5 cm Reference</div>
                <div className={styles.flowStepDesc}>
                  Exhaustive uniform grid at finest 5 cm resolution across entire occupied
                  footprint. Represents theoretical maximum geometric fidelity and maximum
                  memory consumption.
                </div>
              </div>
              <div className={styles.flowStep}>
                <div className={styles.flowStepNumber}>PIPELINE 02</div>
                <div className={styles.flowStepTitle}>Distance-Only Dyadic</div>
                <div className={styles.flowStepDesc}>
                  Banded concentric resolution allocation (0–10 m L0, 10–30 m L1, 30–60 m L2,
                  &gt;60 m L3). Ignores semantics, motion hazards, and geometry complexity.
                </div>
              </div>
              <div className={styles.flowStep}>
                <div className={styles.flowStepNumber}>PIPELINE 03</div>
                <div className={styles.flowStepTitle}>Adaptive Variable-Resolution (AVR)</div>
                <div className={styles.flowStepDesc}>
                  Full persistent adaptive hierarchy with semantic safety steering, dynamic
                  motion isolation, and temporal hysteresis commit. Preserves fine detail on
                  hazards while coarsening open space.
                </div>
              </div>
            </div>

            <div className={styles.protocolTitle} style={{ marginTop: 24 }}>
              Required Benchmark Metrics (Evaluated on Same Input Sequence)
            </div>
            <div className={styles.metricsGrid}>
              {BENCHMARK_METRICS.map((m) => (
                <div key={m.name} className={styles.metricItem}>
                  <div className={styles.metricName}>{m.name}</div>
                  <div className={styles.metricUnit}>{m.unit}</div>
                  <div className={styles.metricDesc}>{m.description}</div>
                </div>
              ))}
            </div>
          </section>

          {/* SECTION B: DEMO / MODELLED TOPOLOGY ANALYSIS */}
          <section className={styles.protocolCard}>
            <div className={styles.protocolHeader}>
              <span className={styles.protocolTitle}>
                Section B · Demo / Modelled Topology Analysis
              </span>
              <span className={styles.warningBadge}>
                DEMO / MODELLED · NOT MEASURED ON HARDWARE
              </span>
            </div>
            <p className={s.footnote} style={{ marginTop: 0 }}>
              Calculated analytically from authored fixture cell geometries over identical occupied footprints.
              Modelled logical map bytes reflect descriptor structures (32 bytes per active leaf descriptor),
              NOT process RSS or physical hardware memory.
            </p>

            <table className={s.table}>
              <thead>
                <tr>
                  <th>Scenario</th>
                  <th>Occupied Area</th>
                  <th>Uniform 5 cm Cells</th>
                  <th>Adaptive Active Leaves</th>
                  <th>Active Cell Reduction</th>
                  <th>Modelled Logical Bytes</th>
                </tr>
              </thead>
              <tbody>
                {topologyRows.map((r) => (
                  <tr
                    key={r.id}
                    style={{
                      background:
                        r.id === selectedScenario
                          ? 'var(--surface-hover)'
                          : undefined,
                    }}
                  >
                    <td>
                      <button
                        className={s.button}
                        style={{
                          border: 'none',
                          background: 'none',
                          padding: 0,
                          color:
                            r.id === selectedScenario
                              ? 'var(--accent-cyan)'
                              : 'var(--text-primary)',
                          fontWeight: r.id === selectedScenario ? 600 : 400,
                          cursor: 'pointer',
                        }}
                        onClick={() => setSelectedScenario(r.id)}
                      >
                        {r.name}
                      </button>
                    </td>
                    <td>{r.occupiedAreaM2} m²</td>
                    <td>{r.uniformCells.toLocaleString()}</td>
                    <td>{r.adaptiveCells.toLocaleString()}</td>
                    <td style={{ color: 'var(--status-healthy)', fontWeight: 600 }}>
                      {r.cellReductionPercent}%
                    </td>
                    <td>
                      {r.adaptiveLogicalKb} kB (vs {r.uniformLogicalKb} kB)
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </section>

          {/* SECTION C: MEASURED RESULTS (AWAITING RUN) */}
          <section className={styles.protocolCard}>
            <div className={styles.protocolHeader}>
              <span className={styles.protocolTitle}>
                Section C · Measured Backend Results
              </span>
              <span className={styles.warningBadge}>
                AWAITING MEASURED BACKEND RUN
              </span>
            </div>

            <div className={styles.awaitingCard}>
              <NestedSquareLogo size={56} />
              <div className={styles.awaitingTitle}>
                Awaiting Hardware Benchmark Execution
              </div>
              <div className={styles.awaitingDesc}>
                Benchmark stream is not connected. No live hardware timing, Process RSS,
                or throughput runs have been executed yet. Synthetic or target curves have been
                strictly excluded to maintain data truth. Connect the C++ mapping core via
                benchmark telemetry adapter to record empirical measurements.
              </div>
              <div className={s.actions}>
                <Link className={s.button} to="/compare">
                  Inspect Compare Topology →
                </Link>
                <Link className={s.button} to="/diagnostics">
                  Inspect Diagnostics Pipeline →
                </Link>
              </div>
            </div>

            <table className={s.table} style={{ marginTop: 20 }}>
              <thead>
                <tr>
                  <th>Measurement Metric</th>
                  <th>Uniform 5 cm</th>
                  <th>Distance Baseline</th>
                  <th>AVR Mapping Core</th>
                  <th>Hardware Status</th>
                </tr>
              </thead>
              <tbody>
                {BENCHMARK_METRICS.map((m) => (
                  <tr key={m.name}>
                    <td>{m.name}</td>
                    <td>—</td>
                    <td>—</td>
                    <td>—</td>
                    <td style={{ color: 'var(--status-warning)' }}>
                      AWAITING RUN
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </section>
        </div>
      </div>
    </div>
  );
}
