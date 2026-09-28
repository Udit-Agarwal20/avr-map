import { useFrameMetadata } from '../runtime/useFrameMetadata';
import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { validatePerceptionFrame } from '../demo/fixtureValidation';
import { pipelineStatusLabel, pipelineStatusSeverity } from '../utils/status';
import s from './Workbench.module.css';
import styles from './DiagnosticsPage.module.css';

export function DiagnosticsPage() {
  const meta = useFrameMetadata();
  const frame = frameRuntime.getLatestFrame();
  const validation = validatePerceptionFrame(frame);
  const severity = pipelineStatusSeverity(meta.pipelineStatus);

  const nodes = [
    ['01', 'INPUT PREFLIGHT', 'FIXTURE SOURCE', `${meta.pointCount} current-frame returns`],
    ['02', 'SEMANTIC EXTRACTION', 'CURRENT FRAME', '8 canonical classes · ephemeral'],
    ['03', 'MOTION FILTERING', 'FRAME LOCAL', `${meta.dynamicPointCount} returns · ${meta.clusterCount} clusters`],
    ['04', 'RESOLUTION DECISION', 'FROZEN POLICY', `${meta.cellCount.toLocaleString()} regions touched`],
    ['05', 'ADAPTIVE MAP FUSION', meta.pipelineStatus, `${meta.cellCount.toLocaleString()} active leaves`],
    ['06', 'TELEMETRY GATEWAY', 'NOT CONNECTED', 'Local prototype runtime'],
  ];

  const staticCount = frame.points.filter((p) => p.routing === 'STATIC_FUSABLE').length;
  const clusteredCount = frame.dynamicClusters.reduce(
    (n, c) => n + c.memberSourceIndices.length,
    0
  );

  return (
    <div className={s.page}>
      <header className={s.header}>
        <div>
          <div className={s.eyebrow}>04 / PIPELINE OBSERVABILITY</div>
          <h1>System Diagnostics & Pipeline Health</h1>
          <p>
            Evidence tracking from input frame to persistent mapping contracts. Operational timing
            and hardware queues are not exposed in fixture mode.
          </p>
        </div>
        <div className={styles.tagGroup}>
          <span className={s.tag}>DEMO PIPELINE STATE</span>
          <span className={styles.warningTag}>SIMULATED DIAGNOSTICS</span>
        </div>
      </header>

      <div className={s.body} tabIndex={0} aria-label="Diagnostics details">
        <div className={styles.pipeline} aria-label="Perception pipeline stages">
          {nodes.map(([n, name, status, detail]) => (
            <div key={n} className={styles.node}>
              <span className={styles.number}>{n}</span>
              <h2>{name}</h2>
              <span className={styles.state}>{status}</span>
              <p>{detail}</p>
              <div className={styles.notExposed}>
                Latency — · NOT EXPOSED
                <br />
                Queue — · NOT EXPOSED
                <br />
                Last commit —
              </div>
            </div>
          ))}
        </div>

        <div className={s.notice} role="status">
          <strong>
            {severity.toUpperCase()} / {pipelineStatusLabel(meta.pipelineStatus)} (Frozen C++ Status #{frame.frameSeq ? 0 : 0})
          </strong>
          {severity === 'healthy'
            ? 'The mapping core reports successful frame fusion and valid hysteresis state. Operational latency benchmarks and hardware queue telemetry require a connected C++ daemon.'
            : severity === 'rejected'
            ? 'This frame was rejected during validation. Observations were not fused into the spatial tree. Inspect input data and provider contract before re-submitting.'
            : severity === 'terminal'
            ? 'The pipeline is in a latched fatal error state. Preserving current spatial topology; restart required.'
            : 'Pipeline reported a degraded outcome. Review operational diagnostics.'}
        </div>

        <div className={styles.columns}>
          <section>
            <h2 className={s.sectionTitle}>POINT ROUTING & CONSERVATION ACCOUNTING</h2>
            <table className={s.table}>
              <tbody>
                {[
                  ['Current-frame returns', `${meta.pointCount} points`],
                  ['STATIC_FUSABLE points', `${staticCount} points (authorized for terrain fusion)`],
                  ['DYNAMIC_EXCLUDED points', `${meta.dynamicPointCount} points (excluded from terrain fusion)`],
                  ['Dynamic cluster members', `${clusteredCount} points in ${meta.clusterCount} candidates`],
                  ['Noise returns isolated', `${meta.noiseCount} points`],
                  ['Fixture conservation check', validation.valid ? 'PASS (conserved)' : 'FAIL (violation)'],
                ].map(([k, v]) => (
                  <tr key={k}>
                    <td>{k}</td>
                    <td>{v}</td>
                  </tr>
                ))}
              </tbody>
            </table>
            {!validation.valid && (
              <p role="alert" style={{ color: 'var(--status-critical)', marginTop: 8 }}>
                {validation.errors.join('; ')}
              </p>
            )}
          </section>

          <section>
            <h2 className={s.sectionTitle}>SOURCE & CAPABILITY BOUNDARY (DATA TRUTH)</h2>
            <table className={s.table}>
              <tbody>
                <tr>
                  <td>Perception Frame Provider</td>
                  <td>FixturePerceptionSource (Deterministic Fixtures)</td>
                </tr>
                <tr>
                  <td>C++ Backend Transport</td>
                  <td style={{ color: 'var(--status-warning)' }}>NOT CONNECTED (FIXTURE MODE)</td>
                </tr>
                <tr>
                  <td>Operational Hardware Telemetry</td>
                  <td style={{ color: 'var(--status-warning)' }}>NOT EXPOSED IN FIXTURE MODE</td>
                </tr>
                <tr>
                  <td>Coordinate Frame</td>
                  <td>SCENE FRAME (origin at ego vehicle)</td>
                </tr>
                <tr>
                  <td>Semantic Persistence Contract</td>
                  <td>CURRENT FRAME ONLY (ephemeral evidence)</td>
                </tr>
                <tr>
                  <td>Temporal Object Tracking</td>
                  <td style={{ color: 'var(--text-muted)' }}>NOT AVAILABLE IN PROTOTYPE</td>
                </tr>
                <tr>
                  <td>Hardware Benchmark Telemetry</td>
                  <td style={{ color: 'var(--status-warning)' }}>AWAITING BENCHMARK RUN</td>
                </tr>
              </tbody>
            </table>
          </section>
        </div>

        <p className={s.footnote}>
          Dynamic observations may refine the decision region and form frame-local clusters. Their
          elevation returns are strictly excluded from persistent static terrain fusion. Operational
          queue depth, cache eviction rates, and microsecond-level stage timings require live C++
          instrumentation.
        </p>
      </div>
    </div>
  );
}
