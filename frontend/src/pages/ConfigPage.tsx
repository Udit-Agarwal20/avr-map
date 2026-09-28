import { useRef, useState } from 'react';
import * as Dialog from '@radix-ui/react-dialog';
import { useConsoleStore, type LayerVisibility } from '../stores/useConsoleStore';
import s from './Workbench.module.css';
import styles from './ConfigPage.module.css';

const categories = [
  'Map',
  'Resolution',
  'Semantics',
  'Tracking',
  'Vehicle',
  'Traversability',
  'Visualization',
];

export const LAYER_LABELS: Record<keyof LayerVisibility, string> = {
  sceneContext: 'Scene context',
  lidarPoints: 'LiDAR points',
  adaptiveCells: 'Adaptive cells',
  dynamicPoints: 'Dynamic points',
  clusterBounds: 'Cluster bounds',
  objectLabels: 'Object labels',
};

export function ConfigPage() {
  const reviewButton = useRef<HTMLButtonElement>(null);
  const reviewBar = useRef<HTMLDivElement>(null);
  const layers = useConsoleStore((state) => state.layers);
  const setLayer = useConsoleStore((state) => state.setLayerVisible);
  const [category, setCategory] = useState('Visualization');
  const [draft, setDraft] = useState({ ...layers });
  const [review, setReview] = useState(false);
  const [applied, setApplied] = useState(false);

  const changes = (Object.keys(layers) as (keyof LayerVisibility)[]).filter(
    (k) => draft[k] !== layers[k]
  );

  const rows: Record<string, string[][]> = {
    Map: [
      ['Map payload', 'Persistent geometric elevation'],
      ['Coordinate frame', 'SCENE FRAME (fixture origin at ego vehicle)'],
      ['Runtime mutation', 'No C++ backend adapter connected (read-only)'],
    ],
    Resolution: [
      ['Dyadic resolution tiers', 'L0: 0.05 m · L1: 0.10 m · L2: 0.20 m · L3: 0.40 m'],
      ['Planar XY distance bands', '[0, 10.0) · [10.0, 30.0) · [30.0, 60.0) · [60.0, ∞) m'],
      ['Policy authority', 'Frozen backend defaults; demo topology is illustrative'],
    ],
    Semantics: [
      ['Canonical classes', 'UNKNOWN, GROUND, ROAD, VEGETATION, BUILDING, VEHICLE, PEDESTRIAN, OBSTACLE_OTHER'],
      ['Persistence model', 'Ephemeral / current-frame evidence only (zero map pollution)'],
      ['Cluster XY radius (default)', '0.50 m (deterministic frame-local clustering)'],
      ['Maximum vertical gap (default)', '0.80 m'],
      ['Minimum cluster points (default)', '5 points'],
      ['Minimum semantic confidence', '0.50'],
    ],
    Tracking: [
      ['Frame-local clustering', 'Available in current prototype (C-01, C-02)'],
      ['Temporal identity / velocity / trajectory', 'NOT AVAILABLE IN CURRENT PROTOTYPE'],
    ],
    Vehicle: [
      ['Physical model', 'Simulated scene context (ego vehicle at 0, 0, 0)'],
      ['Vehicle control / actuation', 'NOT AVAILABLE IN CURRENT PROTOTYPE'],
    ],
    Traversability: [
      ['Slope / roughness / clearance / overhang', 'NOT AVAILABLE IN CURRENT PROTOTYPE (Planned V2)'],
    ],
  };

  return (
    <div className={s.page}>
      <header className={s.header}>
        <div>
          <div className={s.eyebrow}>05 / CONFIGURATION & CONTRACT POLICY</div>
          <h1>Inspect the Contract · Local Demo Configuration</h1>
          <p>
            Backend runtime configuration is not connected. Mapping contracts and spatial policies
            are read-only engineering defaults. Layer changes apply locally to this browser session.
          </p>
        </div>
        <div className={styles.tagGroup}>
          <span className={s.tag}>LOCAL DEMO CONFIG</span>
          <span className={styles.tagMuted}>Backend runtime configuration is not connected</span>
        </div>
      </header>

      <div className={s.workspace}>
        <nav className={s.sidebar} aria-label="Configuration categories">
          <h2>PARAMETERS</h2>
          {categories.map((c) => (
            <button
              key={c}
              className={s.sideButton}
              aria-pressed={category === c}
              onClick={() => setCategory(c)}
            >
              {c}
              <small>{c === 'Visualization' ? 'EDITABLE · LOCAL SESSION' : 'READ ONLY CONTRACT'}</small>
            </button>
          ))}
        </nav>

        <section className={s.body}>
          <h2 className={s.sectionTitle}>{category.toUpperCase()}</h2>

          {category === 'Visualization' ? (
            <>
              <div className={s.notice}>
                <strong>Local Visualization Layers</strong>
                These controls change local rendering visibility only. They do not mutate mapping,
                resolution policy, or static terrain fusion in the C++ core.
              </div>

              <div className={styles.fields}>
                {(
                  Object.entries(LAYER_LABELS) as [keyof LayerVisibility, string][]
                ).map(([key, label]) => (
                  <label key={key} className={styles.field}>
                    <span>
                      {label}
                      <small>Mission renderer layer</small>
                    </span>
                    <input
                      type="checkbox"
                      checked={draft[key]}
                      onChange={(e) => {
                        setApplied(false);
                        setDraft({ ...draft, [key]: e.target.checked });
                      }}
                    />
                  </label>
                ))}
              </div>

              <div className={styles.reviewBar} ref={reviewBar} tabIndex={-1}>
                <span role="status">
                  {applied
                    ? 'Applied to local visualization.'
                    : `${changes.length} pending local changes`}
                </span>
                <div className={s.actions}>
                  <button
                    className={s.button}
                    disabled={!changes.length}
                    onClick={() => setDraft({ ...layers })}
                  >
                    Discard edits
                  </button>
                  <button
                    className={`${s.button} ${s.primary}`}
                    ref={reviewButton}
                    disabled={!changes.length}
                    onClick={() => setReview(true)}
                  >
                    Review changes
                  </button>
                </div>
              </div>
            </>
          ) : (
            <>
              <div className={s.notice}>
                <strong>Frozen Backend Contract · Read-Only</strong>
                Backend parameters cannot be mutated from this frontend prototype. Values reflect
                frozen C++ specifications and engineering defaults (HEAD: cc50675).
              </div>
              <table className={s.table}>
                <tbody>
                  {rows[category].map(([k, v]) => (
                    <tr key={k}>
                      <td>{k}</td>
                      <td>{v}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </>
          )}

          <Dialog.Root open={review} onOpenChange={setReview}>
            <Dialog.Portal>
              <Dialog.Overlay className={styles.overlay} />
              <Dialog.Content
                className={styles.dialog}
                onCloseAutoFocus={(e) => {
                  e.preventDefault();
                  (reviewButton.current?.disabled
                    ? reviewBar.current
                    : reviewButton.current
                  )?.focus();
                }}
              >
                <Dialog.Title>Review Local Visualization Changes</Dialog.Title>
                <Dialog.Description>
                  Apply listed layer toggles to this browser session. The backend mapping core is
                  unaffected and not connected.
                </Dialog.Description>

                <table className={s.table}>
                  <thead>
                    <tr>
                      <th>Layer</th>
                      <th>Current</th>
                      <th>After apply</th>
                    </tr>
                  </thead>
                  <tbody>
                    {changes.map((k) => (
                      <tr key={k}>
                        <td>{LAYER_LABELS[k]}</td>
                        <td>{layers[k] ? 'Visible' : 'Hidden'}</td>
                        <td>{draft[k] ? 'Visible' : 'Hidden'}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>

                <div className={s.actions}>
                  <Dialog.Close className={s.button}>Back to edit</Dialog.Close>
                  <button
                    className={`${s.button} ${s.primary}`}
                    onClick={() => {
                      changes.forEach((k) => setLayer(k, draft[k]));
                      setReview(false);
                      setApplied(true);
                    }}
                  >
                    Apply locally
                  </button>
                </div>
              </Dialog.Content>
            </Dialog.Portal>
          </Dialog.Root>
        </section>
      </div>
    </div>
  );
}
