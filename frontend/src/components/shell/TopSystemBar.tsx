import React from 'react';
import { useFrameMetadata } from '../../runtime/useFrameMetadata';
import { useConsoleStore } from '../../stores/useConsoleStore';
import { ScenarioId } from '../../types/perception';
import { pipelineStatusSeverity } from '../../utils/status';
import { Badge } from '../common/Badge';
import { NestedSquareLogo } from '../common/NestedSquareLogo';
import { StatusIndicator } from '../common/StatusIndicator';
import styles from './TopSystemBar.module.css';

const SCENARIOS: { id: ScenarioId; label: string; name: string }[] = [
  { id: 'A', label: 'A', name: 'Corridor' },
  { id: 'B', label: 'B', name: 'Pedestrian Hazard' },
  { id: 'C', label: 'C', name: 'Dynamic Vehicle' },
  { id: 'D', label: 'D', name: 'Mixed Intersection' },
];

export const TopSystemBar: React.FC = () => {
  const {
    currentScenarioId,
    setScenario,
    isPresentationMode,
    togglePresentationMode,
  } = useConsoleStore();

  const meta = useFrameMetadata();

  return (
    <header className={styles.topBar}>
      {/* Brand & Subtitle */}
      <div className={styles.leftGroup}>
        <div className={styles.brand}>
          <NestedSquareLogo size={20} />
          <span className={styles.logoText}>AVR/MAP</span>
          <span className={styles.subtitle}>ADAPTIVE 2.5D PERCEPTION</span>
        </div>

        {/* Deterministic Scenario Switcher */}
        <div className={styles.scenarioSwitcher} role="group" aria-label="Select Scenario">
          {SCENARIOS.map((sc) => (
            <button
              key={sc.id}
              id={`scenario-btn-${sc.id}`}
              className={`${styles.scenarioBtn} ${
                currentScenarioId === sc.id ? styles.scenarioBtnActive : ''
              }`}
              onClick={() => setScenario(sc.id)}
              title={`${sc.label}: ${sc.name}`}
            >
              {sc.label}
            </button>
          ))}
        </div>
      </div>

      {/* Center Scenario Title & Status */}
      <div className={styles.centerGroup}>
        <Badge variant="cyan">{meta.scenarioTitle}</Badge>
        {/* Strictly SIMULATED DEMO for deterministic fixtures */}
        <Badge variant="amber">SIMULATED DEMO</Badge>
      </div>

      {/* Right Telemetry & Controls */}
      <div className={styles.rightGroup}>
        {/* Explicitly labeled demo playback rate - never false backend latency */}
        <span className={styles.playbackTag}>DEMO PLAYBACK: 10 Hz</span>

        <span className="font-mono" style={{ fontSize: '11px', color: 'var(--text-secondary)' }}>
          FRAME #{meta.frameSeq.toString().padStart(5, '0')}
        </span>

        <StatusIndicator
          label={meta.pipelineStatus}
          status={pipelineStatusSeverity(meta.pipelineStatus)}
        />

        <button
          id="presentation-mode-btn"
          className={`${styles.pptBtn} ${isPresentationMode ? styles.pptBtnActive : ''}`}
          onClick={togglePresentationMode}
          title="Toggle PPT Presentation Mode (Key: P)"
        >
          <span>PPT MODE</span>
          <kbd style={{ fontSize: '9px', opacity: 0.8 }}>[P]</kbd>
        </button>
      </div>
    </header>
  );
};
