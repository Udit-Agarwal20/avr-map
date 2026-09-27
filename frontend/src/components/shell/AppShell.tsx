import React, { useEffect } from 'react';
import { Outlet, useSearchParams } from 'react-router-dom';
import { useConsoleStore } from '../../stores/useConsoleStore';
import styles from './AppShell.module.css';
import { NavRail } from './NavRail';
import { TelemetryStrip } from './TelemetryStrip';
import { TopSystemBar } from './TopSystemBar';

export const AppShell: React.FC = () => {
  const [searchParams] = useSearchParams();
  const { setPresentationMode, togglePresentationMode, setScenario } = useConsoleStore();

  // Check URL query ?demo=ppt or ?mode=presentation
  useEffect(() => {
    const demoParam = searchParams.get('demo');
    const modeParam = searchParams.get('mode');
    if (demoParam === 'ppt' || modeParam === 'presentation') {
      setPresentationMode(true);
      setScenario('D'); // Ensure Hero scenario D is active for PPT
    }
  }, [searchParams, setPresentationMode, setScenario]);

  // Global key listener for 'P' key to toggle presentation mode
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Avoid triggering when user is in input/textarea
      if (
        e.target instanceof HTMLInputElement ||
        e.target instanceof HTMLTextAreaElement ||
        e.target instanceof HTMLSelectElement
      ) {
        return;
      }

      if (e.key === 'p' || e.key === 'P') {
        e.preventDefault();
        togglePresentationMode();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [togglePresentationMode]);

  return (
    <div className={styles.shell} id="avr-console-root">
      <TopSystemBar />
      <div className={styles.mainBody}>
        <NavRail />
        <main className={styles.contentRegion} id="main-content-area">
          <Outlet />
        </main>
      </div>
      <TelemetryStrip />
    </div>
  );
};
