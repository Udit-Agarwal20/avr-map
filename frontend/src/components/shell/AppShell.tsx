import React, { useEffect } from 'react';
import { Outlet, useLocation, useSearchParams, useNavigate } from 'react-router-dom';
import { useConsoleStore } from '../../stores/useConsoleStore';
import { CommandPalette } from '../navigation/CommandPalette';
import styles from './AppShell.module.css';
import { NavRail } from './NavRail';
import { TelemetryStrip } from './TelemetryStrip';
import { TopSystemBar } from './TopSystemBar';

export const AppShell: React.FC = () => {
  const [searchParams] = useSearchParams();
  const location = useLocation();
  const navigate = useNavigate();
  const {
    setPresentationMode,
    togglePresentationMode,
    setScenario,
    isPresentationMode,
  } = useConsoleStore();

  const isMission = location.pathname === '/mission' || location.pathname === '/';

  // Check URL query ?demo=ppt or ?mode=presentation
  useEffect(() => {
    const demoParam = searchParams.get('demo');
    const modeParam = searchParams.get('mode');
    setPresentationMode(demoParam === 'ppt' || modeParam === 'presentation');
    if (demoParam === 'ppt' || modeParam === 'presentation') {
      setPresentationMode(true);
      setScenario('D'); // Ensure Hero scenario D is active for PPT
    }
  }, [searchParams, setPresentationMode, setScenario]);

  // Global key listener for 'P' key to toggle presentation mode
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      const state = useConsoleStore.getState();
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'k') { e.preventDefault(); state.setCommandOpen(!state.commandOpen); return; }
      if (state.commandOpen || document.querySelector('[role=dialog]')) return;
      if (
        e.target instanceof HTMLInputElement ||
        e.target instanceof HTMLTextAreaElement ||
        e.target instanceof HTMLSelectElement
      ) {
        return;
      }

      if (e.ctrlKey || e.metaKey || e.altKey) return;
      if (e.key === 'Escape') { state.clearSelection(); if (state.isLayerDrawerOpen) state.toggleLayerDrawer(); }
      if (e.key.toLowerCase() === 'c') navigate('/compare');
      if (e.key.toLowerCase() === 'r') state.resetCamera();
      if (e.key.toLowerCase() === 'l') state.toggleLayerDrawer();
      if (e.key === '1') state.setCameraMode('PERSPECTIVE');
      if (e.key === '2') state.setCameraMode('TOP_DOWN');
      if (e.key === 'p' || e.key === 'P') {
        e.preventDefault();
        togglePresentationMode();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [togglePresentationMode, navigate]);

  // Mission Hero Page has dedicated scene-first HUD (38px minimal top bar & mode dock)
  if (isMission) {
    return (
      <div
        className={`${styles.shell} ${styles.heroMissionShell}`}
        id="avr-console-root"
        data-presentation={isPresentationMode ? 'true' : 'false'}
      >
        <CommandPalette />
        <a className="skip-link" href="#main-content-area">Skip to workspace</a>
        <div className={styles.missionBody}>
          {/* In presentation mode, NavRail is completely hidden. In normal mode, compact edge rail */}
          {!isPresentationMode && (
            <div className={styles.compactNavEdge}>
              <NavRail />
            </div>
          )}
          <main className={styles.missionContent} id="main-content-area">
            <Outlet />
          </main>
        </div>
      </div>
    );
  }

  // Other pages (Compare, Architecture, Diagnostics, Config) retain standard F1 layout
  return (
    <div
      className={styles.shell}
      id="avr-console-root"
      data-presentation={isPresentationMode ? 'true' : 'false'}
    >
      <CommandPalette />
      <a className="skip-link" href="#main-content-area">Skip to workspace</a>
      <TopSystemBar />
      <div className={styles.mainBody}>
        {!isPresentationMode && <NavRail />}
        <main className={styles.contentRegion} id="main-content-area">
          <Outlet />
        </main>
      </div>
      <TelemetryStrip />
    </div>
  );
};
