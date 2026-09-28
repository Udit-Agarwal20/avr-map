import { NestedSquareLogo } from '../components/common/NestedSquareLogo';
import { ScenarioId } from '../types/perception';
import React, { useEffect, useRef, useState } from 'react';
import { useSearchParams } from 'react-router-dom';
import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { useFrameMetadata } from '../runtime/useFrameMetadata';
import { useConsoleStore, ViewMode } from '../stores/useConsoleStore';
import { PerceptionScene, SceneMode } from './PerceptionScene';
import styles from './PerceptionViewport.module.css';
import { SelectedObjectInspector } from './SelectedObjectInspector';

/**
 * @file PerceptionViewport.tsx
 * @brief High-performance React boundary for Plain Three.js perception renderer.
 *
 * Implements F2A-R2:
 * - Scientific precision instrument UI ("THE SCENE IS THE INTERFACE")
 * - Nested-square resolution motif in hierarchy legend
 * - "WHY THIS RESOLUTION?" explainability inspector
 * - Dynamic cluster inspector (FRAME CLUSTER C-XX)
 * - Metric scale ruler & Scene frame spatial instrumentation
 * - Discrete Mode Architecture (Visual Data Modes vs Camera Modes)
 * - Compact Layer Drawer with direct Three.js layer visibility controls
 * - Automated query presets for ?demo=ppt (&view=resolution, &view=dynamics, &view=raw)
 */
export const PerceptionViewport: React.FC = () => {
  const containerRef = useRef<HTMLDivElement>(null);
  const canvasContainerRef = useRef<HTMLDivElement>(null);
  const labelContainerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<PerceptionScene | null>(null);

  const [searchParams] = useSearchParams();
  const meta = useFrameMetadata();

  const {
    activeViewMode,
    cameraMode,
    layers,
    isLayerDrawerOpen,
    selectedCellId,
    setViewMode,
    setCameraMode,
    toggleLayerDrawer,
    toggleLayer,
    setSelectedClusterId,
    setSelectedCellId,
    clearSelection,
    setPresentationMode,
  } = useConsoleStore();

  const presentation = useConsoleStore(state => state.isPresentationMode);
  const cameraRevision = useConsoleStore(state => state.cameraRevision);
  const [narrow, setNarrow] = useState(window.innerWidth < 1100);
  const rulerRef = useRef<HTMLDivElement>(null);
  const [isLegendOpen, setIsLegendOpen] = useState(true);

  useEffect(() => {
    const media = matchMedia('(max-width:1099px)');
    const update = () => setNarrow(media.matches); media.addEventListener('change', update);
    return () => media.removeEventListener('change', update);
  }, []);
  useEffect(() => { sceneRef.current?.setPresentationLocked(presentation || narrow); }, [presentation,narrow]);
  useEffect(() => {
    const state = useConsoleStore.getState();
    sceneRef.current?.applyCameraPreset(state.activeViewMode === 'RESOLUTION' ? 'RESOLUTION' : state.activeViewMode === 'DYNAMICS' ? 'DYNAMICS' : state.cameraMode === 'TOP_DOWN' ? 'TOP_DOWN' : 'HERO');
  }, [cameraRevision]);

  // Initialize Three.js Perception Scene
  useEffect(() => {
    if (!canvasContainerRef.current || !labelContainerRef.current) return;

    const scene = new PerceptionScene({
      canvasContainer: canvasContainerRef.current,
      labelContainer: labelContainerRef.current,
      onSelectCluster: (clusterId) => {
        setSelectedClusterId(clusterId);
      },
      onSelectCell: (cellId) => {
        setSelectedCellId(cellId);
      },
    });
    sceneRef.current = scene;
    scene.setPresentationLocked(useConsoleStore.getState().isPresentationMode || window.innerWidth < 1100);
    let rulerFrame = 0;
    const updateRuler = () => {
      if (rulerRef.current) rulerRef.current.style.width = `${scene.metricReferencePixels()}px`;
      rulerFrame = requestAnimationFrame(updateRuler);
    };
    updateRuler();

    // Resize Observer
    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        if (width > 0 && height > 0) {
          scene.resize(width, height);
        }
      }
    });

    if (containerRef.current) {
      resizeObserver.observe(containerRef.current);
    }

    return () => {
      cancelAnimationFrame(rulerFrame);
      resizeObserver.disconnect();
      scene.dispose();
      sceneRef.current = null;
    };
  }, [setSelectedClusterId, setSelectedCellId]);

  // Handle URL Query Presets (?demo=ppt, &view=resolution, &view=dynamics, &view=raw)
  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;

    const demo = searchParams.get('demo');
    const view = searchParams.get('view');
    if (demo === 'ppt') useConsoleStore.getState().setScenario('D');
    const frame = frameRuntime.getLatestFrame();

    if (demo === 'ppt') {
      scene.setPresentationLocked(true);
      setPresentationMode(true);

      if (view === 'resolution') {
        setViewMode('RESOLUTION');
        scene.setMode('RESOLUTION');
        scene.applyCameraPreset('RESOLUTION');
        // Auto-select canonical pedestrian hazard cell (D_L0_P_127 at x=3.600, y=10.200)
        const hazardCell =
          frame.cells.find((c) => c.cellId === 'D_L0_P_127') ||
          frame.cells.find(
            (c) =>
              c.level === 0 &&
              Math.abs(c.x - 3.60) < 0.04 &&
              Math.abs(c.y - 10.20) < 0.04
          ) ||
          frame.cells.find(
            (c) => c.level === 0 && c.resolutionReasons.includes('SEMANTIC_RELEVANCE')
          );
        if (hazardCell) {
          setSelectedCellId(hazardCell.cellId);
          scene.selectCell(hazardCell.cellId);
        }
      } else if (view === 'dynamics') {
        setViewMode('DYNAMICS');
        scene.setMode('DYNAMICS');
        scene.applyCameraPreset('DYNAMICS');
        setSelectedClusterId(0);
        scene.selectCell(null);
      } else if (view === 'raw') {
        setViewMode('RAW_LIDAR');
        scene.setMode('RAW_LIDAR');
        scene.applyCameraPreset('HERO');
        clearSelection();
        scene.selectCell(null);
      } else if (view === 'semantic') {
        setViewMode('SEMANTIC');
        scene.setMode('SEMANTIC');
        scene.applyCameraPreset('HERO');
        clearSelection();
        scene.selectCell(null);
      } else {
        setViewMode('OVERVIEW');
        scene.setMode('OVERVIEW');
        scene.applyCameraPreset('HERO');
        clearSelection();
        scene.selectCell(null);
      }
    }
  }, [searchParams, setPresentationMode, setViewMode, setSelectedCellId, setSelectedClusterId, clearSelection]);

  // Synchronize ViewMode changes to Three.js scene
  useEffect(() => {
    if (sceneRef.current) {
      sceneRef.current.setMode(activeViewMode as SceneMode);
    }
  }, [activeViewMode]);

  // Synchronize CameraMode changes
  useEffect(() => {
    if (sceneRef.current) {
      sceneRef.current.setCameraMode(cameraMode);
    }
  }, [cameraMode]);

  // Synchronize selectedCellId to Three.js highlight outline
  useEffect(() => {
    if (sceneRef.current) {
      sceneRef.current.selectCell(selectedCellId);
    }
  }, [selectedCellId]);

  // Synchronize layer visibility to Three.js scene
  useEffect(() => {
    if (sceneRef.current) {
      sceneRef.current.setLayerVisibility('sceneContext', layers.sceneContext);
      sceneRef.current.setLayerVisibility('lidarPoints', layers.lidarPoints);
      sceneRef.current.setLayerVisibility('adaptiveCells', layers.adaptiveCells);
      sceneRef.current.setLayerVisibility('dynamicPoints', layers.dynamicPoints);
      sceneRef.current.setLayerVisibility('clusterBounds', layers.clusterBounds);
      sceneRef.current.setLayerVisibility('objectLabels', layers.objectLabels);
    }
  }, [layers]);

  const handleModeChange = (mode: ViewMode) => {
    setViewMode(mode);
    if (sceneRef.current) {
      sceneRef.current.setMode(mode as SceneMode);
    }
  };

  const handleRecenter = () => {
    if (sceneRef.current) {
      if (activeViewMode === 'RESOLUTION') {
        sceneRef.current.applyCameraPreset('RESOLUTION');
      } else if (activeViewMode === 'DYNAMICS') {
        sceneRef.current.applyCameraPreset('DYNAMICS');
      } else {
        sceneRef.current.applyCameraPreset(
          cameraMode === 'TOP_DOWN' ? 'TOP_DOWN' : 'HERO'
        );
      }
    }
  };

  return (
    <div
      ref={containerRef}
      className={styles.viewportWrapper}
      id="avr-perception-viewport"
      onClick={(e) => {
        // Clear selection if clicking the background canvas directly
        if (e.target === canvasContainerRef.current) {
          clearSelection();
        }
      }}
    >
      {/* Three.js Canvas Container */}
      <div ref={canvasContainerRef} className={styles.canvasContainer} />

      {/* Screen-Space Projected HTML Label Overlay */}
      <div ref={labelContainerRef} className={styles.labelContainer} />

      {/* Minimal Top Bar (40px) */}
      <header className={styles.topBar}>
        <div className={styles.brandGroup}>
          <div className={styles.wordmark}>
            <NestedSquareLogo size={20} />
            AVR/MAP
          </div>
          <div className={styles.scenarioTitle}>
            <select aria-label="Fixture scenario" value={meta.scenarioId} onChange={e=>useConsoleStore.getState().setScenario(e.target.value as ScenarioId)}>
              <option value="A">A / Urban Corridor</option><option value="B">B / Pedestrian Hazard</option><option value="C">C / Dynamic Vehicle</option><option value="D">D / Mixed Intersection</option>
            </select>
          </div>
        </div>

        <div className={styles.topRightGroup}>
          <button className={styles.utilBtn} aria-label="Open command palette" onClick={()=>useConsoleStore.getState().setCommandOpen(true)}>⌘ K</button>
          <button className={styles.utilBtn} aria-pressed={presentation} onClick={()=>useConsoleStore.getState().togglePresentationMode()}>PPT [P]</button>
          <div className={styles.demoBadge}>SIMULATED DEMO</div>
          <div className={styles.statusGroup}>
            <span className={styles.statusDot} />
            {meta.pipelineStatus}
          </div>
          <div className={styles.frameSeq}>
            FRAME {meta.frameSeq.toString().padStart(5, '0')}
          </div>
        </div>
      </header>

      <div className={styles.contextNote}>{narrow ? 'MONITORING MODE · ' : ''}{activeViewMode === 'SEMANTIC' ? 'CURRENT-FRAME SEMANTIC RETURNS' : activeViewMode === 'RAW_LIDAR' ? 'LIDAR INPUT · DEMO FIXTURE' : 'SIMULATED SCENE CONTEXT + AVR PERCEPTION OUTPUT'}</div>
      <div className={styles.inspectActions}>
        <button className={styles.utilBtn} onClick={()=>{useConsoleStore.getState().setScenario('D');setViewMode('RESOLUTION');setSelectedCellId('D_L0_P_127');useConsoleStore.getState().resetCamera();}}>Why 5 cm? ↗</button>
        {meta.dynamicClustersSummary.map(c=><button key={c.clusterLocalId} className={styles.utilBtn} onClick={()=>setSelectedClusterId(c.clusterLocalId)}>Inspect C-{String(c.clusterLocalId).padStart(2,'0')}</button>)}
      </div>
      {/* Upper-Right Control Utilities */}
      <div className={styles.topControlGroup} data-engineering-control>
        <button
          type="button"
          className={`${styles.utilBtn} ${isLayerDrawerOpen ? styles.utilBtnActive : ''}`}
          onClick={toggleLayerDrawer}
          aria-expanded={isLayerDrawerOpen}
          title="Toggle Map Layer Drawer"
        >
          LAYERS
        </button>
        <button
          type="button"
          className={styles.utilBtn}
          onClick={handleRecenter}
          title="Recenter Camera to Canonical Angle"
        >
          RECENTER
        </button>
      </div>

      {/* Layer Drawer Panel */}
      {isLayerDrawerOpen && (
        <div className={styles.layerDrawer} aria-label="Layer Visibility Controls">
          <div className={styles.layerDrawerHeader}>
            <span className={styles.layerDrawerTitle}>DISPLAY LAYERS</span>
            <button
              type="button"
              className={styles.drawerCloseBtn}
              aria-label="Close layer drawer"
              onClick={toggleLayerDrawer}
            >
              ✕
            </button>
          </div>
          <div className={styles.layerList}>
            <label className={styles.layerItem}>
              <input
                type="checkbox"
                checked={layers.sceneContext}
                onChange={() => toggleLayer('sceneContext')}
              />
              <span className={styles.layerLabel}>Scene Context</span>
              <span className={styles.layerMeta}>Static/Road</span>
            </label>
            <label className={styles.layerItem}>
              <input
                type="checkbox"
                checked={layers.lidarPoints}
                onChange={() => toggleLayer('lidarPoints')}
              />
              <span className={styles.layerLabel}>LiDAR Points</span>
              <span className={styles.layerMeta}>Sensor</span>
            </label>
            <label className={styles.layerItem}>
              <input
                type="checkbox"
                checked={layers.adaptiveCells}
                onChange={() => toggleLayer('adaptiveCells')}
              />
              <span className={styles.layerLabel}>Adaptive Cells</span>
              <span className={styles.layerMeta}>L0–L3 Field</span>
            </label>
            <label className={styles.layerItem}>
              <input
                type="checkbox"
                checked={layers.dynamicPoints}
                onChange={() => toggleLayer('dynamicPoints')}
              />
              <span className={styles.layerLabel}>Dynamic Points</span>
              <span className={styles.layerMeta}>Excluded</span>
            </label>
            <label className={styles.layerItem}>
              <input
                type="checkbox"
                checked={layers.clusterBounds}
                onChange={() => toggleLayer('clusterBounds')}
              />
              <span className={styles.layerLabel}>Cluster Bounds</span>
              <span className={styles.layerMeta}>AABB</span>
            </label>
            <label className={styles.layerItem}>
              <input
                type="checkbox"
                checked={layers.objectLabels}
                onChange={() => toggleLayer('objectLabels')}
              />
              <span className={styles.layerLabel}>Object Labels</span>
              <span className={styles.layerMeta}>HUD</span>
            </label>
          </div>
        </div>
      )}

      {/* Floating Bottom Mode Dock (40px) */}
      <nav className={styles.modeDock} aria-label="Perception View Modes">
        <button
          type="button"
          className={`${styles.modeBtn} ${activeViewMode === 'OVERVIEW' ? styles.modeBtnActive : ''}`}
          aria-pressed={activeViewMode === 'OVERVIEW'}
          onClick={() => handleModeChange('OVERVIEW')}
        >
          OVERVIEW
        </button>
        <button
          type="button"
          className={`${styles.modeBtn} ${activeViewMode === 'SEMANTIC' ? styles.modeBtnActive : ''}`}
          aria-pressed={activeViewMode === 'SEMANTIC'}
          onClick={() => handleModeChange('SEMANTIC')}
        >
          SEMANTIC
        </button>
        <button
          type="button"
          className={`${styles.modeBtn} ${activeViewMode === 'RESOLUTION' ? styles.modeBtnActive : ''}`}
          aria-pressed={activeViewMode === 'RESOLUTION'}
          onClick={() => handleModeChange('RESOLUTION')}
        >
          RESOLUTION
        </button>
        <button
          type="button"
          className={`${styles.modeBtn} ${activeViewMode === 'DYNAMICS' ? styles.modeBtnActive : ''}`}
          aria-pressed={activeViewMode === 'DYNAMICS'}
          onClick={() => handleModeChange('DYNAMICS')}
        >
          DYNAMICS
        </button>
        <button
          type="button"
          className={`${styles.modeBtn} ${activeViewMode === 'RAW_LIDAR' ? styles.modeBtnActive : ''}`}
          aria-pressed={activeViewMode === 'RAW_LIDAR'}
          onClick={() => handleModeChange('RAW_LIDAR')}
        >
          LIDAR INPUT
        </button>

        <div className={styles.dockDivider} />

        <button
          type="button"
          className={`${styles.modeBtn} ${cameraMode === 'PERSPECTIVE' ? styles.modeBtnActive : ''}`}
          onClick={() => {
            setCameraMode('PERSPECTIVE');
            if (sceneRef.current) sceneRef.current.setCameraMode('PERSPECTIVE');
          }}
          title="Perspective 3D Camera"
        >
          PERSPECTIVE
        </button>
        <button
          type="button"
          className={`${styles.modeBtn} ${cameraMode === 'TOP_DOWN' ? styles.modeBtnActive : ''}`}
          onClick={() => {
            setCameraMode('TOP_DOWN');
            if (sceneRef.current) sceneRef.current.setCameraMode('TOP_DOWN');
          }}
          title="Top-Down Camera"
        >
          TOP
        </button>
      </nav>

      {/* Metric Scale Ruler (Bottom Center, above mode dock) */}
      <div className={styles.metricScaleBar} aria-label="Physical Metric Scale">
        <div ref={rulerRef} className={styles.rulerTicks}>
          <div className={styles.rulerLine} />
          <div className={styles.rulerTickStart} />
          <div className={styles.rulerTickMid} />
          <div className={styles.rulerTickEnd} />
        </div>
        <div className={styles.rulerLabel}>2 m · GROUND AT Y=10 m</div>
      </div>

      {/* Lower-Left Telemetry & Spatial Instrumentation Ribbon */}
      <div className={styles.telemetryRibbon}>
        <div className={styles.telemetryItem}>
          <span className={styles.telemetryVal}>{meta.cellCount.toLocaleString()}</span>
          <span>ACTIVE CELLS</span>
        </div>
        <span className={styles.telemetrySep}>·</span>
        <div className={styles.telemetryItem}>
          <span>DEMO L0</span>
          <span className={styles.telemetryVal}>
            {meta.resolutionSummary.l0Percent.toFixed(0)}%
          </span>
        </div>
        <span className={styles.telemetrySep}>·</span>
        <div className={styles.telemetryItem}>
          <span className={styles.telemetryVal}>{meta.dynamicPointCount}</span>
          <span>DYNAMIC PTS</span>
        </div>
        <span className={styles.telemetrySep}>·</span>
        <div className={styles.telemetryItem}>
          <span className={styles.telemetryVal}>{meta.clusterCount}</span>
          <span>FRAME CLUSTERS</span>
        </div>
        <span className={styles.telemetrySep}>·</span>
        <div className={styles.telemetryItem}>
          <span className={styles.frameLabel}>SCENE FRAME</span>
          <span className={styles.frameVal}>{`[${meta.sensorOrigin.x.toFixed(2)}, ${meta.sensorOrigin.y.toFixed(2)}, ${meta.sensorOrigin.z.toFixed(2)}]`}</span>
        </div>
      </div>

      {/* Lower-Right Nested-Square Hierarchy Legend */}
      <div className={styles.legendContainer}>
        {isLegendOpen ? (
          <div className={styles.legendCard}>
            <button
              type="button"
              aria-label="Collapse resolution legend"
              style={{
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
                cursor: 'pointer',
                marginBottom: '6px',
              }}
              onClick={() => setIsLegendOpen(false)}
            >
              <span className={styles.legendTitle}>RESOLUTION HIERARCHY</span>
              <span style={{ color: '#94a3b8', fontSize: '11px', fontWeight: 600 }}>−</span>
            </button>

            {/* Nested-Square Motif & Physical Scale Relationship */}
            <div className={styles.nestedSquareContainer}>
              <svg
                width="52"
                height="52"
                viewBox="0 0 52 52"
                className={styles.nestedSvg}
                aria-label="Nested Quadtree Subdivision"
              >
                {/* L3 Outer Box: 40 cm */}
                <rect
                  x="2"
                  y="2"
                  width="48"
                  height="48"
                  fill="rgba(161, 138, 200, 0.12)"
                  stroke="#a18ac8"
                  strokeWidth="1.2"
                />
                {/* L2 Inner Box: 20 cm */}
                <rect
                  x="14"
                  y="14"
                  width="24"
                  height="24"
                  fill="rgba(214, 180, 95, 0.16)"
                  stroke="#d6b45f"
                  strokeWidth="1.2"
                />
                {/* L1 Inner Box: 10 cm */}
                <rect
                  x="20"
                  y="20"
                  width="12"
                  height="12"
                  fill="rgba(112, 184, 232, 0.22)"
                  stroke="#70b8e8"
                  strokeWidth="1.2"
                />
                {/* L0 Core Box: 5 cm */}
                <rect
                  x="23"
                  y="23"
                  width="6"
                  height="6"
                  fill="#62d3c6"
                  stroke="#62d3c6"
                  strokeWidth="1"
                />
              </svg>

              <div className={styles.nestedLabels}>
                <div className={styles.nestedLabelRow}>
                  <span className={styles.nestedDot} style={{ background: '#a18ac8' }} />
                  <span className={styles.nestedLevel}>L3</span>
                  <span className={styles.nestedDim}>40 cm</span>
                  <span className={styles.nestedRole}>Coarse</span>
                </div>
                <div className={styles.nestedLabelRow}>
                  <span className={styles.nestedDot} style={{ background: '#d6b45f' }} />
                  <span className={styles.nestedLevel}>L2</span>
                  <span className={styles.nestedDim}>20 cm</span>
                  <span className={styles.nestedRole}>Approach</span>
                </div>
                <div className={styles.nestedLabelRow}>
                  <span className={styles.nestedDot} style={{ background: '#70b8e8' }} />
                  <span className={styles.nestedLevel}>L1</span>
                  <span className={styles.nestedDim}>10 cm</span>
                  <span className={styles.nestedRole}>Transition</span>
                </div>
                <div className={styles.nestedLabelRow}>
                  <span className={styles.nestedDot} style={{ background: '#62d3c6' }} />
                  <span className={styles.nestedLevel} style={{ color: '#62d3c6' }}>L0</span>
                  <span className={styles.nestedDim} style={{ color: '#ffffff', fontWeight: 600 }}>5 cm</span>
                  <span className={styles.nestedRole} style={{ color: '#62d3c6', fontWeight: 600 }}>Hazard</span>
                </div>
              </div>
            </div>

            <div className={styles.legendDivider} />

            <div className={styles.legendRow}>
              <span
                className={styles.legendSwatch}
                style={{ backgroundColor: 'var(--dynamic-coral, #ff7557)' }}
              />
              <span style={{ color: '#ff7557', fontWeight: 600 }}>DYNAMIC EXCLUDED</span>
            </div>
          </div>
        ) : (
          <button
            type="button"
            className={styles.utilBtn}
            onClick={() => setIsLegendOpen(true)}
            style={{ position: 'static' }}
          >
            LEGEND
          </button>
        )}
      </div>

      {/* Slide-in Selected Object Inspector */}
      <SelectedObjectInspector />
    </div>
  );
};
