import React, { useEffect, useState } from 'react';
import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { useConsoleStore } from '../stores/useConsoleStore';
import { PerceptionFrame } from '../types/perception';
import { resolutionColor, resolutionLabel } from '../utils/resolution';
import { semanticClassColor, semanticClassLabel } from '../utils/semantic';
import { RightInspector } from '../components/shell/RightInspector';
import styles from './MissionPage.module.css';

export const MissionPage: React.FC = () => {
  const [frame, setFrame] = useState<PerceptionFrame>(() => frameRuntime.getLatestFrame());
  const activeViewMode = useConsoleStore((state) => state.activeViewMode);
  const setViewMode = useConsoleStore((state) => state.setViewMode);
  const toggleInspector = useConsoleStore((state) => state.toggleInspector);
  const isInspectorOpen = useConsoleStore((state) => state.isInspectorOpen);

  useEffect(() => {
    return frameRuntime.subscribeFrame((f) => setFrame(f));
  }, []);

  const [zoomLevel, setZoomLevel] = useState<number>(1);

  const { cells, dynamicPoints, dynamicClusters, sensorOrigin } = frame;

  // Coordinate mapping:
  // Lidar X (forward) -> SVG -Y
  // Lidar Y (left) -> SVG -X (or X)
  // Let SVG X = y, SVG Y = -x
  const toSvgX = (lidarY: number) => lidarY;
  const toSvgY = (lidarX: number) => -lidarX;

  const viewBox = `${-22 / zoomLevel} ${-34 / zoomLevel} ${44 / zoomLevel} ${46 / zoomLevel}`;

  return (
    <div className={styles.pageContainer}>
      <div className={styles.viewportContainer}>
        {/* Viewport Toolbar */}
        <div className={styles.toolbar}>
          <div className={styles.modeTabs}>
            <button
              type="button"
              className={`${styles.modeTab} ${activeViewMode === 'RESOLUTION' ? styles.modeTabActive : ''}`}
              onClick={() => setViewMode('RESOLUTION')}
            >
              RESOLUTION
            </button>
            <button
              type="button"
              className={`${styles.modeTab} ${activeViewMode === 'SEMANTIC' ? styles.modeTabActive : ''}`}
              onClick={() => setViewMode('SEMANTIC')}
            >
              SEMANTICS
            </button>
            <button
              type="button"
              className={`${styles.modeTab} ${activeViewMode === 'DYNAMICS' ? styles.modeTabActive : ''}`}
              onClick={() => setViewMode('DYNAMICS')}
            >
              DYNAMICS
            </button>
          </div>

          <div className={styles.viewportActions}>
            <button
              type="button"
              className={styles.actionBtn}
              onClick={() => setZoomLevel((z) => Math.min(2.0, z + 0.25))}
              title="Zoom In"
            >
              + ZOOM
            </button>
            <button
              type="button"
              className={styles.actionBtn}
              onClick={() => setZoomLevel((z) => Math.max(0.5, z - 0.25))}
              title="Zoom Out"
            >
              - ZOOM
            </button>
            <button
              type="button"
              className={styles.actionBtn}
              onClick={() => setZoomLevel(1)}
              title="Reset View"
            >
              RESET
            </button>
            <button
              type="button"
              className={styles.actionBtn}
              onClick={toggleInspector}
              title="Toggle Inspector"
            >
              {isInspectorOpen ? 'HIDE INSPECTOR' : 'SHOW INSPECTOR'}
            </button>
          </div>
        </div>

        {/* Central Scene Area */}
        <div className={styles.sceneArea}>
          <svg
            className={styles.svgViewport}
            viewBox={viewBox}
            preserveAspectRatio="xMidYMid meet"
          >
            <defs>
              <pattern id="gridPattern" width="2" height="2" patternUnits="userSpaceOnUse">
                <path d="M 2 0 L 0 0 0 2" fill="none" stroke="rgba(255,255,255,0.04)" strokeWidth="0.05" />
              </pattern>
            </defs>

            {/* Background Grid */}
            <rect x="-50" y="-50" width="100" height="100" fill="url(#gridPattern)" />

            {/* Range Rings (5m, 10m, 15m, 20m, 25m) */}
            {[5, 10, 15, 20, 25].map((radius) => (
              <g key={`ring-${radius}`}>
                <circle
                  cx="0"
                  cy="0"
                  r={radius}
                  fill="none"
                  stroke="rgba(112, 184, 232, 0.12)"
                  strokeWidth="0.06"
                  strokeDasharray="0.3 0.3"
                />
                <text
                  x={radius + 0.3}
                  y="-0.3"
                  fill="rgba(112, 184, 232, 0.4)"
                  fontSize="0.8"
                  fontFamily="monospace"
                >
                  {radius}m
                </text>
              </g>
            ))}

            {/* Axes */}
            <line x1="-30" y1="0" x2="30" y2="0" stroke="rgba(255,255,255,0.08)" strokeWidth="0.08" />
            <line x1="0" y1="-40" x2="0" y2="15" stroke="rgba(255,255,255,0.08)" strokeWidth="0.08" />

            {/* Cells */}
            {cells.map((cell) => {
              const svgX = toSvgX(cell.y) - cell.sizeMeters / 2;
              const svgY = toSvgY(cell.x) - cell.sizeMeters / 2;

              let fillColor = 'rgba(74, 85, 104, 0.3)';
              let strokeColor = 'rgba(255,255,255,0.05)';
              let strokeWidth = 0.02;

              if (activeViewMode === 'RESOLUTION') {
                fillColor = resolutionColor(cell.level);
                strokeColor = 'rgba(0,0,0,0.3)';
                strokeWidth = cell.level === 0 ? 0.015 : 0.025;
              } else if (activeViewMode === 'SEMANTIC') {
                // If cell has elevation or semantic, fallback to class color
                fillColor = 'rgba(74, 85, 104, 0.45)';
                strokeColor = 'rgba(255,255,255,0.08)';
              } else if (activeViewMode === 'DYNAMICS') {
                fillColor = 'rgba(21, 30, 39, 0.6)';
                strokeColor = 'rgba(255,255,255,0.04)';
              }

              return (
                <rect
                  key={cell.cellId}
                  x={svgX}
                  y={svgY}
                  width={cell.sizeMeters}
                  height={cell.sizeMeters}
                  fill={fillColor}
                  fillOpacity={activeViewMode === 'RESOLUTION' ? 0.75 : 0.4}
                  stroke={strokeColor}
                  strokeWidth={strokeWidth}
                />
              );
            })}

            {/* Dynamic Excluded Points (Coral dots) */}
            {dynamicPoints.map((pt) => (
              <circle
                key={`dp-${pt.sourceIndex}`}
                cx={toSvgX(pt.position.y)}
                cy={toSvgY(pt.position.x)}
                r={0.12}
                fill="var(--dynamic-coral, #FF7557)"
                stroke="#000"
                strokeWidth={0.02}
              />
            ))}

            {/* Dynamic Clusters (AABB Bounding Boxes) */}
            {dynamicClusters.map((cluster) => {
              const minX = toSvgX(cluster.aabbMin.y);
              const maxX = toSvgX(cluster.aabbMax.y);
              const minY = toSvgY(cluster.aabbMax.x);
              const maxY = toSvgY(cluster.aabbMin.x);

              const left = Math.min(minX, maxX);
              const top = Math.min(minY, maxY);
              const width = Math.abs(maxX - minX);
              const height = Math.abs(maxY - minY);

              return (
                <g key={`cluster-${cluster.clusterLocalId}`}>
                  <rect
                    x={left}
                    y={top}
                    width={width}
                    height={height}
                    fill="rgba(255, 117, 87, 0.12)"
                    stroke="var(--dynamic-coral, #FF7557)"
                    strokeWidth="0.12"
                    strokeDasharray="0.3 0.2"
                  />
                  <rect
                    x={left}
                    y={top - 1.0}
                    width={width < 4 ? 4.2 : width}
                    height={0.9}
                    fill="rgba(11, 16, 21, 0.9)"
                    stroke="var(--dynamic-coral, #FF7557)"
                    strokeWidth="0.04"
                  />
                  <text
                    x={left + 0.2}
                    y={top - 0.35}
                    fill="var(--dynamic-coral, #FF7557)"
                    fontSize="0.55"
                    fontFamily="monospace"
                    fontWeight="bold"
                  >
                    [C-0{cluster.clusterLocalId}] {cluster.dominantClass}
                  </text>
                </g>
              );
            })}

            {/* Sensor / Ego Vehicle Glyph at Origin (0,0) */}
            <g transform={`translate(${toSvgX(sensorOrigin.y)}, ${toSvgY(sensorOrigin.x)})`}>
              {/* Ego Vehicle Body (1.8m width, 4.2m length) */}
              <rect
                x="-0.9"
                y="-3.1"
                width="1.8"
                height="4.2"
                rx="0.3"
                fill="rgba(30, 42, 54, 0.9)"
                stroke="var(--accent-cyan)"
                strokeWidth="0.1"
              />
              {/* Heading arrow */}
              <polygon
                points="0,-3.4 -0.5,-2.7 0.5,-2.7"
                fill="var(--accent-cyan)"
              />
              {/* Sensor origin center dot */}
              <circle cx="0" cy="0" r="0.2" fill="var(--accent-cyan)" />
              <circle cx="0" cy="0" r="0.5" fill="none" stroke="var(--accent-cyan)" strokeWidth="0.04" />
              <text
                x="1.3"
                y="0.3"
                fill="var(--accent-cyan)"
                fontSize="0.65"
                fontFamily="monospace"
              >
                EGO / SENSOR ORIGIN
              </text>
            </g>
          </svg>

          {/* Camera Info HUD */}
          <div className={styles.hudCamera}>
            VIEW: BEV 2.5D (TOP-DOWN) | PREVIEW ENGINE: SVG DTO | CANVAS F2 MOUNT
          </div>

          {/* HUD Legend */}
          <div className={styles.hudLegend}>
            <div style={{ color: 'var(--text-primary)', fontWeight: 600, marginBottom: 2 }}>
              {activeViewMode === 'RESOLUTION' && 'ADAPTIVE RESOLUTION'}
              {activeViewMode === 'SEMANTIC' && 'SEMANTIC TAXONOMY'}
              {activeViewMode === 'DYNAMICS' && 'DYNAMIC ROUTING'}
            </div>

            {activeViewMode === 'RESOLUTION' && (
              <>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: resolutionColor(0) }} />
                  <span>{resolutionLabel(0)} (Critical Hazard / Detailed Feature)</span>
                </div>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: resolutionColor(1) }} />
                  <span>{resolutionLabel(1)} (Near Obstacle / Feature Boundary)</span>
                </div>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: resolutionColor(2) }} />
                  <span>{resolutionLabel(2)} (Standard Navigable Surface)</span>
                </div>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: resolutionColor(3) }} />
                  <span>{resolutionLabel(3)} (Coarse Baseline / Open Far Field)</span>
                </div>
              </>
            )}

            {activeViewMode === 'SEMANTIC' && (
              <>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: semanticClassColor('ROAD') }} />
                  <span>{semanticClassLabel('ROAD')}</span>
                </div>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: semanticClassColor('VEHICLE') }} />
                  <span>{semanticClassLabel('VEHICLE')}</span>
                </div>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: semanticClassColor('PEDESTRIAN') }} />
                  <span>{semanticClassLabel('PEDESTRIAN')}</span>
                </div>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: semanticClassColor('BUILDING') }} />
                  <span>{semanticClassLabel('BUILDING')}</span>
                </div>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: semanticClassColor('VEGETATION') }} />
                  <span>{semanticClassLabel('VEGETATION')}</span>
                </div>
              </>
            )}

            {activeViewMode === 'DYNAMICS' && (
              <>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: 'var(--dynamic-coral, #FF7557)' }} />
                  <span>DYNAMIC_EXCLUDED / Frame-Local Cluster</span>
                </div>
                <div className={styles.legendRow}>
                  <span className={styles.legendColor} style={{ backgroundColor: 'rgba(74, 85, 104, 0.5)' }} />
                  <span>STATIC_FUSABLE (Static Map Evidence)</span>
                </div>
              </>
            )}
          </div>
        </div>
      </div>

      {/* Right-hand Inspector */}
      <RightInspector />
    </div>
  );
};
