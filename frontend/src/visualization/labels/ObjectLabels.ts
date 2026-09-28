import * as THREE from 'three';
import { DynamicObjectCluster } from '../../types/perception';

export interface LabelData {
  cluster: DynamicObjectCluster;
  screenX: number;
  screenY: number;
  visible: boolean;
}

/**
 * @file ObjectLabels.ts
 * @brief High-performance projected 3D-to-2D instrumentation labels anchored to cluster AABBs.
 */
export class ObjectLabels {
  private container: HTMLDivElement;
  private labelElements: Map<number, HTMLButtonElement> = new Map();
  private clusters: DynamicObjectCluster[] = [];
  private onSelectCluster?: (clusterId: number) => void;
  private isVisible = true;

  constructor(container: HTMLDivElement, onSelectCluster?: (clusterId: number) => void) {
    this.container = container;
    this.onSelectCluster = onSelectCluster;
  }

  public setVisible(visible: boolean): void {
    this.isVisible = visible;
    this.container.style.display = visible ? 'block' : 'none';
  }

  public updateClusters(clusters: DynamicObjectCluster[]): void {
    this.clusters = clusters;

    // Remove obsolete label elements
    const activeIds = new Set(clusters.map((c) => c.clusterLocalId));
    for (const [id, el] of this.labelElements.entries()) {
      if (!activeIds.has(id)) {
        if (el.parentNode) el.parentNode.removeChild(el);
        this.labelElements.delete(id);
      }
    }

    // Create or update elements
    for (const cluster of clusters) {
      let el = this.labelElements.get(cluster.clusterLocalId);
      if (!el) {
        el = document.createElement('button');
        el.className = 'avr-object-label';
        el.style.position = 'absolute';
        el.style.transform = 'translate(-50%, -100%)';
        el.style.pointerEvents = 'auto';
        el.style.cursor = 'pointer';
        el.style.background = 'rgba(10, 14, 20, 0.90)';

        el.style.border = '1px solid rgba(255, 117, 87, 0.7)';
        el.style.borderRadius = '4px';
        el.style.padding = '6px 10px';
        el.style.fontFamily = "var(--font-sans), 'IBM Plex Sans', -apple-system, sans-serif";
        el.style.fontSize = '11px';
        el.style.color = '#e2e8f0';

        el.style.userSelect = 'none';
        el.style.zIndex = '10';
        el.style.whiteSpace = 'nowrap';
        el.style.lineHeight = '1.35';

        el.addEventListener('click', (e) => {
          e.stopPropagation();
          if (this.onSelectCluster) {
            this.onSelectCluster(cluster.clusterLocalId);
          }
        });

        this.container.appendChild(el);
        this.labelElements.set(cluster.clusterLocalId, el);
      }

      const classColor = '#FF7557';
      el.innerHTML = `
        <div style="display:flex; justify-content:space-between; align-items:center; gap:12px; margin-bottom:3px;">
          <span style="font-family:var(--font-sans); font-weight:600; font-size:13px; color:${classColor}; letter-spacing:0.04em; display:flex; align-items:center; gap:5px;">
            <span style="display:inline-block; width:7px; height:7px; border-radius:1px; background:${classColor};"></span>
            ${cluster.dominantClass}
          </span>
          <span style="font-family:var(--font-condensed); color:#94a3b8; font-size:11px; font-weight:600; letter-spacing:0.03em;">
            FRAME C-0${cluster.clusterLocalId} · ${cluster.pointCount} PTS
          </span>
        </div>
        <div style="font-family:var(--font-mono); color:#cbd5e1; font-size:10px; font-weight:500; display:flex; gap:12px; letter-spacing:0.02em;">
          <span>SEM ${cluster.meanSemanticConfidence.toFixed(2)}</span>
          <span>MOTION ${cluster.meanMotionConfidence.toFixed(2)}</span>
        </div>
      `;
    }
  }

  public updatePositions(
    camera: THREE.Camera,
    viewportWidth: number,
    viewportHeight: number
  ): void {
    if (!this.isVisible) return;
    const tempVec = new THREE.Vector3();

    for (const cluster of this.clusters) {
      const el = this.labelElements.get(cluster.clusterLocalId);
      if (!el) continue;

      // Anchor to top-center of cluster AABB
      const centerX = (cluster.aabbMin.x + cluster.aabbMax.x) / 2;
      const topY = cluster.aabbMax.z + 0.15;
      const centerZ = -((cluster.aabbMin.y + cluster.aabbMax.y) / 2);

      tempVec.set(centerX, topY, centerZ);
      tempVec.project(camera);

      // Check if behind camera or outside clip space
      if (tempVec.z > 1.0) {
        el.style.display = 'none';
        continue;
      }

      const x = ((tempVec.x + 1) / 2) * viewportWidth;
      const y = ((-tempVec.y + 1) / 2) * viewportHeight;

      // Check viewport margins
      if (x < 130 || x > viewportWidth - 130 || y < 130 || y > viewportHeight - 80) {
        el.style.display = 'none';
        continue;
      }

      el.style.display = 'block';
      el.style.left = `${Math.round(x)}px`;
      el.style.top = `${Math.round(y)}px`;
    }
  }

  public dispose(): void {
    for (const el of this.labelElements.values()) {
      if (el.parentNode) el.parentNode.removeChild(el);
    }
    this.labelElements.clear();
  }
}
