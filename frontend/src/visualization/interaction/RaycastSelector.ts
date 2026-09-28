import * as THREE from 'three';
import { AdaptiveCellField } from '../layers/AdaptiveCellField';
import { ClusterVolumeLayer } from '../layers/ClusterVolumeLayer';

export interface SelectionCallbacks {
  onSelectCluster: (clusterId: number) => void;
  onSelectCell: (cellId: string) => void;
  onClearSelection: () => void;
}

/**
 * @file RaycastSelector.ts
 * @brief Raycasting interaction handler for dynamic clusters and adaptive cells.
 */
export class RaycastSelector {
  private camera: THREE.Camera;
  private domElement: HTMLElement;
  private clusterLayer: ClusterVolumeLayer;
  private cellField: AdaptiveCellField;
  private callbacks: SelectionCallbacks;
  private raycaster: THREE.Raycaster;
  private pointer: THREE.Vector2;
  private pointerDownPos = { x: 0, y: 0 };

  constructor(
    camera: THREE.Camera,
    domElement: HTMLElement,
    clusterLayer: ClusterVolumeLayer,
    cellField: AdaptiveCellField,
    callbacks: SelectionCallbacks
  ) {
    this.camera = camera;
    this.domElement = domElement;
    this.clusterLayer = clusterLayer;
    this.cellField = cellField;
    this.callbacks = callbacks;
    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();

    this.onPointerDown = this.onPointerDown.bind(this);
    this.onPointerUp = this.onPointerUp.bind(this);

    this.domElement.addEventListener('pointerdown', this.onPointerDown);
    this.domElement.addEventListener('pointerup', this.onPointerUp);
  }

  private onPointerDown(e: PointerEvent): void {
    this.pointerDownPos = { x: e.clientX, y: e.clientY };
  }

  private onPointerUp(e: PointerEvent): void {
    // Ignore drags
    const dist = Math.hypot(e.clientX - this.pointerDownPos.x, e.clientY - this.pointerDownPos.y);
    if (dist > 5) return;

    const rect = this.domElement.getBoundingClientRect();
    this.pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

    this.raycaster.setFromCamera(this.pointer, this.camera);

    // 1. Check clusters first (higher interaction priority)
    const clusterTargets = this.clusterLayer.group.visible ? this.clusterLayer.getRaycastTargets() : [];
    const clusterHits = this.raycaster.intersectObjects(clusterTargets, false);
    if (clusterHits.length > 0) {
      const hit = clusterHits[0];
      const clusterId = hit.object.userData?.clusterId;
      if (typeof clusterId === 'number') {
        this.callbacks.onSelectCluster(clusterId);
        return;
      }
    }

    // 2. Check adaptive cells
    const cellMesh = this.cellField.getMeshes();
    if (cellMesh) {
      const cellHits = this.raycaster.intersectObjects(cellMesh, false);
      if (cellHits.length > 0 && typeof cellHits[0].instanceId === 'number') {
        const cell = this.cellField.getCellAtInstance(cellHits[0].object, cellHits[0].instanceId);
        if (cell) {
          this.callbacks.onSelectCell(cell.cellId);
          return;
        }
      }
    }

    // 3. Clicked empty space -> clear selection
    this.callbacks.onClearSelection();
  }

  public dispose(): void {
    this.domElement.removeEventListener('pointerdown', this.onPointerDown);
    this.domElement.removeEventListener('pointerup', this.onPointerUp);
  }
}
