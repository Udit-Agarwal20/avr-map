import * as THREE from 'three';
import { AdaptiveCell, ResolutionLevel } from '../../types/perception';
import { PALETTE } from '../materials/PerceptionMaterials';

/**
 * @file AdaptiveCellField.ts
 * @brief High-performance InstancedMesh renderer for the AVR dyadic cell hierarchy.
 *
 * Implements:
 * - Physical sizing: L0=0.05m, L1=0.10m, L2=0.20m, L3=0.40m
 * - Exact dyadic color palette:
 *   L0 (5 cm)  = #62D3C6
 *   L1 (10 cm) = #70B8E8
 *   L2 (20 cm) = #D6B45F
 *   L3 (40 cm) = #A18AC8
 * - Thin shared-edge boundaries forming a coherent spatial partition (no artificial tile gaps)
 * - Level-modulated opacity in PERCEPTION mode: L3 faint, L2 subtle, L1 moderate, L0 strong
 * - Prominent full-hierarchy display in RESOLUTION mode
 */
export class AdaptiveCellField {
  public group: THREE.Group;
  private meshes: THREE.InstancedMesh[] = [];
  private cellMaterial: THREE.MeshBasicMaterial;
  private cellTexture: THREE.CanvasTexture;
  private cellGeometry: THREE.PlaneGeometry;
  private activeCells: AdaptiveCell[] = [];
  private currentMode: 'OVERVIEW' | 'SEMANTIC' | 'RESOLUTION' | 'DYNAMICS' | 'RAW_LIDAR' = 'OVERVIEW';

  private selectedCellOutline: THREE.LineSegments | null = null;
  private selectedCellFill: THREE.Mesh | null = null;

  constructor() {
    this.group = new THREE.Group();
    this.group.name = 'adaptive-cell-field';

    // Generate procedural cell texture: crisp hairline border + subtle translucent interior
    this.cellTexture = this.createCellTexture();
    this.cellGeometry = new THREE.PlaneGeometry(1, 1);
    this.cellGeometry.rotateX(-Math.PI / 2);

    this.cellMaterial = new THREE.MeshBasicMaterial({
      map: this.cellTexture,
      transparent: true,
      opacity: 0.40,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
  }

  private createCellTexture(): THREE.CanvasTexture {
    const size = 256;
    const canvas = document.createElement('canvas');
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext('2d')!;

    // 1. Subtle interior translucent wash (12% opacity)
    ctx.fillStyle = 'rgba(255, 255, 255, 0.12)';
    ctx.fillRect(0, 0, size, size);

    // 2. Crisp perimeter hairline border (width 2.5px)
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = '#ffffff';
    ctx.strokeRect(1.5, 1.5, size - 3, size - 3);

    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.minFilter = THREE.LinearMipmapLinearFilter;
    texture.magFilter = THREE.LinearFilter;
    return texture;
  }

  public updateCells(cells: AdaptiveCell[]): void {
    this.activeCells = cells;
    for (const mesh of this.meshes) { this.group.remove(mesh); mesh.dispose(); }
    this.meshes = [];
    for (const level of [0, 1, 2, 3] as ResolutionLevel[]) {
      const group = cells.filter(c => c.level === level);
      if (!group.length) continue;
      const mesh = new THREE.InstancedMesh(this.cellGeometry, this.cellMaterial, group.length);
      mesh.name = `active-leaves-L${level}`;
      mesh.userData.cells = group;
      this.meshes.push(mesh); this.group.add(mesh);
    }
    this.refreshInstances();
  }

  private refreshInstances(): void {
    const dummy = new THREE.Object3D();
    const color = new THREE.Color();
    for (const mesh of this.meshes) {
      const cells = mesh.userData.cells as AdaptiveCell[];
      cells.forEach((cell, i) => {
        dummy.scale.set(cell.sizeMeters, 1, cell.sizeMeters);
        dummy.position.set(cell.x, .028, -cell.y);
        dummy.updateMatrix(); mesh.setMatrixAt(i, dummy.matrix);
        color.setHex(this.getLevelColor(cell.level));
        if (this.currentMode === 'SEMANTIC') color.set('#7e8b95');
        if (this.currentMode === 'OVERVIEW') color.multiplyScalar([1,.75,.5,.3][cell.level]);
        mesh.setColorAt(i, color);
      });
      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
      mesh.computeBoundingSphere();
    }
  }

  public getLevelColor(level: ResolutionLevel): number {
    switch (level) {
      case 0:
        return PALETTE.level0; // #62D3C6 5 cm
      case 1:
        return PALETTE.level1; // #70B8E8 10 cm
      case 2:
        return PALETTE.level2; // #D6B45F 20 cm
      case 3:
      default:
        return PALETTE.level3; // #A18AC8 40 cm
    }
  }

  public setMode(mode: 'OVERVIEW' | 'SEMANTIC' | 'RESOLUTION' | 'DYNAMICS' | 'RAW_LIDAR'): void {
    this.currentMode = mode;

    switch (mode) {
      case 'RESOLUTION':
        // Striking hero resolution display: full brightness and strong opacity
        this.cellMaterial.opacity = 0.85;
        break;
      case 'SEMANTIC':
        // Balanced semantic mode
        this.cellMaterial.opacity = 0.10;
        break;
      case 'DYNAMICS':
        // De-emphasized to let dynamic objects dominate
        this.cellMaterial.opacity = 0.15;
        break;
      case 'RAW_LIDAR':
        // Bare minimum context in raw point cloud mode
        this.cellMaterial.opacity = 0.04;
        break;
      case 'OVERVIEW':
      default:
        // Balanced overview mode: road remains clear road
        this.cellMaterial.opacity = 0.40;
        break;
    }

    this.refreshInstances();
  }

  public setSelectedCell(cellId: string | null): void {
    if (!cellId) {
      if (this.selectedCellOutline) this.selectedCellOutline.visible = false;
      if (this.selectedCellFill) this.selectedCellFill.visible = false;
      return;
    }

    const cell = this.activeCells.find((c) => c.cellId === cellId);
    if (!cell) {
      if (this.selectedCellOutline) this.selectedCellOutline.visible = false;
      if (this.selectedCellFill) this.selectedCellFill.visible = false;
      return;
    }

    const s = cell.sizeMeters;
    const yOffset = 0.022 + (3 - cell.level) * 0.0015 + 0.003;

    if (!this.selectedCellOutline) {
      const boxGeom = new THREE.BoxGeometry(1, 0.02, 1);
      const edgeGeom = new THREE.EdgesGeometry(boxGeom);
      boxGeom.dispose();
      const lineMat = new THREE.LineBasicMaterial({
        color: 0xffffff,
        linewidth: 2,
        transparent: true,
        opacity: 0.95,
        depthTest: false,
      });
      this.selectedCellOutline = new THREE.LineSegments(edgeGeom, lineMat);
      this.selectedCellOutline.renderOrder = 999;
      this.group.add(this.selectedCellOutline);
    }

    if (!this.selectedCellFill) {
      const planeGeom = new THREE.PlaneGeometry(1, 1);
      planeGeom.rotateX(-Math.PI / 2);
      const fillMat = new THREE.MeshBasicMaterial({
        color: 0x62d3c6,
        transparent: true,
        opacity: 0.35,
        depthWrite: false,
        side: THREE.DoubleSide,
      });
      this.selectedCellFill = new THREE.Mesh(planeGeom, fillMat);
      this.selectedCellFill.renderOrder = 998;
      this.group.add(this.selectedCellFill);
    }

    this.selectedCellOutline.scale.set(s * 1.02, 1, s * 1.02);
    this.selectedCellOutline.position.set(cell.x, yOffset + 0.01, -cell.y);
    this.selectedCellOutline.visible = true;

    this.selectedCellFill.scale.set(s, 1, s);
    this.selectedCellFill.position.set(cell.x, yOffset + 0.002, -cell.y);
    this.selectedCellFill.visible = true;
  }

  public getCellAtInstance(mesh: THREE.Object3D, instanceId: number): AdaptiveCell | null {
    return (mesh.userData.cells as AdaptiveCell[] | undefined)?.[instanceId] ?? null;
  }
  public getMeshes(): THREE.InstancedMesh[] { return this.group.visible ? this.meshes : []; }

  public dispose(): void {
    if (this.selectedCellOutline) {
      this.selectedCellOutline.geometry.dispose();
      (this.selectedCellOutline.material as THREE.Material).dispose();
    }
    if (this.selectedCellFill) {
      this.selectedCellFill.geometry.dispose();
      (this.selectedCellFill.material as THREE.Material).dispose();
    }
    for (const mesh of this.meshes) mesh.dispose();
    this.cellGeometry.dispose();
    this.cellMaterial.dispose();
    this.cellTexture.dispose();
  }
}
