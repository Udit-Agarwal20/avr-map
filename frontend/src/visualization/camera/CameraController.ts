import * as THREE from 'three';

export type CameraViewMode = 'PERSPECTIVE' | 'TOP_DOWN';
export type CameraPreset = 'HERO' | 'RESOLUTION' | 'DYNAMICS' | 'TOP_DOWN';

/**
 * @file CameraController.ts
 * @brief Cinematic autonomous perception camera controller with orbit/pan/zoom and canonical presets.
 */
export class CameraController {
  public camera: THREE.PerspectiveCamera;
  private domElement: HTMLElement;
  private target: THREE.Vector3 = new THREE.Vector3(0, 1.0, -11.0);
  private currentMode: CameraViewMode = 'PERSPECTIVE';
  private isLocked = false;

  // Orbit state
  private isPointerDown = false;
  private pointerButton = -1;
  private previousPointerPosition = { x: 0, y: 0 };
  private spherical = new THREE.Spherical();

  constructor(camera: THREE.PerspectiveCamera, domElement: HTMLElement) {
    this.camera = camera;
    this.domElement = domElement;

    // Apply default canonical Hero perspective
    this.applyPreset('HERO');
    this.bindEvents();
  }

  public setLocked(locked: boolean): void {
    this.isLocked = locked;
  }

  public getCurrentMode(): CameraViewMode {
    return this.currentMode;
  }

  public setViewMode(mode: CameraViewMode): void {
    if (this.currentMode === mode) return;
    this.currentMode = mode;
    if (mode === 'TOP_DOWN') {
      this.applyPreset('TOP_DOWN');
    } else {
      this.applyPreset('HERO');
    }
  }

  public applyPreset(preset: CameraPreset): void {
    switch (preset) {
      case 'HERO':
        // Cinematic elevated rear-quarter perspective
        // Ego vehicle is in lower-center third; intersection, pedestrian, and secondary vehicle in view
        this.camera.position.set(3.2, 8.8, 14.0);
        this.target.set(0.2, 0.9, -11.5);
        this.currentMode = 'PERSPECTIVE';
        break;

      case 'RESOLUTION':
        // Canonical resolution explainability preset:
        // Positioned on roadway clear of sidewalk poles; reveals pedestrian hazard,
        // surrounding crosswalk & road, dyadic L3->L2->L1->L0 quadtree, and background vehicle without clipping
        this.camera.position.set(2.4, 3.8, -3.8);
        this.target.set(3.4, 0.7, -10.5);
        this.currentMode = 'PERSPECTIVE';
        break;

      case 'DYNAMICS':
        // Canonical dynamic hazard preset:
        // Preserves relationship to sensor origin while framing approaching vehicle in FRAME C-00 AABB
        this.camera.position.set(0.4, 3.8, -5.8);
        this.target.set(-2.8, 0.8, -15.0);
        this.currentMode = 'PERSPECTIVE';
        break;

      case 'TOP_DOWN':
        // High-altitude technical BEV inspection
        this.camera.position.set(0.0, 34.0, -12.0);
        this.target.set(0.0, 0.0, -12.0);
        this.currentMode = 'TOP_DOWN';
        break;
    }

    this.camera.lookAt(this.target);
    this.updateSphericalFromCamera();
  }

  private updateSphericalFromCamera(): void {
    const offset = new THREE.Vector3().subVectors(this.camera.position, this.target);
    this.spherical.setFromVector3(offset);
  }

  private updateCameraFromSpherical(): void {
    const offset = new THREE.Vector3().setFromSpherical(this.spherical);
    this.camera.position.copy(this.target).add(offset);
    this.camera.lookAt(this.target);
  }

  private bindEvents(): void {
    this.onPointerDown = this.onPointerDown.bind(this);
    this.onPointerMove = this.onPointerMove.bind(this);
    this.onPointerUp = this.onPointerUp.bind(this);
    this.onWheel = this.onWheel.bind(this);
    this.onContextMenu = this.onContextMenu.bind(this);

    this.domElement.addEventListener('pointerdown', this.onPointerDown);
    window.addEventListener('pointermove', this.onPointerMove);
    window.addEventListener('pointerup', this.onPointerUp);
    this.domElement.addEventListener('wheel', this.onWheel, { passive: false });
    this.domElement.addEventListener('contextmenu', this.onContextMenu);
  }

  private onContextMenu(e: Event): void {
    e.preventDefault();
  }

  private onPointerDown(e: PointerEvent): void {
    if (this.isLocked) return;
    this.isPointerDown = true;
    this.pointerButton = e.button;
    this.previousPointerPosition = { x: e.clientX, y: e.clientY };
  }

  private onPointerMove(e: PointerEvent): void {
    if (!this.isPointerDown || this.isLocked) return;

    const deltaX = e.clientX - this.previousPointerPosition.x;
    const deltaY = e.clientY - this.previousPointerPosition.y;
    this.previousPointerPosition = { x: e.clientX, y: e.clientY };

    if (this.pointerButton === 0) {
      // Left click: Orbit
      const rotateSpeed = 0.005;
      this.spherical.theta -= deltaX * rotateSpeed;
      this.spherical.phi -= deltaY * rotateSpeed;
      // Clamp phi to prevent flipping
      this.spherical.phi = Math.max(0.08, Math.min(Math.PI / 2 - 0.04, this.spherical.phi));
      this.updateCameraFromSpherical();
    } else if (this.pointerButton === 2) {
      // Right click: Pan
      const panSpeed = 0.015;
      const right = new THREE.Vector3(1, 0, 0).applyQuaternion(this.camera.quaternion);
      const up = new THREE.Vector3(0, 1, 0).applyQuaternion(this.camera.quaternion);

      this.target.addScaledVector(right, -deltaX * panSpeed);
      this.target.addScaledVector(up, deltaY * panSpeed);
      this.updateCameraFromSpherical();
    }
  }

  private onPointerUp(): void {
    this.isPointerDown = false;
    this.pointerButton = -1;
  }

  private onWheel(e: WheelEvent): void {
    if (this.isLocked) return;
    e.preventDefault();

    const zoomFactor = 1.0 + e.deltaY * 0.001;
    this.spherical.radius = Math.max(3.0, Math.min(65.0, this.spherical.radius * zoomFactor));
    this.updateCameraFromSpherical();
  }

  public dispose(): void {
    this.domElement.removeEventListener('pointerdown', this.onPointerDown);
    window.removeEventListener('pointermove', this.onPointerMove);
    window.removeEventListener('pointerup', this.onPointerUp);
    this.domElement.removeEventListener('wheel', this.onWheel);
    this.domElement.removeEventListener('contextmenu', this.onContextMenu);
  }
}
