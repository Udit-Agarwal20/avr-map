import * as THREE from 'three';
import { PALETTE } from '../materials/PerceptionMaterials';

/**
 * @file Lighting.ts
 * @brief High-contrast overcast daylight lighting and atmospheric horizon fog.
 *
 * Implements:
 * - Cool overcast daylight sky background and matching horizon fog
 * - Strong sky hemisphere contribution eliminating crushed shadows
 * - Soft directional daylight sun with high-resolution shadow maps
 * - High exposure for clear visibility of environment and physical actors
 */
export class Lighting {
  public hemisphereLight: THREE.HemisphereLight;
  public sunLight: THREE.DirectionalLight;
  public fillLight: THREE.DirectionalLight;
  public ambientLight: THREE.AmbientLight;
  public fog: THREE.Fog;

  constructor(scene: THREE.Scene) {
    // Sky background color (cool pale blue-gray)
    scene.background = new THREE.Color(PALETTE.sky);

    // Ambient fill to eliminate crushed darks while preserving shading
    this.ambientLight = new THREE.AmbientLight(0xdbe4eb, 0.45);
    scene.add(this.ambientLight);

    // Hemisphere light: bright cool sky (0xdce5ec) + soft neutral ground bounce (0x55606d)
    this.hemisphereLight = new THREE.HemisphereLight(0xdce5ec, 0x626d7c, 1.25);
    scene.add(this.hemisphereLight);

    // Primary directional daylight sun
    this.sunLight = new THREE.DirectionalLight(0xfffdf7, 1.45);
    this.sunLight.position.set(18, 30, 15);
    this.sunLight.castShadow = true;
    this.sunLight.shadow.mapSize.width = 2048;
    this.sunLight.shadow.mapSize.height = 2048;
    this.sunLight.shadow.camera.near = 0.5;
    this.sunLight.shadow.camera.far = 90;
    this.sunLight.shadow.camera.left = -30;
    this.sunLight.shadow.camera.right = 30;
    this.sunLight.shadow.camera.top = 30;
    this.sunLight.shadow.camera.bottom = -30;
    this.sunLight.shadow.bias = -0.0003;
    scene.add(this.sunLight);

    // Soft cool daylight fill from opposite side
    this.fillLight = new THREE.DirectionalLight(0xb0c4de, 0.60);
    this.fillLight.position.set(-18, 16, -18);
    scene.add(this.fillLight);

    // Linear atmospheric horizon fog matching sky color (keeps intersection crisp, softens horizon)
    this.fog = new THREE.Fog(PALETTE.sky, 38, 115);
    scene.fog = this.fog;
  }

  public setMode(mode: 'OVERVIEW' | 'SEMANTIC' | 'RESOLUTION' | 'DYNAMICS' | 'RAW_LIDAR'): void {
    if (mode === 'RESOLUTION') {
      this.hemisphereLight.intensity = 0.85;
      this.sunLight.intensity = 1.05;
      this.fillLight.intensity = 0.35;
      this.ambientLight.intensity = 0.28;
    } else if (mode === 'SEMANTIC') {
      this.hemisphereLight.intensity = 0.90;
      this.sunLight.intensity = 1.10;
      this.fillLight.intensity = 0.40;
      this.ambientLight.intensity = 0.30;
    } else if (mode === 'DYNAMICS') {
      this.hemisphereLight.intensity = 0.80;
      this.sunLight.intensity = 1.00;
      this.fillLight.intensity = 0.30;
      this.ambientLight.intensity = 0.25;
    } else if (mode === 'RAW_LIDAR') {
      this.hemisphereLight.intensity = 0.35;
      this.sunLight.intensity = 0.40;
      this.fillLight.intensity = 0.15;
      this.ambientLight.intensity = 0.15;
    } else {
      // OVERVIEW (default hero)
      this.hemisphereLight.intensity = 1.15;
      this.sunLight.intensity = 1.35;
      this.fillLight.intensity = 0.55;
      this.ambientLight.intensity = 0.38;
    }
  }

  public dispose(): void {
    this.sunLight.dispose();
    this.fillLight.dispose();
  }
}
