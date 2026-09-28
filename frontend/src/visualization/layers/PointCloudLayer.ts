import * as THREE from 'three';
import { DynamicPoint, PerceptionPoint } from '../../types/perception';
import { semanticClassColor } from '../../utils/semantic';
import { PALETTE } from '../materials/PerceptionMaterials';

/**
 * @file PointCloudLayer.ts
 * @brief LiDAR point returns rendering discrete surface samples without blocky artifacts.
 *
 * Implements:
 * - Scaled sample point sizing (static: 0.05m, dynamic: 0.08m)
 * - Tracing physical actor surfaces, lane lines, curbs, and road edges
 * - True semantic color support in SEMANTIC mode
 * - High-readability raw point returns in RAW_LIDAR mode
 */
export class PointCloudLayer {
  public group: THREE.Group;
  private staticGeometry: THREE.BufferGeometry;
  private staticMaterial: THREE.PointsMaterial;
  public staticMesh: THREE.Points;

  private dynamicGeometry: THREE.BufferGeometry;
  private dynamicMaterial: THREE.PointsMaterial;
  public dynamicMesh: THREE.Points;

  constructor() {
    this.group = new THREE.Group();
    this.group.name = 'point-cloud-layer';

    // Static points (ice-cyan, small crisp samples)
    this.staticGeometry = new THREE.BufferGeometry();
    this.staticMaterial = new THREE.PointsMaterial({
      color: PALETTE.staticPoint,
      size: 0.05,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.65,
      depthWrite: false,
    });
    this.staticMesh = new THREE.Points(this.staticGeometry, this.staticMaterial);
    this.staticMesh.name = 'static-points';
    this.group.add(this.staticMesh);

    // Dynamic points (coral, crisp surface returns on vehicle & pedestrian)
    this.dynamicGeometry = new THREE.BufferGeometry();
    this.dynamicMaterial = new THREE.PointsMaterial({
      color: PALETTE.dynamicCoral,
      size: 0.08,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.98,
      depthWrite: false,
    });
    this.dynamicMesh = new THREE.Points(this.dynamicGeometry, this.dynamicMaterial);
    this.dynamicMesh.name = 'dynamic-points';
    this.group.add(this.dynamicMesh);
  }

  public updatePoints(allPoints: PerceptionPoint[], dynamicPoints: DynamicPoint[]): void {
    // 1. Static Points (routing === 'STATIC_FUSABLE')
    const staticList = allPoints.filter((p) => p.routing === 'STATIC_FUSABLE');
    const staticPositions = new Float32Array(staticList.length * 3);
    const staticColors = new Float32Array(staticList.length * 3);
    const colorHelper = new THREE.Color();

    for (let i = 0; i < staticList.length; i++) {
      const p = staticList[i];
      // Map perception (x, y, z) to Three.js: (x, z, -y)
      staticPositions[i * 3] = p.x;
      staticPositions[i * 3 + 1] = p.z;
      staticPositions[i * 3 + 2] = -p.y;

      // Semantic class color attribute
      colorHelper.set(semanticClassColor(p.semanticClass));
      staticColors[i * 3] = colorHelper.r;
      staticColors[i * 3 + 1] = colorHelper.g;
      staticColors[i * 3 + 2] = colorHelper.b;
    }
    this.staticGeometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute(staticPositions, 3)
    );
    this.staticGeometry.setAttribute(
      'color',
      new THREE.Float32BufferAttribute(staticColors, 3)
    );
    this.staticGeometry.computeBoundingSphere();

    // 2. Dynamic Points (Coral returns tracing actor surfaces)
    const dynPositions = new Float32Array(dynamicPoints.length * 3);
    const dynColors = new Float32Array(dynamicPoints.length * 3);
    for (let i = 0; i < dynamicPoints.length; i++) {
      const dp = dynamicPoints[i];
      colorHelper.set(semanticClassColor(dp.semanticClass));
      dynColors.set([colorHelper.r, colorHelper.g, colorHelper.b], i * 3);
      dynPositions[i * 3] = dp.position.x;
      dynPositions[i * 3 + 1] = dp.position.z;
      dynPositions[i * 3 + 2] = -dp.position.y;
    }
    this.dynamicGeometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute(dynPositions, 3)
    );
    this.dynamicGeometry.setAttribute('color', new THREE.Float32BufferAttribute(dynColors, 3));
    this.dynamicGeometry.computeBoundingSphere();
  }

  public setMode(mode: 'OVERVIEW' | 'SEMANTIC' | 'RESOLUTION' | 'DYNAMICS' | 'RAW_LIDAR'): void {
    this.dynamicMaterial.vertexColors = mode === 'SEMANTIC';
    this.dynamicMaterial.color.set(mode === 'SEMANTIC' ? 0xffffff : PALETTE.dynamicCoral);
    this.dynamicMaterial.needsUpdate = true;
    switch (mode) {
      case 'SEMANTIC':
        this.staticMaterial.vertexColors = true;
        this.staticMaterial.color.set(0xffffff);
        this.staticMaterial.opacity = 0.85;
        this.staticMaterial.size = 0.06;
        this.staticMaterial.needsUpdate = true;
        this.dynamicMaterial.opacity = 0.95;
        this.dynamicMaterial.size = 0.08;
        break;
      case 'DYNAMICS':
        this.staticMaterial.vertexColors = false;
        this.staticMaterial.color.set(PALETTE.staticPoint);
        this.staticMaterial.opacity = 0.12;
        this.staticMaterial.size = 0.05;
        this.staticMaterial.needsUpdate = true;
        this.dynamicMaterial.opacity = 1.0;
        this.dynamicMaterial.size = 0.10;
        break;
      case 'RAW_LIDAR':
        this.staticMaterial.vertexColors = false;
        this.staticMaterial.color.set(PALETTE.staticPoint);
        this.staticMaterial.opacity = 0.95;
        this.staticMaterial.size = 0.085;
        this.staticMaterial.needsUpdate = true;
        this.dynamicMaterial.opacity = 1.0;
        this.dynamicMaterial.size = 0.11;
        break;
      case 'RESOLUTION':
        this.staticMaterial.vertexColors = false;
        this.staticMaterial.color.set(PALETTE.staticPoint);
        this.staticMaterial.opacity = 0.22;
        this.staticMaterial.size = 0.05;
        this.staticMaterial.needsUpdate = true;
        this.dynamicMaterial.opacity = 0.85;
        this.dynamicMaterial.size = 0.07;
        break;
      case 'OVERVIEW':
      default:
        this.staticMaterial.vertexColors = false;
        this.staticMaterial.color.set(PALETTE.staticPoint);
        this.staticMaterial.opacity = 0.65;
        this.staticMaterial.size = 0.05;
        this.staticMaterial.needsUpdate = true;
        this.dynamicMaterial.opacity = 0.98;
        this.dynamicMaterial.size = 0.08;
        break;
    }
  }

  public dispose(): void {
    this.staticGeometry.dispose();
    this.staticMaterial.dispose();
    this.dynamicGeometry.dispose();
    this.dynamicMaterial.dispose();
  }
}
