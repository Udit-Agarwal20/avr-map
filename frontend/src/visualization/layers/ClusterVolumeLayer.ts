import * as THREE from 'three';
import { DynamicObjectCluster } from '../../types/perception';
import { PALETTE } from '../materials/PerceptionMaterials';

/**
 * @file ClusterVolumeLayer.ts
 * @brief 3D AABB wireframe and translucent volume rendering for frame-local dynamic clusters.
 *
 * Implements:
 * - Coral wireframe (#FF7557) with low fill opacity (0.08) keeping physical actors clearly visible
 * - Exact bounds derived from cluster aabbMin and aabbMax
 * - Raycastable volume for selection
 */
export class ClusterVolumeLayer {
  public group: THREE.Group;
  private fillMaterial: THREE.MeshBasicMaterial;
  private wireMaterial: THREE.LineBasicMaterial;
  private clusterMeshes: THREE.Mesh[] = [];
  private disposables: (THREE.BufferGeometry | THREE.Material)[] = [];
  private activeClusters: DynamicObjectCluster[] = [];

  constructor() {
    this.group = new THREE.Group();
    this.group.name = 'cluster-volume-layer';

    // Translucent fill: low opacity (0.08) so internal physical actor is clearly visible
    this.fillMaterial = new THREE.MeshBasicMaterial({
      color: PALETTE.dynamicCoral,
      transparent: true,
      opacity: 0.08,
      depthWrite: false,
    });

    // Wireframe edges: crisp hairline (0.82)
    this.wireMaterial = new THREE.LineBasicMaterial({
      color: PALETTE.dynamicCoral,
      transparent: true,
      opacity: 0.82,
      linewidth: 1.5,
    });
  }

  public updateClusters(clusters: DynamicObjectCluster[]): void {
    this.activeClusters = clusters;

    // Clear old objects
    while (this.group.children.length > 0) {
      const child = this.group.children[0];
      this.group.remove(child);
    }
    for (const d of this.disposables) {
      d.dispose();
    }
    this.disposables = [];
    this.clusterMeshes = [];

    for (const cluster of clusters) {
      const { aabbMin, aabbMax } = cluster;

      // Coordinate mapping:
      // Perception: x (lateral), y (longitudinal), z (elevation)
      // Three.js: (ThreeX: x, ThreeY: z, ThreeZ: -y)
      const width = Math.max(0.4, aabbMax.x - aabbMin.x);
      const height = Math.max(0.4, aabbMax.z - aabbMin.z);
      const depth = Math.max(0.4, aabbMax.y - aabbMin.y);

      const centerX = (aabbMin.x + aabbMax.x) / 2;
      const centerY = (aabbMin.z + aabbMax.z) / 2;
      const centerZ = -((aabbMin.y + aabbMax.y) / 2);

      const boxGeom = new THREE.BoxGeometry(width, height, depth);
      this.disposables.push(boxGeom);

      // Translucent interior volume
      const boxMesh = new THREE.Mesh(boxGeom, this.fillMaterial);
      boxMesh.position.set(centerX, centerY, centerZ);
      boxMesh.userData = { clusterId: cluster.clusterLocalId, cluster };
      this.clusterMeshes.push(boxMesh);

      // Crisp outline edges
      const edgesGeom = new THREE.EdgesGeometry(boxGeom);
      this.disposables.push(edgesGeom);
      const wireframe = new THREE.LineSegments(edgesGeom, this.wireMaterial);
      wireframe.position.set(centerX, centerY, centerZ);

      const clusterGroup = new THREE.Group();
      clusterGroup.name = `cluster-${cluster.clusterLocalId}`;
      clusterGroup.add(boxMesh);
      clusterGroup.add(wireframe);

      this.group.add(clusterGroup);
    }
  }

  public getRaycastTargets(): THREE.Mesh[] {
    return this.clusterMeshes;
  }

  public getActiveClusters(): DynamicObjectCluster[] {
    return this.activeClusters;
  }

  public setMode(mode: 'OVERVIEW' | 'SEMANTIC' | 'RESOLUTION' | 'DYNAMICS' | 'RAW_LIDAR'): void {
    switch (mode) {
      case 'DYNAMICS':
        this.fillMaterial.opacity = 0.16;
        this.wireMaterial.opacity = 0.95;
        break;
      case 'RESOLUTION':
        this.fillMaterial.opacity = 0.05;
        this.wireMaterial.opacity = 0.65;
        break;
      case 'RAW_LIDAR':
        this.fillMaterial.opacity = 0.02;
        this.wireMaterial.opacity = 0.35;
        break;
      case 'SEMANTIC':
        this.fillMaterial.opacity = 0.06;
        this.wireMaterial.opacity = 0.75;
        break;
      case 'OVERVIEW':
      default:
        this.fillMaterial.opacity = 0.08;
        this.wireMaterial.opacity = 0.82;
        break;
    }
  }

  public dispose(): void {
    for (const d of this.disposables) {
      d.dispose();
    }
    this.disposables = [];
    this.fillMaterial.dispose();
    this.wireMaterial.dispose();
  }
}
