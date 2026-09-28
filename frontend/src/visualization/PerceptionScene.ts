import * as THREE from 'three';
import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { PerceptionFrame } from '../types/perception';
import { EgoVehicle } from './actors/EgoVehicle';
import { PhysicalActors } from './actors/PhysicalActors';
import { CameraController, CameraPreset, CameraViewMode } from './camera/CameraController';
import { RaycastSelector } from './interaction/RaycastSelector';
import { ObjectLabels } from './labels/ObjectLabels';
import { AdaptiveCellField } from './layers/AdaptiveCellField';
import { ClusterVolumeLayer } from './layers/ClusterVolumeLayer';
import { PointCloudLayer } from './layers/PointCloudLayer';
import { PerceptionMaterials } from './materials/PerceptionMaterials';
import { Lighting } from './scene/Lighting';
import { UrbanEnvironment } from './scene/UrbanEnvironment';

export interface SceneOptions {
  canvasContainer: HTMLDivElement;
  labelContainer: HTMLDivElement;
  onSelectCluster: (clusterId: number | null) => void;
  onSelectCell: (cellId: string | null) => void;
}

export type SceneMode = 'OVERVIEW' | 'SEMANTIC' | 'RESOLUTION' | 'DYNAMICS' | 'RAW_LIDAR';

/**
 * @file PerceptionScene.ts
 * @brief Master Plain Three.js perception renderer orchestrating digital twin and layers.
 *
 * Implements:
 * - Direct frame read from PerceptionFrameRuntime
 * - Zero high-volume point/cell data inside React or Zustand
 * - Complete disposal and WebGL resource safety
 */
export class PerceptionScene {
  private canvasContainer: HTMLDivElement;
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;

  private materials: PerceptionMaterials;
  private lighting: Lighting;
  private environment: UrbanEnvironment;
  private egoVehicle: EgoVehicle;
  private physicalActors: PhysicalActors;
  private cellField: AdaptiveCellField;
  private pointCloudLayer: PointCloudLayer;
  private clusterVolumeLayer: ClusterVolumeLayer;
  private objectLabels: ObjectLabels;
  private cameraController: CameraController;
  private raycastSelector: RaycastSelector;

  private unsubscribeFrame: () => void;
  private animationFrameId: number | null = null;
  private lastCameraMatrix = new THREE.Matrix4();
  private needsRender = true;
  private isDisposed = false;

  private mode: SceneMode = 'OVERVIEW';
  private requestedVisibility: Record<string, boolean> = {};
  private currentWidth = 1;
  private currentHeight = 1;

  constructor(options: SceneOptions) {
    this.canvasContainer = options.canvasContainer;
    const width = Math.max(1, this.canvasContainer.clientWidth);
    const height = Math.max(1, this.canvasContainer.clientHeight);
    this.needsRender = true;
    this.currentWidth = width;
    this.currentHeight = height;

    // 1. Scene & Camera (Natural 44° perspective FOV without distortion)
    this.scene = new THREE.Scene();
    this.scene.name = 'avr-perception-scene';
    this.camera = new THREE.PerspectiveCamera(44, width / height, 0.2, 250);

    // 2. High-performance WebGL Renderer
    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      powerPreference: 'high-performance',
      stencil: false,
    });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.15;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.renderer.domElement.style.display = 'block';
    this.canvasContainer.appendChild(this.renderer.domElement);

    // 3. Materials & Lighting
    this.materials = new PerceptionMaterials();
    this.lighting = new Lighting(this.scene);

    // 4. Urban Digital Twin Environment & Physical Actors
    this.environment = new UrbanEnvironment(this.materials);
    this.scene.add(this.environment.group);

    this.egoVehicle = new EgoVehicle(this.materials);
    this.scene.add(this.egoVehicle.group);

    this.physicalActors = new PhysicalActors(this.materials);
    this.scene.add(this.physicalActors.group);

    // 5. Perception Layers
    this.cellField = new AdaptiveCellField();
    this.scene.add(this.cellField.group);

    this.pointCloudLayer = new PointCloudLayer();
    this.scene.add(this.pointCloudLayer.group);

    this.clusterVolumeLayer = new ClusterVolumeLayer();
    this.scene.add(this.clusterVolumeLayer.group);

    // 6. Labels & Camera Controller
    this.objectLabels = new ObjectLabels(options.labelContainer, (id) => {
      options.onSelectCluster(id);
    });

    this.cameraController = new CameraController(this.camera, this.renderer.domElement);

    // 7. Raycasting Selection Interaction
    this.raycastSelector = new RaycastSelector(
      this.camera,
      this.renderer.domElement,
      this.clusterVolumeLayer,
      this.cellField,
      {
        onSelectCluster: (clusterId) => options.onSelectCluster(clusterId),
        onSelectCell: (cellId) => options.onSelectCell(cellId),
        onClearSelection: () => {
          options.onSelectCluster(null);
          options.onSelectCell(null);
        },
      }
    );

    // 8. Direct Frame Subscription

    this.onFrame(frameRuntime.getLatestFrame());
    this.unsubscribeFrame = frameRuntime.subscribeFrame((f) => this.onFrame(f));

    // 9. Start Animation Loop
    this.render = this.render.bind(this);
    this.render();
  }

  private onFrame(frame: PerceptionFrame): void {
    if (this.isDisposed) return;
    this.needsRender = true;
    this.physicalActors.group.visible = frame.scenarioId === 'D' && this.requestedVisibility.sceneContext !== false;
    this.cellField.updateCells(frame.cells);
    this.pointCloudLayer.updatePoints(frame.points, frame.dynamicPoints);
    this.clusterVolumeLayer.updateClusters(frame.dynamicClusters);
    this.objectLabels.updateClusters(frame.dynamicClusters);
  }

  public resize(width: number, height: number): void {
    if (this.isDisposed || width <= 0 || height <= 0) return;
    this.needsRender = true;
    this.currentWidth = width;
    this.currentHeight = height;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();

    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    this.renderer.setSize(width, height);
  }

  public setMode(mode: SceneMode): void {
    this.needsRender = true;
    this.mode = mode;
    this.objectLabels.setVisible(mode !== 'RAW_LIDAR' && this.requestedVisibility.objectLabels !== false);
    this.clusterVolumeLayer.group.visible = mode !== 'RAW_LIDAR' && this.requestedVisibility.clusterBounds !== false;
    this.lighting.setMode(mode);
    this.cellField.setMode(mode);
    this.pointCloudLayer.setMode(mode);
    this.clusterVolumeLayer.setMode(mode);
  }

  public selectCell(cellId: string | null): void {
    this.needsRender = true;
    this.cellField.setSelectedCell(cellId);
  }

  public setLayerVisibility(layer: string, visible: boolean): void {
    this.needsRender = true;
    this.requestedVisibility[layer] = visible;
    switch (layer) {
      case 'sceneContext':
        this.environment.group.visible = visible;
        this.egoVehicle.group.visible = visible;
        this.physicalActors.group.visible = visible && frameRuntime.getLatestFrame().scenarioId === 'D';
        break;
      case 'lidarPoints':
        this.pointCloudLayer.staticMesh.visible = visible;
        break;
      case 'adaptiveCells':
        this.cellField.group.visible = visible;
        break;
      case 'dynamicPoints':
        this.pointCloudLayer.dynamicMesh.visible = visible;
        break;
      case 'clusterBounds':
        this.clusterVolumeLayer.group.visible = visible && this.mode !== 'RAW_LIDAR';
        break;
      case 'objectLabels':
        this.objectLabels.setVisible(visible && this.mode !== 'RAW_LIDAR');
        break;
    }
  }

  public setCameraMode(mode: CameraViewMode): void {
    this.needsRender = true;
    this.cameraController.setViewMode(mode);
  }

  public applyCameraPreset(preset: CameraPreset): void {
    this.needsRender = true;
    this.cameraController.applyPreset(preset);
  }

  public metricReferencePixels(): number {
    const a = new THREE.Vector3(0, 0, -10).project(this.camera);
    const b = new THREE.Vector3(2, 0, -10).project(this.camera);
    return Math.hypot((a.x-b.x)*this.currentWidth/2, (a.y-b.y)*this.currentHeight/2);
  }

  public setPresentationLocked(locked: boolean): void {
    this.cameraController.setLocked(locked);
  }

  private render(): void {
    if (this.isDisposed) return;

    this.camera.updateMatrixWorld();
    const changed = !this.lastCameraMatrix.equals(this.camera.matrixWorld);
    if (document.hidden || (!this.needsRender && !changed)) {
      this.animationFrameId = requestAnimationFrame(this.render);
      return;
    }
    this.lastCameraMatrix.copy(this.camera.matrixWorld);
    this.needsRender = false;

    // Update 3D-to-2D label screen coordinates
    this.objectLabels.updatePositions(this.camera, this.currentWidth, this.currentHeight);

    // Render WebGL frame
    this.renderer.render(this.scene, this.camera);

    this.animationFrameId = requestAnimationFrame(this.render);
  }

  public dispose(): void {
    this.isDisposed = true;

    if (this.animationFrameId !== null) {
      cancelAnimationFrame(this.animationFrameId);
      this.animationFrameId = null;
    }

    if (this.unsubscribeFrame) {
      this.unsubscribeFrame();
    }

    this.raycastSelector.dispose();
    this.cameraController.dispose();
    this.objectLabels.dispose();
    this.clusterVolumeLayer.dispose();
    this.pointCloudLayer.dispose();
    this.cellField.dispose();
    this.physicalActors.dispose();
    this.egoVehicle.dispose();
    this.environment.dispose();
    this.lighting.dispose();
    this.materials.dispose();

    this.renderer.dispose();
    if (this.renderer.domElement.parentNode) {
      this.renderer.domElement.parentNode.removeChild(this.renderer.domElement);
    }
  }
}
