import * as THREE from 'three';

/**
 * @file PerceptionMaterials.ts
 * @brief Calibrated materials for the AVR perception digital twin (Daylight Overcast Specification).
 *
 * Visual Character:
 * - Clear overcast daylight environment (medium charcoal asphalt, light concrete sidewalks, architectural facades).
 * - High-contrast physical separation between road, curbs, sidewalk, buildings, and actors.
 * - Restrained perception overlays (ice-cyan LiDAR sweeps, coral dynamic hazards, dyadic resolution field).
 * - No pitch-black void, no crushed blacks, no neon bloom.
 */

export interface PerceptionPalette {
  sky: number;
  roadAsphalt: number;
  roadMarkingWhite: number;
  roadMarkingYellow: number;
  sidewalkConcrete: number;
  curbConcrete: number;
  buildingConcrete: number;
  buildingAccent: number;
  buildingGlass: number;
  treeTrunk: number;
  treeFoliage: number;
  vehicleBody: number;
  vehicleGlass: number;
  vehicleTrim: number;
  secondaryVehicleBody: number;
  pedestrianJacket: number;
  pedestrianPants: number;
  pedestrianSkin: number;
  sensorPuck: number;
  lidarBeam: number;
  staticPoint: number;
  dynamicCoral: number;
  level0: number; // 5 cm  #62D3C6
  level1: number; // 10 cm #70B8E8
  level2: number; // 20 cm #D6B45F
  level3: number; // 40 cm #A18AC8
}

export const PALETTE: PerceptionPalette = {
  sky: 0x94a6b2,
  roadAsphalt: 0x2a323c,
  roadMarkingWhite: 0xf0f4f8,
  roadMarkingYellow: 0xebb356,
  sidewalkConcrete: 0x768392,
  curbConcrete: 0x8c99a8,
  buildingConcrete: 0x485566,
  buildingAccent: 0x5b6b80,
  buildingGlass: 0x26374a,
  treeTrunk: 0x4e3e34,
  treeFoliage: 0x3a5845,
  vehicleBody: 0x5a6a7e,
  vehicleGlass: 0x223244,
  vehicleTrim: 0x28323e,
  secondaryVehicleBody: 0x7c8a9a,
  pedestrianJacket: 0xb55a3c, // Terracotta jacket reads cleanly against asphalt
  pedestrianPants: 0x485260,
  pedestrianSkin: 0xdca882,
  sensorPuck: 0x7b8b9c,
  lidarBeam: 0x62d3c6,
  staticPoint: 0x55c6d4,
  dynamicCoral: 0xff7557,
  level0: 0x62d3c6, // 5 cm
  level1: 0x70b8e8, // 10 cm
  level2: 0xd6b45f, // 20 cm
  level3: 0xa18ac8, // 40 cm
};

export class PerceptionMaterials {
  public roadMaterial: THREE.MeshStandardMaterial;
  public laneWhiteMaterial: THREE.MeshStandardMaterial;
  public laneYellowMaterial: THREE.MeshStandardMaterial;
  public sidewalkMaterial: THREE.MeshStandardMaterial;
  public curbMaterial: THREE.MeshStandardMaterial;
  public buildingMaterial: THREE.MeshStandardMaterial;
  public buildingAccentMaterial: THREE.MeshStandardMaterial;
  public buildingGlassMaterial: THREE.MeshStandardMaterial;
  public treeTrunkMaterial: THREE.MeshStandardMaterial;
  public treeFoliageMaterial: THREE.MeshStandardMaterial;

  public egoBodyMaterial: THREE.MeshStandardMaterial;
  public egoGlassMaterial: THREE.MeshStandardMaterial;
  public egoTrimMaterial: THREE.MeshStandardMaterial;
  public sensorPuckMaterial: THREE.MeshStandardMaterial;
  public sensorGlowMaterial: THREE.MeshBasicMaterial;

  public secondaryVehicleMaterial: THREE.MeshStandardMaterial;
  public pedestrianJacketMaterial: THREE.MeshStandardMaterial;
  public pedestrianPantsMaterial: THREE.MeshStandardMaterial;
  public pedestrianSkinMaterial: THREE.MeshStandardMaterial;

  public staticPointMaterial: THREE.PointsMaterial;
  public dynamicPointMaterial: THREE.PointsMaterial;

  public clusterFillMaterial: THREE.MeshBasicMaterial;
  public clusterWireMaterial: THREE.LineBasicMaterial;

  constructor() {
    this.roadMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.roadAsphalt,
      roughness: 0.82,
      metalness: 0.08,
    });

    this.laneWhiteMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.roadMarkingWhite,
      roughness: 0.55,
      metalness: 0.05,
    });

    this.laneYellowMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.roadMarkingYellow,
      roughness: 0.55,
      metalness: 0.05,
    });

    this.sidewalkMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.sidewalkConcrete,
      roughness: 0.88,
      metalness: 0.04,
    });

    this.curbMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.curbConcrete,
      roughness: 0.78,
      metalness: 0.08,
    });

    this.buildingMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.buildingConcrete,
      roughness: 0.75,
      metalness: 0.12,
    });

    this.buildingAccentMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.buildingAccent,
      roughness: 0.7,
      metalness: 0.15,
    });

    this.buildingGlassMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.buildingGlass,
      roughness: 0.18,
      metalness: 0.65,
    });

    this.treeTrunkMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.treeTrunk,
      roughness: 0.95,
      metalness: 0.0,
    });

    this.treeFoliageMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.treeFoliage,
      roughness: 0.85,
      metalness: 0.0,
      flatShading: true,
    });

    this.egoBodyMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.vehicleBody,
      roughness: 0.35,
      metalness: 0.55,
    });

    this.egoGlassMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.vehicleGlass,
      roughness: 0.12,
      metalness: 0.85,
    });

    this.egoTrimMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.vehicleTrim,
      roughness: 0.85,
      metalness: 0.2,
    });

    this.sensorPuckMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.sensorPuck,
      roughness: 0.3,
      metalness: 0.7,
    });

    this.sensorGlowMaterial = new THREE.MeshBasicMaterial({
      color: PALETTE.lidarBeam,
    });

    this.secondaryVehicleMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.secondaryVehicleBody,
      roughness: 0.35,
      metalness: 0.6,
    });

    this.pedestrianJacketMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.pedestrianJacket,
      roughness: 0.9,
      metalness: 0.05,
    });

    this.pedestrianPantsMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.pedestrianPants,
      roughness: 0.92,
      metalness: 0.02,
    });

    this.pedestrianSkinMaterial = new THREE.MeshStandardMaterial({
      color: PALETTE.pedestrianSkin,
      roughness: 0.7,
      metalness: 0.0,
    });

    this.staticPointMaterial = new THREE.PointsMaterial({
      color: PALETTE.staticPoint,
      size: 0.05,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.7,
    });

    this.dynamicPointMaterial = new THREE.PointsMaterial({
      color: PALETTE.dynamicCoral,
      size: 0.09,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.98,
    });

    this.clusterFillMaterial = new THREE.MeshBasicMaterial({
      color: PALETTE.dynamicCoral,
      transparent: true,
      opacity: 0.08,
      depthWrite: false,
    });

    this.clusterWireMaterial = new THREE.LineBasicMaterial({
      color: PALETTE.dynamicCoral,
      transparent: true,
      opacity: 0.8,
      linewidth: 1.5,
    });
  }

  public dispose(): void {
    this.roadMaterial.dispose();
    this.laneWhiteMaterial.dispose();
    this.laneYellowMaterial.dispose();
    this.sidewalkMaterial.dispose();
    this.curbMaterial.dispose();
    this.buildingMaterial.dispose();
    this.buildingAccentMaterial.dispose();
    this.buildingGlassMaterial.dispose();
    this.treeTrunkMaterial.dispose();
    this.treeFoliageMaterial.dispose();
    this.egoBodyMaterial.dispose();
    this.egoGlassMaterial.dispose();
    this.egoTrimMaterial.dispose();
    this.sensorPuckMaterial.dispose();
    this.sensorGlowMaterial.dispose();
    this.secondaryVehicleMaterial.dispose();
    this.pedestrianJacketMaterial.dispose();
    this.pedestrianPantsMaterial.dispose();
    this.pedestrianSkinMaterial.dispose();
    this.staticPointMaterial.dispose();
    this.dynamicPointMaterial.dispose();
    this.clusterFillMaterial.dispose();
    this.clusterWireMaterial.dispose();
  }
}
