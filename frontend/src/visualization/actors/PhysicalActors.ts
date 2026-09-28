import * as THREE from 'three';
import { PerceptionMaterials } from '../materials/PerceptionMaterials';

/**
 * @file PhysicalActors.ts
 * @brief Procedural physical actors (pedestrian and secondary dynamic vehicle)
 *        anchored within perception cluster bounding volumes.
 *
 * Requirements:
 * - Secondary vehicle physically visible inside FRAME C-00 AABB.
 * - Recognizable human pedestrian silhouette in walking pose inside FRAME C-01 AABB.
 * - Realistic anatomical proportions (height ~1.72m, head ~0.11m, shoulder ~0.4m).
 */
export class PhysicalActors {
  public group: THREE.Group;
  private disposables: (THREE.BufferGeometry | THREE.Material)[] = [];

  constructor(materials: PerceptionMaterials) {
    this.group = new THREE.Group();
    this.group.name = 'physical-actors';

    this.buildDynamicVehicle(materials);
    this.buildCrossingPedestrian(materials);
  }

  private registerGeometry<T extends THREE.BufferGeometry>(geom: T): T {
    this.disposables.push(geom);
    return geom;
  }

  private registerMaterial<T extends THREE.Material>(mat: T): T {
    this.disposables.push(mat);
    return mat;
  }

  /**
   * Secondary Dynamic Vehicle (Cluster 0: x = -3.0, y = 15.5 -> ThreeX = -3.0, ThreeZ = -15.5)
   * Approaching oncoming traffic facing towards +Z (toward ego vehicle).
   */
  private buildDynamicVehicle(materials: PerceptionMaterials): void {
    const vehGroup = new THREE.Group();
    vehGroup.name = 'actor-vehicle-c00';
    vehGroup.position.set(-3.0, 0, -15.5);
    vehGroup.rotation.y = Math.PI; // Heading south towards ego vehicle

    // 1. Lower Body / Chassis (length 4.1m, width 1.76m, height 0.52m)
    const lowerBodyGeom = this.registerGeometry(new THREE.BoxGeometry(1.76, 0.52, 4.1));
    const lowerBody = new THREE.Mesh(lowerBodyGeom, materials.secondaryVehicleMaterial);
    lowerBody.position.set(0, 0.44, 0);
    lowerBody.castShadow = true;
    lowerBody.receiveShadow = true;
    vehGroup.add(lowerBody);

    // Front hood / nose
    const hoodGeom = this.registerGeometry(new THREE.BoxGeometry(1.72, 0.22, 1.15));
    const hood = new THREE.Mesh(hoodGeom, materials.secondaryVehicleMaterial);
    hood.position.set(0, 0.58, 1.25);
    hood.castShadow = true;
    vehGroup.add(hood);

    // 2. Cabin Greenhouse (Glass + Pillars)
    const cabinGeom = this.registerGeometry(new THREE.BoxGeometry(1.48, 0.52, 2.1));
    const cabin = new THREE.Mesh(cabinGeom, materials.egoGlassMaterial);
    cabin.position.set(0, 0.94, -0.15);
    cabin.castShadow = true;
    vehGroup.add(cabin);

    // Roof cap
    const roofGeom = this.registerGeometry(new THREE.BoxGeometry(1.42, 0.08, 1.85));
    const roof = new THREE.Mesh(roofGeom, materials.secondaryVehicleMaterial);
    roof.position.set(0, 1.22, -0.15);
    roof.castShadow = true;
    vehGroup.add(roof);

    // 3. 4 Wheels contacting road
    const wheelGeom = this.registerGeometry(new THREE.CylinderGeometry(0.32, 0.32, 0.24, 16));
    wheelGeom.rotateZ(Math.PI / 2);

    const rimGeom = this.registerGeometry(new THREE.CylinderGeometry(0.2, 0.2, 0.26, 12));
    rimGeom.rotateZ(Math.PI / 2);

    const wheelPositions = [
      { x: -0.86, z: 1.25 },
      { x: 0.86, z: 1.25 },
      { x: -0.86, z: -1.25 },
      { x: 0.86, z: -1.25 },
    ];

    for (const wp of wheelPositions) {
      const tire = new THREE.Mesh(wheelGeom, materials.egoTrimMaterial);
      tire.position.set(wp.x, 0.32, wp.z);
      tire.castShadow = true;
      vehGroup.add(tire);

      const rim = new THREE.Mesh(rimGeom, materials.sensorPuckMaterial);
      rim.position.set(wp.x, 0.32, wp.z);
      vehGroup.add(rim);
    }

    // 4. Lights (Front facing +Z in local space)
    const headlightMat = this.registerMaterial(new THREE.MeshBasicMaterial({ color: 0xfffee4 }));
    const hlGeom = this.registerGeometry(new THREE.BoxGeometry(0.32, 0.08, 0.04));

    const hlL = new THREE.Mesh(hlGeom, headlightMat);
    hlL.position.set(-0.62, 0.52, 2.06);
    vehGroup.add(hlL);

    const hlR = new THREE.Mesh(hlGeom, headlightMat);
    hlR.position.set(0.62, 0.52, 2.06);
    vehGroup.add(hlR);

    // Taillights
    const taillightMat = this.registerMaterial(new THREE.MeshBasicMaterial({ color: 0xcc2929 }));
    const tlGeom = this.registerGeometry(new THREE.BoxGeometry(0.32, 0.08, 0.04));

    const tlL = new THREE.Mesh(tlGeom, taillightMat);
    tlL.position.set(-0.62, 0.58, -2.06);
    vehGroup.add(tlL);

    const tlR = new THREE.Mesh(tlGeom, taillightMat);
    tlR.position.set(0.62, 0.58, -2.06);
    vehGroup.add(tlR);

    // Contact shadow
    const shadowGeom = this.registerGeometry(new THREE.PlaneGeometry(1.95, 4.3));
    const shadowMat = this.registerMaterial(
      new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.42 })
    );
    const shadow = new THREE.Mesh(shadowGeom, shadowMat);
    shadow.rotation.x = -Math.PI / 2;
    shadow.position.set(0, 0.015, 0);
    vehGroup.add(shadow);

    this.group.add(vehGroup);
  }

  /**
   * Crossing Pedestrian (Cluster 1: x = 3.65, y = 10.25 -> ThreeX = 3.65, ThreeZ = -10.25)
   * Crossing the road from right to left (heading along -X).
   */
  private buildCrossingPedestrian(materials: PerceptionMaterials): void {
    const pedGroup = new THREE.Group();
    pedGroup.name = 'actor-pedestrian-c01';
    pedGroup.position.set(3.60, 0, -10.20);
    pedGroup.rotation.y = -Math.PI / 2; // Facing across the street (towards -X)

    // 1. Head (sphere r = 0.11m)
    const headGeom = this.registerGeometry(new THREE.SphereGeometry(0.11, 14, 14));
    const head = new THREE.Mesh(headGeom, materials.pedestrianSkinMaterial);
    head.position.set(0, 1.61, 0);
    head.castShadow = true;
    pedGroup.add(head);

    // Hair cap
    const hairGeom = this.registerGeometry(new THREE.SphereGeometry(0.115, 12, 10, 0, Math.PI * 2, 0, Math.PI * 0.55));
    const hairMat = this.registerMaterial(new THREE.MeshStandardMaterial({ color: 0x241d18, roughness: 0.9 }));
    const hair = new THREE.Mesh(hairGeom, hairMat);
    hair.position.set(0, 1.62, 0);
    hair.castShadow = true;
    pedGroup.add(hair);

    // Neck
    const neckGeom = this.registerGeometry(new THREE.CylinderGeometry(0.05, 0.055, 0.08, 10));
    const neck = new THREE.Mesh(neckGeom, materials.pedestrianSkinMaterial);
    neck.position.set(0, 1.48, 0);
    pedGroup.add(neck);

    // 2. Torso (Jacket: width 0.38m, height 0.54m, depth 0.22m)
    const torsoGeom = this.registerGeometry(new THREE.BoxGeometry(0.36, 0.52, 0.22));
    const torso = new THREE.Mesh(torsoGeom, materials.pedestrianJacketMaterial);
    torso.position.set(0, 1.20, 0);
    torso.castShadow = true;
    torso.receiveShadow = true;
    pedGroup.add(torso);

    // 3. Arms (In natural walking swing)
    const armGeom = this.registerGeometry(new THREE.CylinderGeometry(0.045, 0.04, 0.54, 8));

    // Left arm (swung forward)
    const leftArm = new THREE.Mesh(armGeom, materials.pedestrianJacketMaterial);
    leftArm.position.set(0.22, 1.15, 0.10);
    leftArm.rotation.x = 0.35;
    leftArm.castShadow = true;
    pedGroup.add(leftArm);

    // Right arm (swung backward)
    const rightArm = new THREE.Mesh(armGeom, materials.pedestrianJacketMaterial);
    rightArm.position.set(-0.22, 1.15, -0.10);
    rightArm.rotation.x = -0.35;
    rightArm.castShadow = true;
    pedGroup.add(rightArm);

    // 4. Legs (In natural mid-stride crossing pose)
    const legGeom = this.registerGeometry(new THREE.CylinderGeometry(0.065, 0.055, 0.78, 10));

    // Forward leg (right leg stride forward along local Z)
    const legFwd = new THREE.Mesh(legGeom, materials.pedestrianPantsMaterial);
    legFwd.position.set(0.10, 0.52, 0.14);
    legFwd.rotation.x = 0.28;
    legFwd.castShadow = true;
    pedGroup.add(legFwd);

    // Rear leg (left leg stride back along local -Z)
    const legBack = new THREE.Mesh(legGeom, materials.pedestrianPantsMaterial);
    legBack.position.set(-0.10, 0.52, -0.14);
    legBack.rotation.x = -0.28;
    legBack.castShadow = true;
    pedGroup.add(legBack);

    // Shoes
    const shoeGeom = this.registerGeometry(new THREE.BoxGeometry(0.11, 0.07, 0.22));
    const shoeMat = this.registerMaterial(new THREE.MeshStandardMaterial({ color: 0x1f2328, roughness: 0.8 }));

    const shoeFwd = new THREE.Mesh(shoeGeom, shoeMat);
    shoeFwd.position.set(0.10, 0.04, 0.25);
    shoeFwd.castShadow = true;
    pedGroup.add(shoeFwd);

    const shoeBack = new THREE.Mesh(shoeGeom, shoeMat);
    shoeBack.position.set(-0.10, 0.04, -0.25);
    shoeBack.castShadow = true;
    pedGroup.add(shoeBack);

    // Subtle contact shadow
    const shadowGeom = this.registerGeometry(new THREE.PlaneGeometry(0.7, 0.9));
    const shadowMat = this.registerMaterial(
      new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.35 })
    );
    const shadow = new THREE.Mesh(shadowGeom, shadowMat);
    shadow.rotation.x = -Math.PI / 2;
    shadow.position.set(0, 0.015, 0);
    pedGroup.add(shadow);

    this.group.add(pedGroup);
  }

  public dispose(): void {
    for (const d of this.disposables) {
      d.dispose();
    }
    this.disposables = [];
  }
}
