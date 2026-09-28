import * as THREE from 'three';
import { PerceptionMaterials } from '../materials/PerceptionMaterials';

/**
 * @file UrbanEnvironment.ts
 * @brief High-fidelity procedural urban intersection digital twin.
 *
 * Implements:
 * - Medium charcoal asphalt road with visible surface contrast and crisp markings
 * - Light neutral concrete sidewalks with 15cm raised curbs
 * - Multi-story architectural facades with window rhythms, setbacks, and ground-floor storefronts
 * - Natural low-poly clustered foliage trees (zero primitive cones)
 * - Sightline-cleared street furniture (no occlusion of pedestrian hazard or dynamic vehicle)
 */
export class UrbanEnvironment {
  public group: THREE.Group;
  private disposables: (THREE.BufferGeometry | THREE.Material)[] = [];

  constructor(materials: PerceptionMaterials) {
    this.group = new THREE.Group();
    this.group.name = 'urban-environment';

    this.buildGroundAndRoads(materials);
    this.buildLaneMarkings(materials);
    this.buildSidewalksAndCurbs(materials);
    this.buildArchitecturalBuildings(materials);
    this.buildStreetFurniture(materials);
    this.buildNaturalTrees(materials);
    this.buildParkedVehicle(materials);
  }

  private registerGeometry<T extends THREE.BufferGeometry>(geom: T): T {
    this.disposables.push(geom);
    return geom;
  }

  private registerMaterial<T extends THREE.Material>(mat: T): T {
    this.disposables.push(mat);
    return mat;
  }

  private buildGroundAndRoads(materials: PerceptionMaterials): void {
    // 1. Broad terrain / ground foundation (soft neutral tone matching daylight)
    const groundGeom = this.registerGeometry(new THREE.PlaneGeometry(140, 140));
    const groundMat = this.registerMaterial(
      new THREE.MeshStandardMaterial({
        color: 0x3d4752,
        roughness: 0.95,
        metalness: 0.05,
      })
    );
    const ground = new THREE.Mesh(groundGeom, groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.05;
    ground.receiveShadow = true;
    this.group.add(ground);

    // 2. Primary North-South Avenue (width 9.4m, spans ThreeZ -60 to +25)
    const primaryRoadGeom = this.registerGeometry(new THREE.PlaneGeometry(9.4, 85));
    const primaryRoad = new THREE.Mesh(primaryRoadGeom, materials.roadMaterial);
    primaryRoad.rotation.x = -Math.PI / 2;
    primaryRoad.position.set(0, 0, -17.5);
    primaryRoad.receiveShadow = true;
    this.group.add(primaryRoad);

    // 3. Cross Street (East-West Avenue: width 8.6m, spans ThreeX -60 to +60 at ThreeZ = -14.5)
    const crossRoadGeom = this.registerGeometry(new THREE.PlaneGeometry(120, 8.6));
    const crossRoad = new THREE.Mesh(crossRoadGeom, materials.roadMaterial);
    crossRoad.rotation.x = -Math.PI / 2;
    crossRoad.position.set(0, 0.002, -14.5);
    crossRoad.receiveShadow = true;
    this.group.add(crossRoad);
  }

  private buildLaneMarkings(materials: PerceptionMaterials): void {
    const yMarking = 0.006;

    // A. Double Yellow Centerline on Primary Road (South of intersection: ThreeZ +8 to -9.0)
    const southYellow1 = new THREE.Mesh(
      this.registerGeometry(new THREE.PlaneGeometry(0.12, 17)),
      materials.laneYellowMaterial
    );
    southYellow1.rotation.x = -Math.PI / 2;
    southYellow1.position.set(-0.11, yMarking, -0.5);
    this.group.add(southYellow1);

    const southYellow2 = new THREE.Mesh(
      this.registerGeometry(new THREE.PlaneGeometry(0.12, 17)),
      materials.laneYellowMaterial
    );
    southYellow2.rotation.x = -Math.PI / 2;
    southYellow2.position.set(0.11, yMarking, -0.5);
    this.group.add(southYellow2);

    // North of intersection: ThreeZ -20.0 to -50
    const northYellow1 = new THREE.Mesh(
      this.registerGeometry(new THREE.PlaneGeometry(0.12, 30)),
      materials.laneYellowMaterial
    );
    northYellow1.rotation.x = -Math.PI / 2;
    northYellow1.position.set(-0.11, yMarking, -35);
    this.group.add(northYellow1);

    const northYellow2 = new THREE.Mesh(
      this.registerGeometry(new THREE.PlaneGeometry(0.12, 30)),
      materials.laneYellowMaterial
    );
    northYellow2.rotation.x = -Math.PI / 2;
    northYellow2.position.set(0.11, yMarking, -35);
    this.group.add(northYellow2);

    // B. Dashed White Lane Divider Lines (Primary road: at x = -2.35 and +2.35)
    for (const laneX of [-2.35, 2.35]) {
      for (let z = 6; z >= -8.5; z -= 3.8) {
        const dash = new THREE.Mesh(
          this.registerGeometry(new THREE.PlaneGeometry(0.13, 2.0)),
          materials.laneWhiteMaterial
        );
        dash.rotation.x = -Math.PI / 2;
        dash.position.set(laneX, yMarking, z);
        this.group.add(dash);
      }
      for (let z = -20.5; z >= -52; z -= 3.8) {
        const dash = new THREE.Mesh(
          this.registerGeometry(new THREE.PlaneGeometry(0.13, 2.0)),
          materials.laneWhiteMaterial
        );
        dash.rotation.x = -Math.PI / 2;
        dash.position.set(laneX, yMarking, z);
        this.group.add(dash);
      }
    }

    // C. Pedestrian Crosswalk (at ThreeZ = -10.25, under pedestrian hazard!)
    for (let stripeX = -4.1; stripeX <= 4.1; stripeX += 0.82) {
      const stripe = new THREE.Mesh(
        this.registerGeometry(new THREE.PlaneGeometry(0.48, 2.4)),
        materials.laneWhiteMaterial
      );
      stripe.rotation.x = -Math.PI / 2;
      stripe.position.set(stripeX, yMarking, -10.25);
      this.group.add(stripe);
    }

    // Solid Stop Line before crosswalk (at ThreeZ = -8.2)
    const stopLine = new THREE.Mesh(
      this.registerGeometry(new THREE.PlaneGeometry(4.4, 0.45)),
      materials.laneWhiteMaterial
    );
    stopLine.rotation.x = -Math.PI / 2;
    stopLine.position.set(2.35, yMarking, -8.3);
    this.group.add(stopLine);
  }

  private buildSidewalksAndCurbs(materials: PerceptionMaterials): void {
    const curbHeight = 0.16;
    const sidewalkWidth = 3.6;

    // 4 Corner Sidewalk Blocks with Raised Curbs:
    const corners = [
      { x: -4.7 - sidewalkWidth / 2, z: 3.5, w: sidewalkWidth, l: 28 },
      { x: 4.7 + sidewalkWidth / 2, z: 3.5, w: sidewalkWidth, l: 28 },
      { x: -4.7 - sidewalkWidth / 2, z: -35.5, w: sidewalkWidth, l: 32 },
      { x: 4.7 + sidewalkWidth / 2, z: -35.5, w: sidewalkWidth, l: 32 },
    ];

    for (const c of corners) {
      // Main sidewalk surface
      const walk = new THREE.Mesh(
        this.registerGeometry(new THREE.BoxGeometry(c.w, curbHeight, c.l)),
        materials.sidewalkMaterial
      );
      walk.position.set(c.x, curbHeight / 2, c.z);
      walk.receiveShadow = true;
      this.group.add(walk);

      // Distinct raised curb stone facing road
      const curbX = c.x > 0 ? c.x - c.w / 2 + 0.08 : c.x + c.w / 2 - 0.08;
      const curbStone = new THREE.Mesh(
        this.registerGeometry(new THREE.BoxGeometry(0.16, curbHeight + 0.02, c.l)),
        materials.curbMaterial
      );
      curbStone.position.set(curbX, (curbHeight + 0.02) / 2, c.z);
      curbStone.receiveShadow = true;
      this.group.add(curbStone);
    }

    // East-West Sidewalk extensions along cross street
    const crossWalks = [
      { x: -25, z: -10.2 + sidewalkWidth / 2, w: 38, l: sidewalkWidth },
      { x: 25, z: -10.2 + sidewalkWidth / 2, w: 38, l: sidewalkWidth },
      { x: -25, z: -18.8 - sidewalkWidth / 2, w: 38, l: sidewalkWidth },
      { x: 25, z: -18.8 - sidewalkWidth / 2, w: 38, l: sidewalkWidth },
    ];
    for (const cw of crossWalks) {
      const walk = new THREE.Mesh(
        this.registerGeometry(new THREE.BoxGeometry(cw.w, curbHeight, cw.l)),
        materials.sidewalkMaterial
      );
      walk.position.set(cw.x, curbHeight / 2, cw.z);
      walk.receiveShadow = true;
      this.group.add(walk);
    }
  }

  private buildArchitecturalBuildings(materials: PerceptionMaterials): void {
    interface BuildingSpec {
      x: number;
      z: number;
      w: number;
      d: number;
      h: number;
      stories: number;
      hasSetback?: boolean;
    }

    const buildingSpecs: BuildingSpec[] = [
      // South-West corner block (Commercial/Office)
      { x: -17, z: 2, w: 15, d: 18, h: 16, stories: 5, hasSetback: true },
      // South-East corner block (Multi-tenant commercial)
      { x: 17, z: 2, w: 15, d: 18, h: 20, stories: 6, hasSetback: true },
      // North-West corner block (High-rise office)
      { x: -18, z: -32, w: 17, d: 20, h: 25, stories: 7, hasSetback: true },
      // North-East corner block (Mid-rise modern)
      { x: 18, z: -32, w: 17, d: 20, h: 18, stories: 5, hasSetback: false },
      // Distant background silhouettes
      { x: -19, z: -55, w: 19, d: 16, h: 32, stories: 9, hasSetback: true },
      { x: 19, z: -55, w: 19, d: 16, h: 28, stories: 8, hasSetback: true },
    ];

    for (const b of buildingSpecs) {
      const bldgGroup = new THREE.Group();
      bldgGroup.position.set(b.x, 0, b.z);

      const baseHeight = b.hasSetback ? b.h * 0.72 : b.h;

      // 1. Lower Architectural Mass
      const baseMesh = new THREE.Mesh(
        this.registerGeometry(new THREE.BoxGeometry(b.w, baseHeight, b.d)),
        materials.buildingMaterial
      );
      baseMesh.position.y = baseHeight / 2;
      baseMesh.castShadow = true;
      baseMesh.receiveShadow = true;
      bldgGroup.add(baseMesh);

      // 2. Ground-Floor Glass Storefront Band (Y: 0 to 3.8m)
      const facingRoadX = b.x < 0 ? b.w / 2 + 0.04 : -b.w / 2 - 0.04;
      const storefront = new THREE.Mesh(
        this.registerGeometry(new THREE.BoxGeometry(0.12, 3.4, b.d * 0.88)),
        materials.buildingGlassMaterial
      );
      storefront.position.set(facingRoadX, 1.85, 0);
      bldgGroup.add(storefront);

      // Storefront Entrance Mullions
      for (let mz = -b.d * 0.38; mz <= b.d * 0.38; mz += b.d * 0.22) {
        const mullion = new THREE.Mesh(
          this.registerGeometry(new THREE.BoxGeometry(0.18, 3.6, 0.22)),
          materials.buildingAccentMaterial
        );
        mullion.position.set(facingRoadX, 1.85, mz);
        bldgGroup.add(mullion);
      }

      // 3. Upper Story Window Ribbons & Architectural Mullions
      const storyH = baseHeight / (b.hasSetback ? Math.floor(b.stories * 0.72) : b.stories);
      const winH = storyH * 0.44;

      for (let s = 1; s < (b.hasSetback ? Math.floor(b.stories * 0.72) : b.stories); s++) {
        const ribbon = new THREE.Mesh(
          this.registerGeometry(new THREE.BoxGeometry(0.10, winH, b.d * 0.86)),
          materials.buildingGlassMaterial
        );
        ribbon.position.set(facingRoadX, s * storyH + winH / 2 + 0.2, 0);
        bldgGroup.add(ribbon);

        // Accent spandrel panel below each window ribbon
        const spandrel = new THREE.Mesh(
          this.registerGeometry(new THREE.BoxGeometry(0.14, storyH * 0.28, b.d * 0.88)),
          materials.buildingAccentMaterial
        );
        spandrel.position.set(facingRoadX, s * storyH - (storyH * 0.28) / 2 + 0.1, 0);
        bldgGroup.add(spandrel);
      }

      // 4. Upper Setback Story (if enabled)
      if (b.hasSetback) {
        const setbackH = b.h - baseHeight;
        const setbackW = b.w * 0.82;
        const setbackD = b.d * 0.82;
        const setbackMesh = new THREE.Mesh(
          this.registerGeometry(new THREE.BoxGeometry(setbackW, setbackH, setbackD)),
          materials.buildingAccentMaterial
        );
        setbackMesh.position.set(b.x < 0 ? -b.w * 0.08 : b.w * 0.08, baseHeight + setbackH / 2, 0);
        setbackMesh.castShadow = true;
        bldgGroup.add(setbackMesh);
      }

      // 5. Rooftop Mechanical Enclosure / Penthouse
      const roofH = b.hasSetback ? b.h : baseHeight;
      const hvac = new THREE.Mesh(
        this.registerGeometry(new THREE.BoxGeometry(b.w * 0.32, 1.8, b.d * 0.32)),
        materials.egoTrimMaterial
      );
      hvac.position.set(b.x < 0 ? -1.5 : 1.5, roofH + 0.9, -1.0);
      hvac.castShadow = true;
      bldgGroup.add(hvac);

      this.group.add(bldgGroup);
    }
  }

  private buildStreetFurniture(_materials: PerceptionMaterials): void {
    const poleMat = this.registerMaterial(
      new THREE.MeshStandardMaterial({
        color: 0x5a6778,
        roughness: 0.45,
        metalness: 0.75,
      })
    );
    const signalHousingMat = this.registerMaterial(
      new THREE.MeshStandardMaterial({
        color: 0x222a35,
        roughness: 0.7,
        metalness: 0.3,
      })
    );

    // 4 Traffic Signal Poles placed safely clear of camera sightlines
    const signalPoles = [
      { x: -5.4, z: -9.2, armDir: 1 },
      { x: 5.4, z: -8.0, armDir: -1 },  // Shifted away from pedestrian line of sight
      { x: -5.4, z: -19.5, armDir: 1 },
      { x: 5.4, z: -19.5, armDir: -1 },
    ];

    for (const sp of signalPoles) {
      // Upright pole
      const pole = new THREE.Mesh(
        this.registerGeometry(new THREE.CylinderGeometry(0.1, 0.13, 6.2, 12)),
        poleMat
      );
      pole.position.set(sp.x, 3.1, sp.z);
      pole.castShadow = true;
      this.group.add(pole);

      // Horizontal mast arm
      const armLength = 3.6;
      const arm = new THREE.Mesh(
        this.registerGeometry(new THREE.CylinderGeometry(0.06, 0.08, armLength, 8)),
        poleMat
      );
      arm.rotation.z = Math.PI / 2;
      arm.position.set(sp.x + (sp.armDir * armLength) / 2, 5.9, sp.z);
      this.group.add(arm);

      // Signal Head
      const head = new THREE.Mesh(
        this.registerGeometry(new THREE.BoxGeometry(0.35, 1.0, 0.35)),
        signalHousingMat
      );
      head.position.set(sp.x + sp.armDir * 2.8, 5.5, sp.z);
      this.group.add(head);
    }

    // Street light luminaires along sidewalk (placed outside critical sightlines)
    const lampPositions = [
      { x: -5.4, z: 4.0 },
      { x: 5.4, z: 4.0 },
      { x: -5.4, z: -34.0 },
      { x: 5.4, z: -34.0 },
    ];

    for (const lp of lampPositions) {
      const pole = new THREE.Mesh(
        this.registerGeometry(new THREE.CylinderGeometry(0.08, 0.11, 5.6, 10)),
        poleMat
      );
      pole.position.set(lp.x, 2.8, lp.z);
      pole.castShadow = true;
      this.group.add(pole);

      const lampHead = new THREE.Mesh(
        this.registerGeometry(new THREE.BoxGeometry(0.28, 0.14, 0.75)),
        poleMat
      );
      const dir = lp.x > 0 ? -1 : 1;
      lampHead.position.set(lp.x + dir * 0.45, 5.5, lp.z);
      this.group.add(lampHead);
    }
  }

  /**
   * Natural Clustered Procedural Trees (Replacing all primitive cones).
   * Sightline safe: Leaves pedestrian and vehicle completely unobstructed.
   */
  private buildNaturalTrees(materials: PerceptionMaterials): void {
    const treePositions = [
      { x: -6.5, z: 1.5, scale: 1.0 },
      { x: 6.8, z: 2.0, scale: 0.95 },  // South of crosswalk
      { x: -6.8, z: -25.0, scale: 1.15 },
      { x: 6.8, z: -25.0, scale: 1.1 },
      { x: 6.8, z: -38.0, scale: 1.25 },
      { x: -6.8, z: -42.0, scale: 1.2 },
    ];

    for (const tp of treePositions) {
      const treeGroup = new THREE.Group();
      treeGroup.position.set(tp.x, 0, tp.z);
      treeGroup.scale.setScalar(tp.scale);

      // 1. Tapered Trunk
      const trunk = new THREE.Mesh(
        this.registerGeometry(new THREE.CylinderGeometry(0.14, 0.20, 2.6, 8)),
        materials.treeTrunkMaterial
      );
      trunk.position.y = 1.3;
      trunk.castShadow = true;
      treeGroup.add(trunk);

      // 2. Natural Multi-Cluster Foliage (3-4 overlapping low-poly icosahedrons)
      const clusters = [
        { x: 0, y: 3.0, z: 0, r: 1.4 },
        { x: 0.35, y: 3.6, z: 0.25, r: 1.1 },
        { x: -0.3, y: 3.4, z: -0.3, r: 1.15 },
        { x: 0.1, y: 4.2, z: -0.1, r: 0.95 },
      ];

      for (const cl of clusters) {
        const foliage = new THREE.Mesh(
          this.registerGeometry(new THREE.IcosahedronGeometry(cl.r, 1)),
          materials.treeFoliageMaterial
        );
        foliage.position.set(cl.x, cl.y, cl.z);
        foliage.castShadow = true;
        treeGroup.add(foliage);
      }

      this.group.add(treeGroup);
    }
  }

  private buildParkedVehicle(materials: PerceptionMaterials): void {
    // Parked sedan on right shoulder at Y = -2.5 (ThreeZ = 2.5)
    const parkedGroup = new THREE.Group();
    parkedGroup.position.set(3.8, 0, 2.5);

    const bodyMat = this.registerMaterial(
      new THREE.MeshStandardMaterial({
        color: 0x5a6676,
        roughness: 0.45,
        metalness: 0.45,
      })
    );

    // Chassis
    const chassis = new THREE.Mesh(
      this.registerGeometry(new THREE.BoxGeometry(1.78, 0.52, 4.2)),
      bodyMat
    );
    chassis.position.y = 0.44;
    chassis.castShadow = true;
    parkedGroup.add(chassis);

    // Cabin
    const cabin = new THREE.Mesh(
      this.registerGeometry(new THREE.BoxGeometry(1.48, 0.48, 2.1)),
      materials.egoGlassMaterial
    );
    cabin.position.set(0, 0.94, -0.15);
    cabin.castShadow = true;
    parkedGroup.add(cabin);

    // Wheels
    const wheelGeom = this.registerGeometry(new THREE.CylinderGeometry(0.32, 0.32, 0.22, 14));
    wheelGeom.rotateZ(Math.PI / 2);
    for (const z of [-1.3, 1.3]) {
      for (const x of [-0.85, 0.85]) {
        const w = new THREE.Mesh(wheelGeom, materials.egoTrimMaterial);
        w.position.set(x, 0.32, z);
        w.castShadow = true;
        parkedGroup.add(w);
      }
    }

    this.group.add(parkedGroup);
  }

  public dispose(): void {
    for (const item of this.disposables) {
      item.dispose();
    }
    this.disposables = [];
  }
}
