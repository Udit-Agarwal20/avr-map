import * as THREE from 'three';
import { PerceptionMaterials } from '../materials/PerceptionMaterials';

/**
 * @file EgoVehicle.ts
 * @brief Premium sculpted autonomous research vehicle with roof LiDAR sensor module.
 *
 * Proportions:
 * - Length: 4.42m
 * - Width: 1.86m (lower chassis) / 1.50m (tapered cabin)
 * - Height: 1.45m
 * - Four wheels firmly contacting road surface (Y = 0)
 * - Distinct sloped hood, raked windshield, rear deck, and wheel arches.
 */
export class EgoVehicle {
  public group: THREE.Group;
  private disposables: (THREE.BufferGeometry | THREE.Material)[] = [];
  private sweepGroup: THREE.Group;
  private sweepAngle = 0;

  constructor(materials: PerceptionMaterials) {
    this.group = new THREE.Group();
    this.group.name = 'ego-vehicle';

    this.buildBody(materials);
    this.buildCabinAndGlass(materials);
    this.buildWheels(materials);
    this.buildLighting(materials);
    this.buildRoofSensorRig(materials);
    this.sweepGroup = this.buildLidarSamplingField();
    this.group.add(this.sweepGroup);
  }

  private registerGeometry<T extends THREE.BufferGeometry>(geom: T): T {
    this.disposables.push(geom);
    return geom;
  }

  private registerMaterial<T extends THREE.Material>(mat: T): T {
    this.disposables.push(mat);
    return mat;
  }

  private buildBody(materials: PerceptionMaterials): void {
    const bodyGroup = new THREE.Group();
    bodyGroup.name = 'ego-chassis';

    // 1. Lower Core Tub / Rocker Panels (length 4.38m, width 1.84m, height 0.38m)
    const tubGeom = this.registerGeometry(new THREE.BoxGeometry(1.84, 0.36, 4.36));
    const tub = new THREE.Mesh(tubGeom, materials.egoBodyMaterial);
    tub.position.set(0, 0.36, 0);
    tub.castShadow = true;
    tub.receiveShadow = true;
    bodyGroup.add(tub);

    // 2. Sculpted Front Hood (slopes gently forward from Y = 0.68m to Y = 0.52m at front bumper)
    const hoodGeom = this.registerGeometry(new THREE.BoxGeometry(1.78, 0.22, 1.45));
    const hood = new THREE.Mesh(hoodGeom, materials.egoBodyMaterial);
    hood.position.set(0, 0.60, -1.35);
    hood.rotation.x = 0.08; // subtle aerodynamic downward slope
    hood.castShadow = true;
    bodyGroup.add(hood);

    // Front Bumper / Air Dam
    const bumperGeom = this.registerGeometry(new THREE.BoxGeometry(1.82, 0.28, 0.32));
    const bumper = new THREE.Mesh(bumperGeom, materials.egoTrimMaterial);
    bumper.position.set(0, 0.32, -2.12);
    bumper.castShadow = true;
    bodyGroup.add(bumper);

    // 3. Rear Trunk Deck
    const trunkGeom = this.registerGeometry(new THREE.BoxGeometry(1.76, 0.26, 1.05));
    const trunk = new THREE.Mesh(trunkGeom, materials.egoBodyMaterial);
    trunk.position.set(0, 0.64, 1.55);
    trunk.castShadow = true;
    bodyGroup.add(trunk);

    // Rear Bumper / Diffuser
    const rearBumperGeom = this.registerGeometry(new THREE.BoxGeometry(1.82, 0.30, 0.28));
    const rearBumper = new THREE.Mesh(rearBumperGeom, materials.egoTrimMaterial);
    rearBumper.position.set(0, 0.32, 2.12);
    rearBumper.castShadow = true;
    bodyGroup.add(rearBumper);

    // 4. Contact Shadow on Asphalt
    const shadowGeom = this.registerGeometry(new THREE.PlaneGeometry(2.1, 4.6));
    const shadowMat = this.registerMaterial(
      new THREE.MeshBasicMaterial({ color: 0x0c1218, transparent: true, opacity: 0.55 })
    );
    const shadow = new THREE.Mesh(shadowGeom, shadowMat);
    shadow.rotation.x = -Math.PI / 2;
    shadow.position.set(0, 0.015, 0);
    bodyGroup.add(shadow);

    this.group.add(bodyGroup);
  }

  private buildCabinAndGlass(materials: PerceptionMaterials): void {
    const cabinGroup = new THREE.Group();
    cabinGroup.name = 'ego-cabin';

    // Raked Front Windshield (sloped at ~45 degrees from hood to roof)
    const windshieldGeom = this.registerGeometry(new THREE.BoxGeometry(1.50, 0.06, 0.95));
    const windshield = new THREE.Mesh(windshieldGeom, materials.egoGlassMaterial);
    windshield.position.set(0, 0.98, -0.68);
    windshield.rotation.x = 0.72; // ~41 deg rake
    windshield.castShadow = true;
    cabinGroup.add(windshield);

    // Center Cabin Enclosure
    const cabinCoreGeom = this.registerGeometry(new THREE.BoxGeometry(1.52, 0.48, 1.65));
    const cabinCore = new THREE.Mesh(cabinCoreGeom, materials.egoGlassMaterial);
    cabinCore.position.set(0, 1.04, 0.22);
    cabinCore.castShadow = true;
    cabinGroup.add(cabinCore);

    // Sloped Rear Glass
    const rearGlassGeom = this.registerGeometry(new THREE.BoxGeometry(1.48, 0.06, 0.85));
    const rearGlass = new THREE.Mesh(rearGlassGeom, materials.egoGlassMaterial);
    rearGlass.position.set(0, 0.98, 1.15);
    rearGlass.rotation.x = -0.68;
    rearGlass.castShadow = true;
    cabinGroup.add(rearGlass);

    // Sleek Metallic Roof Cap
    const roofGeom = this.registerGeometry(new THREE.BoxGeometry(1.46, 0.07, 1.62));
    const roof = new THREE.Mesh(roofGeom, materials.egoBodyMaterial);
    roof.position.set(0, 1.30, 0.25);
    roof.castShadow = true;
    cabinGroup.add(roof);

    // Slim Black Pillars (A-pillar left & right)
    const pillarGeom = this.registerGeometry(new THREE.BoxGeometry(0.06, 0.06, 1.0));
    const pillarL = new THREE.Mesh(pillarGeom, materials.egoTrimMaterial);
    pillarL.position.set(-0.72, 0.98, -0.68);
    pillarL.rotation.x = 0.72;
    cabinGroup.add(pillarL);

    const pillarR = new THREE.Mesh(pillarGeom, materials.egoTrimMaterial);
    pillarR.position.set(0.72, 0.98, -0.68);
    pillarR.rotation.x = 0.72;
    cabinGroup.add(pillarR);

    this.group.add(cabinGroup);
  }

  private buildWheels(materials: PerceptionMaterials): void {
    const wheelRadius = 0.33;
    const wheelWidth = 0.25;

    const tireGeom = this.registerGeometry(
      new THREE.CylinderGeometry(wheelRadius, wheelRadius, wheelWidth, 20)
    );
    tireGeom.rotateZ(Math.PI / 2);

    const rimGeom = this.registerGeometry(
      new THREE.CylinderGeometry(wheelRadius * 0.65, wheelRadius * 0.65, wheelWidth + 0.015, 14)
    );
    rimGeom.rotateZ(Math.PI / 2);

    const hubcapGeom = this.registerGeometry(
      new THREE.CylinderGeometry(wheelRadius * 0.22, wheelRadius * 0.22, wheelWidth + 0.025, 8)
    );
    hubcapGeom.rotateZ(Math.PI / 2);

    const hubcapMat = this.registerMaterial(
      new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.8, roughness: 0.2 })
    );

    const wheelPositions = [
      { x: -0.90, z: -1.38 }, // Front-Left
      { x: 0.90, z: -1.38 },  // Front-Right
      { x: -0.90, z: 1.38 },   // Rear-Left
      { x: 0.90, z: 1.38 },    // Rear-Right
    ];

    for (const wp of wheelPositions) {
      const tire = new THREE.Mesh(tireGeom, materials.egoTrimMaterial);
      tire.position.set(wp.x, wheelRadius, wp.z);
      tire.castShadow = true;
      this.group.add(tire);

      const rim = new THREE.Mesh(rimGeom, materials.sensorPuckMaterial);
      rim.position.set(wp.x, wheelRadius, wp.z);
      this.group.add(rim);

      const hub = new THREE.Mesh(hubcapGeom, hubcapMat);
      hub.position.set(wp.x, wheelRadius, wp.z);
      this.group.add(hub);
    }
  }

  private buildLighting(_materials: PerceptionMaterials): void {
    // 1. Front Headlights / Horizontal LED Light Blade (facing -Z)
    const headlightMat = this.registerMaterial(
      new THREE.MeshBasicMaterial({ color: 0xf1f9ff })
    );

    // Left and right headlamp clusters
    const headlampGeom = this.registerGeometry(new THREE.BoxGeometry(0.38, 0.07, 0.05));
    const hlL = new THREE.Mesh(headlampGeom, headlightMat);
    hlL.position.set(-0.64, 0.54, -2.18);
    this.group.add(hlL);

    const hlR = new THREE.Mesh(headlampGeom, headlightMat);
    hlR.position.set(0.64, 0.54, -2.18);
    this.group.add(hlR);

    // Center connecting daytime running light strip
    const drlGeom = this.registerGeometry(new THREE.BoxGeometry(0.88, 0.025, 0.04));
    const drl = new THREE.Mesh(drlGeom, headlightMat);
    drl.position.set(0, 0.56, -2.19);
    this.group.add(drl);

    // 2. Rear Continuous Red Light Blade (facing +Z)
    const taillightMat = this.registerMaterial(
      new THREE.MeshBasicMaterial({ color: 0xd92626 })
    );
    const taillightGeom = this.registerGeometry(new THREE.BoxGeometry(1.68, 0.045, 0.04));
    const taillight = new THREE.Mesh(taillightGeom, taillightMat);
    taillight.position.set(0, 0.65, 2.18);
    this.group.add(taillight);
  }

  private buildRoofSensorRig(materials: PerceptionMaterials): void {
    const rigGroup = new THREE.Group();
    rigGroup.name = 'sensor-rig';
    rigGroup.position.set(0, 1.34, 0.15);

    // 1. Sleek Matte Black Roof Rack / Base Tray
    const rackGeom = this.registerGeometry(new THREE.BoxGeometry(0.65, 0.045, 0.65));
    const rack = new THREE.Mesh(rackGeom, materials.egoTrimMaterial);
    rack.position.y = 0.02;
    rack.castShadow = true;
    rigGroup.add(rack);

    // 2. Silver-Graphite Cylindrical LiDAR Puck (r = 0.11m, h = 0.13m)
    const puckGeom = this.registerGeometry(
      new THREE.CylinderGeometry(0.11, 0.11, 0.13, 18)
    );
    const puck = new THREE.Mesh(puckGeom, materials.sensorPuckMaterial);
    puck.position.y = 0.11;
    puck.castShadow = true;
    rigGroup.add(puck);

    // 3. Cyan Optical Emitter Ring
    const ringGeom = this.registerGeometry(new THREE.TorusGeometry(0.115, 0.012, 8, 24));
    ringGeom.rotateX(Math.PI / 2);
    const ring = new THREE.Mesh(ringGeom, materials.sensorGlowMaterial);
    ring.position.y = 0.11;
    rigGroup.add(ring);

    // Cap on sensor puck
    const capGeom = this.registerGeometry(new THREE.CylinderGeometry(0.09, 0.11, 0.02, 18));
    const cap = new THREE.Mesh(capGeom, materials.egoTrimMaterial);
    cap.position.y = 0.18;
    rigGroup.add(cap);

    this.group.add(rigGroup);
  }

  private buildLidarSamplingField(): THREE.Group {
    const sweep = new THREE.Group();
    sweep.name = 'lidar-sweeps';

    // Thin ice-cyan partial sensor sampling arcs around ego vehicle (forward 120-degree cone)
    const arcRadii = [4.0, 8.5, 14.0, 20.0];
    const arcMaterial = this.registerMaterial(
      new THREE.LineBasicMaterial({
        color: 0x62d3c6,
        transparent: true,
        opacity: 0.24,
      })
    );

    for (const r of arcRadii) {
      const points: THREE.Vector3[] = [];
      const segments = 44;
      const startAngle = -Math.PI * 0.72;
      const endAngle = -Math.PI * 0.28;

      for (let i = 0; i <= segments; i++) {
        const theta = startAngle + (i / segments) * (endAngle - startAngle);
        points.push(new THREE.Vector3(r * Math.cos(theta), 0.035, r * Math.sin(theta)));
      }

      const geom = this.registerGeometry(new THREE.BufferGeometry().setFromPoints(points));
      const line = new THREE.Line(geom, arcMaterial);
      sweep.add(line);
    }

    // Dynamic sweeping beam line
    const sweepLinePoints = [
      new THREE.Vector3(0, 0.035, -0.2),
      new THREE.Vector3(0, 0.035, -18.0),
    ];
    const sweepLineGeom = this.registerGeometry(
      new THREE.BufferGeometry().setFromPoints(sweepLinePoints)
    );
    const sweepLineMat = this.registerMaterial(
      new THREE.LineBasicMaterial({
        color: 0x62d3c6,
        transparent: true,
        opacity: 0.38,
      })
    );
    const sweepLine = new THREE.Line(sweepLineGeom, sweepLineMat);
    sweepLine.name = 'sweep-line';
    sweep.add(sweepLine);

    return sweep;
  }

  public updateAnimation(deltaSeconds: number): void {
    this.sweepAngle += deltaSeconds * 1.6;
    const sweepLine = this.sweepGroup.getObjectByName('sweep-line');
    if (sweepLine) {
      sweepLine.rotation.y = Math.sin(this.sweepAngle) * 0.42;
    }
  }

  public dispose(): void {
    for (const item of this.disposables) {
      item.dispose();
    }
    this.disposables = [];
  }
}
