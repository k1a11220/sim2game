
import * as THREE           from 'three';
import { GUI              } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls    } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { InputManager } from './utils/InputManager.js';
import { setupGUI, downloadExampleScenesFolder, loadSceneFromURL, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import   load_mujoco        from '../dist/mujoco_wasm.js';

const CONTROL_KEYS = new Set(['Space', 'KeyZ', 'KeyW', 'KeyA', 'KeyS', 'KeyD']);
const PITCH_SIGNS = [1, 1, -1, -1];
const ROLL_SIGNS  = [1, -1, -1, 1];

// Load the MuJoCo Module
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
const initialScene = "skydio_x2/scene.xml";
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
await downloadExampleScenesFolder(mujoco);

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    // Load in the state from XML
    this.model      = new mujoco.Model("/working/" + initialScene);
    this.state      = new mujoco.State(this.model);
    this.simulation = new mujoco.Simulation(this.model, this.state);

    // Define Random State Variables
    this.params = { scene: initialScene, paused: false, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, keyframeNumber: 0 };
    this.mujoco_time = 0.0;
    this.bodies  = {}, this.lights = {};
    this.tmpVec  = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];
    this.inputManager = new InputManager({
      allowedKeys: Array.from(CONTROL_KEYS),
      allowWithinSelectors: ['.dg']
    });

    this.ctrlTargets = new Float32Array(this.simulation.ctrl.length);
    this.thrustIndices = [];
    this.skydioControlConfigured = false;
    this.hoverThrust = 3.2495625;
    this.ascendGain = 1.2;
    this.pitchGain = 0.35;
    this.rollGain = 0.35;
    this.controlSmoothing = 0.25;
    this.thrustRange = { min: 0.0, max: 13.0 };

    this.container = document.createElement( 'div' );
    document.body.appendChild( this.container );

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5 );

    this.ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 );
    this.ambientLight.name = 'AmbientLight';
    this.scene.add( this.ambientLight );

    this.renderer = new THREE.WebGLRenderer( { antialias: true } );
    this.renderer.setPixelRatio( window.devicePixelRatio );
    this.renderer.setSize( window.innerWidth, window.innerHeight );
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
    this.renderer.setAnimationLoop( this.render.bind(this) );

    this.container.appendChild( this.renderer.domElement );

    const canvas = this.renderer.domElement;
    if (canvas.tabIndex === undefined || canvas.tabIndex < 0) {
      canvas.tabIndex = 0;
    }
    const focusCanvas = () => canvas.focus();
    canvas.addEventListener('mousedown', focusCanvas);
    canvas.addEventListener('pointerdown', focusCanvas);
    canvas.addEventListener('touchstart', focusCanvas, { passive: true });
    canvas.focus();

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', this.onWindowResize.bind(this));

    // Initialize the Drag State Manager.
    this.dragStateManager = new DragStateManager(this.scene, this.renderer, this.camera, this.container.parentElement, this.controls);
    this.resetCtrlTargets();
  }

  async init() {
    // Initialize the three.js Scene using the .xml Model in initialScene
    [this.model, this.state, this.simulation, this.bodies, this.lights] =  
      await loadSceneFromURL(mujoco, initialScene, this);
    this.resetCtrlTargets();

    this.gui = new GUI();
    setupGUI(this);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize( window.innerWidth, window.innerHeight );
  }

  render(timeMS) {
    if (!this.model || !this.state || !this.simulation) {
      return;
    }
    this.controls.update();

    if (!this.params["paused"]) {
      let timestep = this.model.getOptions().timestep;
      if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS; }
      while (this.mujoco_time < timeMS) {
        this.applySkydioKeyboardControl();

        // Jitter the control state with gaussian random noise
        if (this.params["ctrlnoisestd"] > 0.0) {
          let rate  = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.simulation.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            this.params["Actuator " + i] = currentCtrl[i];
          }
        }

        // Clear old perturbations, apply new ones.
        for (let i = 0; i < this.simulation.qfrc_applied.length; i++) { this.simulation.qfrc_applied[i] = 0.0; }
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition  (this.simulation.xpos , b, this.bodies[b].position);
              getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }
          let bodyID = dragged.bodyID;
          this.dragStateManager.update(); // Update the world-space force origin
          let force = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass[bodyID] * 250));
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID);

          // TODO: Apply pose perturbations (mocap bodies only).
        }

        this.simulation.step();

        this.mujoco_time += timestep * 1000.0;
      }

    } else if (this.params["paused"]) {
      this.dragStateManager.update(); // Update the world-space force origin
      let dragged = this.dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        let b = dragged.bodyID;
        getPosition  (this.simulation.xpos , b, this.tmpVec , false); // Get raw coordinate from MuJoCo
        getQuaternion(this.simulation.xquat, b, this.tmpQuat, false); // Get raw coordinate from MuJoCo

        let offset = toMujocoPos(this.dragStateManager.currentWorld.clone()
          .sub(this.dragStateManager.worldHit).multiplyScalar(0.3));
        if (this.model.body_mocapid[b] >= 0) {
          // Set the root body's mocap position...
          console.log("Trying to move mocap body", b);
          let addr = this.model.body_mocapid[b] * 3;
          let pos  = this.simulation.mocap_pos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;
        } else {
          // Set the root body's position directly...
          let root = this.model.body_rootid[b];
          let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
          let pos  = this.simulation.qpos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;

          //// Save the original root body position
          //let x  = pos[addr + 0], y  = pos[addr + 1], z  = pos[addr + 2];
          //let xq = pos[addr + 3], yq = pos[addr + 4], zq = pos[addr + 5], wq = pos[addr + 6];

          //// Clear old perturbations, apply new ones.
          //for (let i = 0; i < this.simulation.qfrc_applied().length; i++) { this.simulation.qfrc_applied()[i] = 0.0; }
          //for (let bi = 0; bi < this.model.nbody(); bi++) {
          //  if (this.bodies[b]) {
          //    getPosition  (this.simulation.xpos (), bi, this.bodies[bi].position);
          //    getQuaternion(this.simulation.xquat(), bi, this.bodies[bi].quaternion);
          //    this.bodies[bi].updateWorldMatrix();
          //  }
          //}
          ////dragStateManager.update(); // Update the world-space force origin
          //let force = toMujocoPos(this.dragStateManager.currentWorld.clone()
          //  .sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass()[b] * 0.01));
          //let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          //// This force is dumped into xrfc_applied
          //this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, b);
          //this.simulation.integratePos(this.simulation.qpos(), this.simulation.qfrc_applied(), 1);

          //// Add extra drag to the root body
          //pos[addr + 0] = x  + (pos[addr + 0] - x ) * 0.1;
          //pos[addr + 1] = y  + (pos[addr + 1] - y ) * 0.1;
          //pos[addr + 2] = z  + (pos[addr + 2] - z ) * 0.1;
          //pos[addr + 3] = xq + (pos[addr + 3] - xq) * 0.1;
          //pos[addr + 4] = yq + (pos[addr + 4] - yq) * 0.1;
          //pos[addr + 5] = zq + (pos[addr + 5] - zq) * 0.1;
          //pos[addr + 6] = wq + (pos[addr + 6] - wq) * 0.1;


        }
      }

      this.simulation.forward();
    }

    // Update body transforms.
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition  (this.simulation.xpos , b, this.bodies[b].position);
        getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // Update light transforms.
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.simulation.light_xpos, l, this.lights[l].position);
        getPosition(this.simulation.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Update tendon transforms.
    let numWraps = 0;
    if (this.mujocoRoot && this.mujocoRoot.cylinders) {
      let mat = new THREE.Matrix4();
      for (let t = 0; t < this.model.ntendon; t++) {
        let startW = this.simulation.ten_wrapadr[t];
        let r = this.model.tendon_width[t];
        for (let w = startW; w < startW + this.simulation.ten_wrapnum[t] -1 ; w++) {
          let tendonStart = getPosition(this.simulation.wrap_xpos, w    , new THREE.Vector3());
          let tendonEnd   = getPosition(this.simulation.wrap_xpos, w + 1, new THREE.Vector3());
          let tendonAvg   = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);

          let validStart = tendonStart.length() > 0.01;
          let validEnd   = tendonEnd  .length() > 0.01;

          if (validStart) { this.mujocoRoot.spheres.setMatrixAt(numWraps    , mat.compose(tendonStart, new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
          if (validEnd  ) { this.mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd  , new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
          if (validStart && validEnd) {
            mat.compose(tendonAvg, new THREE.Quaternion().setFromUnitVectors(
              new THREE.Vector3(0, 1, 0), tendonEnd.clone().sub(tendonStart).normalize()),
              new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r));
            this.mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
            numWraps++;
          }
        }
      }
      this.mujocoRoot.cylinders.count = numWraps;
      this.mujocoRoot.spheres  .count = numWraps > 0 ? numWraps + 1: 0;
      this.mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
      this.mujocoRoot.spheres  .instanceMatrix.needsUpdate = true;
    }

    // Render!
    this.renderer.render( this.scene, this.camera );
  }

  resetCtrlTargets() {
    if (this.simulation && this.simulation.ctrl) {
      this.ctrlTargets = new Float32Array(this.simulation.ctrl.length);
    } else {
      this.ctrlTargets = new Float32Array(0);
    }
    this.thrustIndices = [];
    this.skydioControlConfigured = false;
    this.thrustRange = { min: 0.0, max: 13.0 };
    if (this.inputManager) {
      this.inputManager.reset();
    }
  }

  shouldApplySkydioControl() {
    return typeof this.params.scene === 'string' && this.params.scene.includes('skydio_x2');
  }

  configureSkydioActuators() {
    if (!this.simulation || !this.simulation.ctrl) { return false; }
    const ctrl = this.simulation.ctrl;
    if (ctrl.length < 4) { return false; }

    this.thrustIndices = [0, 1, 2, 3];
    if (this.model && this.model.actuator_ctrlrange && this.model.actuator_ctrlrange.length >= 8) {
      const ranges = this.model.actuator_ctrlrange;
      let min = Number.POSITIVE_INFINITY;
      let max = Number.NEGATIVE_INFINITY;
      for (let i = 0; i < this.thrustIndices.length; i++) {
        const idx = this.thrustIndices[i];
        const lo = ranges[(idx * 2) + 0];
        const hi = ranges[(idx * 2) + 1];
        if (Number.isFinite(lo)) { min = Math.min(min, lo); }
        if (Number.isFinite(hi)) { max = Math.max(max, hi); }
      }
      if (min < max) {
        this.thrustRange.min = min;
        this.thrustRange.max = max;
      }
    }

    this.ctrlTargets = new Float32Array(this.thrustIndices.length);
    this.ctrlTargets.fill(this.hoverThrust);
    for (let i = 0; i < this.thrustIndices.length; i++) {
      ctrl[this.thrustIndices[i]] = this.hoverThrust;
    }

    this.skydioControlConfigured = true;
    return true;
  }

  applySkydioKeyboardControl() {
    const manager = this.inputManager;
    const active = this.shouldApplySkydioControl();
    if (manager) {
      manager.setEnabled(active);
    }
    if (!active) { return; }
    if (!this.skydioControlConfigured && !this.configureSkydioActuators()) { return; }

    const ascendInput = (manager?.isPressed('Space') ? 1 : 0) - (manager?.isPressed('KeyZ') ? 1 : 0);
    const pitchInput  = (manager?.isPressed('KeyW')  ? 1 : 0) - (manager?.isPressed('KeyS')  ? 1 : 0);
    const rollInput   = (manager?.isPressed('KeyD')  ? 1 : 0) - (manager?.isPressed('KeyA')  ? 1 : 0);

    const ascendOffset = ascendInput * this.ascendGain;
    const pitchOffset  = pitchInput  * this.pitchGain;
    const rollOffset   = rollInput   * this.rollGain;

    for (let i = 0; i < this.thrustIndices.length; i++) {
      const target = this.hoverThrust + ascendOffset + (pitchOffset * PITCH_SIGNS[i]) + (rollOffset * ROLL_SIGNS[i]);
      const clamped = Math.min(this.thrustRange.max, Math.max(this.thrustRange.min, target));
      this.ctrlTargets[i] = clamped;
    }

    const ctrl = this.simulation.ctrl;
    for (let i = 0; i < this.thrustIndices.length; i++) {
      const idx = this.thrustIndices[i];
      const current = ctrl[idx];
      const desired = this.ctrlTargets[i];
      const next = current + (desired - current) * this.controlSmoothing;
      ctrl[idx] = Math.min(this.thrustRange.max, Math.max(this.thrustRange.min, next));
    }
  }

}

let demo = new MuJoCoDemo();
await demo.init();
