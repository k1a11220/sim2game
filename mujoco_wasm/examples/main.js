import * as THREE from "three";
import load_mujoco from "../dist/mujoco_wasm.js";
import { OrbitControls } from "./libs/controls/OrbitControls.js";
import { GUI } from "./libs/libs/lil-gui.module.min.js";
import {
  downloadExampleScenesFolder,
  getPosition,
  getQuaternion,
  loadSceneFromURL,
  setupGUI,
  standardNormal,
  toMujocoPos,
} from "./mujocoUtils.js";
import { DragStateManager } from "./utils/DragStateManager.js";
import { InputManager } from "./utils/InputManager.js";

const CONTROL_KEYS = new Set(["Space", "KeyZ", "KeyW", "KeyA", "KeyS", "KeyD"]);
const ROLL_SIGNS = [-1, +1, +1, -1];
const PITCH_SIGNS = [+1, +1, -1, -1];
const ftick = 5;

// Load the MuJoCo Module
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
const initialScene = "skydio_x2/scene.xml";
mujoco.FS.mkdir("/working");
mujoco.FS.mount(mujoco.MEMFS, { root: "." }, "/working");
await downloadExampleScenesFolder(mujoco);

// --- MuJoCo -> three.js frame conversions ---
const MJ2THREE_ADJ = new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(1, 0, 0), -Math.PI / 2 // rotate frame: Z-up -> Y-up
);

// (x, y, z)_mjc  -->  (x, z, -y)_three
function mjcPosToThree(x, y, z) {
  return new THREE.Vector3(x, z, -y);
}

// MuJoCo quat is (w,x,y,z); three wants (x,y,z,w) + frame rotation
function mjcQuatToThree(qw, qx, qy, qz) {
  const q = new THREE.Quaternion(qx, qy, qz, qw);
  q.premultiply(MJ2THREE_ADJ);
  return q;
}

// --- Minimal PID ---
class PID {
  constructor(
    kp = 0,
    ki = 0,
    kd = 0,
    outMin = -Infinity,
    outMax = Infinity,
    iMin = -2,
    iMax = 2
  ) {
    Object.assign(this, { kp, ki, kd, outMin, outMax });
    this.iMin = iMin;
    this.iMax = iMax;
    this.i = 0;
    this.prevErr = 0;
    this.initialized = false;
  }
  reset() {
    this.i = 0;
    this.prevErr = 0;
    this.initialized = false;
  }

  step(setpoint, measurement, dt) {
    const err = setpoint - measurement;
    if (!this.initialized) {
      this.initialized = true;
      this.prevErr = err;
      return this._clamp(this.kp * err);
    }

    // Proportional, Integral (with anti-windup), Derivative
    const p = this.kp * err;
    const d = (this.kd * (err - this.prevErr)) / Math.max(1e-6, dt);

    const iNew = this.i + err * dt;
    let uCand = p + this.ki * iNew + d;

    // Stop integrating when pushing further into saturation
    if ((uCand > this.outMax && err > 0) || (uCand < this.outMin && err < 0)) {
      // keep old integral
      uCand = p + this.ki * this.i + d;
    } else {
      this.i = Math.min(this.iMax, Math.max(this.iMin, iNew));
      uCand = p + this.ki * this.i + d;
    }

    this.prevErr = err;
    return this._clamp(uCand);
  }
  _clamp(x) {
    return Math.min(this.outMax, Math.max(this.outMin, x));
  }
}

const wrapPI = (a) => {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
};

function quatToRPY(qw, qx, qy, qz) {
  const sinr_cosp = 2 * (qw * qx + qy * qz);
  const cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);
  const sinp = 2 * (qw * qy - qz * qx);
  const pitch =
    Math.abs(sinp) >= 1 ? (Math.sign(sinp) * Math.PI) / 2 : Math.asin(sinp);
  const siny_cosp = 2 * (qw * qz + qx * qy);
  const cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);
  return { roll, pitch, yaw };
}

const axisVec = (name) => {
  switch (name) {
    case 'x':  return new THREE.Vector3( 1, 0, 0);
    case '-x': return new THREE.Vector3(-1, 0, 0);
    case 'y':  return new THREE.Vector3( 0, 1, 0);
    case '-y': return new THREE.Vector3( 0,-1, 0);
    case 'z':  return new THREE.Vector3( 0, 0, 1);
    case '-z': return new THREE.Vector3( 0, 0,-1);
    default:   return new THREE.Vector3( 1, 0, 0);
  }
};

export class MuJoCoDemo {
  constructor() {
    console.log("=== CONSTRUCTOR START ===");
    this.mujoco = mujoco;
    this.quatToRPY = quatToRPY;

    // Load in the state from XML
    this.model = new mujoco.Model("/working/" + initialScene);
    this.state = new mujoco.State(this.model);
    this.simulation = new mujoco.Simulation(this.model, this.state);
    console.log("Model loaded successfully");

    // Define Random State Variables
    this.params = {
      scene: initialScene,
      paused: false,
      help: false,
      ctrlnoiserate: 0.0,
      ctrlnoisestd: 0.0,
      keyframeNumber: 0,
      useTungModel: false,  // Toggle for model switching
    };
    this.params.altitudeOnly = false;

    // Camera tuning params from main_new.js
    this.params.trackBack = 1.5;   // meters "behind" the MuJoCo camera
    this.params.trackUp   = 0.2;   // slight upward offset
    this.params.fpvForward = 0.20; // move forward in view direction (meters)
    this.params.fpvUp      = 0.05; // nudge up
    this.params.fpvForwardAxis = 'x'; // 'x' | '-x' | 'y' | '-y' | 'z' | '-z'
    this.params.fpvUpAxis      = 'z'; // body up axis (usually 'z')
    this.params.fpvYawOffsetDeg = 0;   // fine trim if needed

    this.mujoco_time = 0.0;
    (this.bodies = {}), (this.lights = {});
    this.tmpVec = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    this.inputManager = new InputManager({
      allowedKeys: Array.from(CONTROL_KEYS),
      allowWithinSelectors: [".dg"],
    });

    // Defer ctrl array initialization to avoid memory alignment issues
    this.ctrlTargets = null;
    this.thrustIndices = [];
    this.skydioControlConfigured = false;
    this.hoverThrust = 3.2495625;
    this.ascendGain = 1.2;
    this.pitchGain = 0.35;
    this.rollGain = 0.35;
    this.controlSmoothing = 0.25;
    this.thrustRange = { min: 0.0, max: 13.0 };

    this.container = document.createElement("div");
    document.body.appendChild(this.container);

    this.scene = new THREE.Scene();
    this.scene.name = "scene";

    // CREATE CAMERAS
    this.camera = new THREE.PerspectiveCamera(
      45,
      window.innerWidth / window.innerHeight,
      0.001,
      2000
    );
    this.camera.name = "PerspectiveCamera";
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    // Add drone camera from main_new.js
    this.droneCamera = new THREE.PerspectiveCamera(
      75,
      window.innerWidth / window.innerHeight,
      0.01,
      2000
    );
    this.droneCamera.near = 0.001;
    this.droneCamera.updateProjectionMatrix();
    this.droneCamera.name = 'DroneCamera';
    this.scene.add(this.droneCamera);

    // Camera mode management
    this.cameraMode = 'orbit';

    this.scene.background = new THREE.Color(0.9, 0.9, 0.9);

    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.1);
    this.ambientLight.name = "AmbientLight";
    this.scene.add(this.ambientLight);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.setAnimationLoop(this.render.bind(this));

    this.container.appendChild(this.renderer.domElement);

    // Wheel zoom for track/fpv modes
    this.container.addEventListener('wheel', (e) => {
      if (this.cameraMode === 'track') {
        const delta = Math.sign(e.deltaY) * 0.2;
        this.params.trackBack = Math.max(0.1, this.params.trackBack + delta);
      } else if (this.cameraMode === 'fpv') {
        const delta = Math.sign(e.deltaY) * 0.05;
        this.params.fpvForward = Math.max(0.0, this.params.fpvForward + delta);
      }
    }, { passive: true });

    const canvas = this.renderer.domElement;
    if (canvas.tabIndex === undefined || canvas.tabIndex < 0) {
      canvas.tabIndex = 0;
    }
    const focusCanvas = () => canvas.focus();
    canvas.addEventListener("mousedown", focusCanvas);
    canvas.addEventListener("pointerdown", focusCanvas);
    canvas.addEventListener("touchstart", focusCanvas, { passive: true });
    canvas.focus();

    // Create controls after camera and renderer
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    // Add camera switching
    window.addEventListener('keydown', (e) => {
      if (e.key === 'c' || e.key === 'C') {
        this.cycleCameraMode();
      }
    });

    // Camera indicator HUD
    this.cameraIndicator = document.createElement('div');
    this.cameraIndicator.style.position = 'absolute';
    this.cameraIndicator.style.bottom = '10px';
    this.cameraIndicator.style.left = '50%';
    this.cameraIndicator.style.transform = 'translateX(-50%)';
    this.cameraIndicator.style.padding = '10px';
    this.cameraIndicator.style.backgroundColor = 'rgba(0,0,0,0.5)';
    this.cameraIndicator.style.color = 'white';
    this.cameraIndicator.style.fontFamily = 'monospace';
    this.cameraIndicator.style.borderRadius = '5px';
    this.cameraIndicator.textContent = 'Camera: ORBIT (Press C to cycle)';
    this.container.appendChild(this.cameraIndicator);

    window.addEventListener("resize", this.onWindowResize.bind(this));

    // Initialize the Drag State Manager
    this.dragStateManager = new DragStateManager(
      this.scene,
      this.renderer,
      this.camera,
      this.container.parentElement,
      this.controls
    );

    // PID control setup
    this.params.pidEnabled = true;
    console.log("PID initially:", this.params.pidEnabled ? "ENABLED" : "DISABLED");

    this.pidTarget = { roll: 0, pitch: 0, yaw: 0, yawRateCmd: 0, alt: 0.8 };

    this.rollPID = new PID(2.0, 2.0, 2.0, -4, 4);
    this.pitchPID = new PID(2.0, 2.0, 2.0, -4, 4);
    this.yawPID = new PID(2.0, 2.0, 2.0, -3, 3);
    this.altPID = new PID(2.0, 2.0, 2.0, -12, 12);

    this.baseHover = this.hoverThrust;
    console.log("baseHover set to:", this.baseHover);
    console.log("=== CONSTRUCTOR END ===");
  }

  // Camera mode cycling from main_new.js
  cycleCameraMode() {
    const modes = ['orbit', 'track', 'fpv'];
    const currentIndex = modes.indexOf(this.cameraMode);
    this.cameraMode = modes[(currentIndex + 1) % modes.length];

    this.controls.enabled = (this.cameraMode === 'orbit');
    console.log(`Switched to ${this.cameraMode} camera`);
  }

  // Add the cycleCameraMode method here
  cycleCameraMode() {
    const modes = ['orbit', 'track', 'fpv'];
    const currentIndex = modes.indexOf(this.cameraMode);
    this.cameraMode = modes[(currentIndex + 1) % modes.length];
    
    this.controls.enabled = (this.cameraMode === 'orbit');
    console.log(`Switched to ${this.cameraMode} camera`);

  }
  

  async init() {
    console.log("=== INIT START ===");
    // Initialize the three.js Scene using the .xml Model in initialScene
    [this.model, this.state, this.simulation, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, initialScene, this);
    console.log(
      "Scene loaded, bodies:",
      Object.keys(this.bodies).length,
      "lights:",
      Object.keys(this.lights).length
    );

    // Initialize ctrl array after scene load
    if (this.simulation && this.simulation.ctrl) {
      this.ctrlTargets = new Float32Array(this.simulation.ctrl.length);
    }

    this.simulation.forward();

    this.resetCtrlTargets();
    console.log(
      "After resetCtrlTargets - thrustIndices:",
      this.thrustIndices,
      "skydioConfigured:",
      this.skydioControlConfigured
    );

    this.gui = new GUI();
    setupGUI(this);

    // Add model toggle button from main.js
    this.gui.add(this.params, 'useTungModel').name('Use Tung Model').onChange((value) => {
      this.swapModel(value);
    });

    console.log("=== INIT COMPLETE ===");
  }

  // Model swap function from main.js
  swapModel(useTung) {
    const fs = this.mujoco.FS;

    console.log(`Starting model swap: ${useTung ? 'to Tung' : 'to Drone'}`);

    try {
      console.log('=== DEBUGGING FILE SYSTEM ===');

      // Check root working directory
      try {
        console.log('Root working directory contents:', fs.readdir('/working'));
      } catch (e) {
        console.error('Cannot access /working:', e);
      }

      // Try different possible paths
      const possiblePaths = [
        '/working/skydio_x2',
        '/working/skydio_x2/assets',
        '/working',
        '/working/scenes/skydio_x2',
        '/working/scenes/skydio_x2/assets'
      ];

      let actualAssetsPath = null;
      let actualScenePath = null;

      for (const path of possiblePaths) {
        try {
          const contents = fs.readdir(path);
          console.log(`Contents of ${path}:`, contents);

          if (path.includes('assets')) {
            actualAssetsPath = path;
          } else if (path.includes('skydio_x2') && !path.includes('assets')) {
            actualScenePath = path;
          }
        } catch (e) {
          console.log(`Cannot access ${path}:`, e.message);
        }
      }

      if (!actualAssetsPath) {
        console.error('Could not find assets directory');
        return;
      }

      console.log(`Using assets path: ${actualAssetsPath}`);
      console.log(`Using scene path: ${actualScenePath}`);

      // Define file names
      const droneFile = 'X2_lowpoly_original.obj';
      const tungFile = 'tung_nomtl.obj';
      const targetFile = 'X2_lowpoly.obj';

      // 1. Copy the appropriate mesh file
      const sourceFile = useTung ? tungFile : droneFile;
      const fullSourcePath = `${actualAssetsPath}/${sourceFile}`;
      const fullTargetPath = `${actualAssetsPath}/${targetFile}`;

      console.log(`Copying: ${fullSourcePath} -> ${fullTargetPath}`);

      // Check if source file exists first
      try {
        const stat = fs.stat(fullSourcePath);
        console.log(`Source file ${sourceFile} exists, size: ${stat.size}`);
      } catch (statErr) {
        console.error(`Source file ${fullSourcePath} does not exist:`, statErr);
        return;
      }

      const data = fs.readFile(fullSourcePath);
      console.log(`Read ${data.length} bytes from ${sourceFile}`);

      fs.writeFile(fullTargetPath, data);
      console.log('File written successfully');

      // 2. Update XML with appropriate scale (only if scene path found)
      if (actualScenePath) {
        const xmlFile = 'x2.xml';
        const xmlPath = `${actualScenePath}/${xmlFile}`;
        console.log(`Reading XML file: ${xmlPath}`);

        try {
          let xmlContent = new TextDecoder().decode(fs.readFile(xmlPath));
          console.log('XML file read successfully');

          // Replace scale based on model type
          const targetScale = useTung ? '100 100 100' : '0.1 0.1 0.1';
          const currentScalePattern = /<mesh scale="[0-9.]+ [0-9.]+ [0-9.]+"/;
          const newScaleString = `<mesh scale="${targetScale}"`;

          xmlContent = xmlContent.replace(currentScalePattern, newScaleString);
          console.log(`Updated XML scale to: ${targetScale}`);

          // Write updated XML
          fs.writeFile(xmlPath, new TextEncoder().encode(xmlContent));
          console.log('XML file updated successfully');
        } catch (xmlErr) {
          console.warn('Could not update XML scale:', xmlErr);
        }
      }

      // 3. Reload the entire scene
      console.log('Reloading scene...');
      loadSceneFromURL(this.mujoco, this.params.scene, this).then(result => {
        console.log('Scene reloaded successfully');
        [this.model, this.state, this.simulation, this.bodies, this.lights] = result;
        this.simulation.forward();
        if (typeof this.resetCtrlTargets === 'function') {
          this.resetCtrlTargets();
        }
        console.log(`Model successfully swapped to: ${useTung ? 'Tung' : 'Drone'}`);
      }).catch(reloadErr => {
        console.error('Failed to reload scene:', reloadErr);
      });

    } catch (err) {
      console.error('Failed to swap model:', err);
      console.error('Error details:', err.message, err.stack);
    }
  }
  
  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.droneCamera.aspect = window.innerWidth / window.innerHeight;
    this.droneCamera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }
  
  render(timeMS) {
    if (!this.model || !this.state || !this.simulation) {
      console.warn(
        "Render called but missing:",
        !this.model ? "model" : !this.state ? "state" : "simulation"
      );
      return;
    }
    this.controls.update();

    if (!this.params["paused"]) {
      let timestep = this.model.getOptions().timestep;
      if (timeMS - this.mujoco_time > 35.0) {
        this.mujoco_time = timeMS;
      }

      // Add frame counter for debugging
      if (!this.frameCount) this.frameCount = 0;
      this.frameCount++;

      while (this.mujoco_time < timeMS) {

        if (this.params.pidEnabled) {
          // Debug PID flow every 60 frames
          if (this.frameCount % ftick === 0) {
            console.log(
              "PID enabled, calling updateTargetsFromKeys and updatePIDControl"
            );
            const zDbg = this.simulation.qpos?.[2];
            console.log(
              "Current state - z:",
              zDbg,
              "target alt:",
              this.pidTarget.alt
            );
          }
          this.updateTargetsFromKeys(timestep);
          this.updatePIDControl(performance.now());
        } else {
          // Debug manual control every 60 frames
          if (this.frameCount % ftick === 0) {
            console.log("PID disabled, using applySkydioKeyboardControl");
          }
          this.applySkydioKeyboardControl();
        }

        // Jitter the control state with gaussian random noise
        if (this.params["ctrlnoisestd"] > 0.0) {
          let rate = Math.exp(
            -timestep / Math.max(1e-10, this.params["ctrlnoiserate"])
          );
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.simulation.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            this.params["Actuator " + i] = currentCtrl[i];
          }
        }

        // Clear old perturbations, apply new ones.
        for (let i = 0; i < this.simulation.qfrc_applied.length; i++) {
          this.simulation.qfrc_applied[i] = 0.0;
        }
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition(this.simulation.xpos, b, this.bodies[b].position);
              getQuaternion(
                this.simulation.xquat,
                b,
                this.bodies[b].quaternion
              );
              this.bodies[b].updateWorldMatrix();
            }
          }
          let bodyID = dragged.bodyID;
          this.dragStateManager.update();
          let force = toMujocoPos(
            this.dragStateManager.currentWorld
              .clone()
              .sub(this.dragStateManager.worldHit)
              .multiplyScalar(this.model.body_mass[bodyID] * 250)
          );
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          this.simulation.applyForce(
            force.x,
            force.y,
            force.z,
            0,
            0,
            0,
            point.x,
            point.y,
            point.z,
            bodyID
          );
        }

        this.simulation.step();

        this.mujoco_time += timestep * 1000.0;
      }
    } else if (this.params["paused"]) {
      this.dragStateManager.update();
      let dragged = this.dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        let b = dragged.bodyID;
        getPosition(this.simulation.xpos, b, this.tmpVec, false);
        getQuaternion(this.simulation.xquat, b, this.tmpQuat, false);

        let offset = toMujocoPos(
          this.dragStateManager.currentWorld
            .clone()
            .sub(this.dragStateManager.worldHit)
            .multiplyScalar(0.3)
        );
        if (this.model.body_mocapid[b] >= 0) {
          console.log("Trying to move mocap body", b);
          let addr = this.model.body_mocapid[b] * 3;
          let pos = this.simulation.mocap_pos;
          pos[addr + 0] += offset.x;
          pos[addr + 1] += offset.y;
          pos[addr + 2] += offset.z;
        } else {
          let root = this.model.body_rootid[b];
          let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
          let pos = this.simulation.qpos;
          pos[addr + 0] += offset.x;
          pos[addr + 1] += offset.y;
          pos[addr + 2] += offset.z;
        }
      }

      this.simulation.forward();
    }

    // Update body transforms
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition(this.simulation.xpos, b, this.bodies[b].position);
        getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // Update light transforms
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.simulation.light_xpos, l, this.lights[l].position);
        getPosition(this.simulation.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Update tendon transforms
    let numWraps = 0;
    if (this.mujocoRoot && this.mujocoRoot.cylinders) {
      let mat = new THREE.Matrix4();
      for (let t = 0; t < this.model.ntendon; t++) {
        let startW = this.simulation.ten_wrapadr[t];
        let r = this.model.tendon_width[t];
        for (
          let w = startW;
          w < startW + this.simulation.ten_wrapnum[t] - 1;
          w++
        ) {
          let tendonStart = getPosition(
            this.simulation.wrap_xpos,
            w,
            new THREE.Vector3()
          );
          let tendonEnd = getPosition(
            this.simulation.wrap_xpos,
            w + 1,
            new THREE.Vector3()
          );
          let tendonAvg = new THREE.Vector3()
            .addVectors(tendonStart, tendonEnd)
            .multiplyScalar(0.5);

          let validStart = tendonStart.length() > 0.01;
          let validEnd = tendonEnd.length() > 0.01;

          if (validStart) {
            this.mujocoRoot.spheres.setMatrixAt(
              numWraps,
              mat.compose(
                tendonStart,
                new THREE.Quaternion(),
                new THREE.Vector3(r, r, r)
              )
            );
          }
          if (validEnd) {
            this.mujocoRoot.spheres.setMatrixAt(
              numWraps + 1,
              mat.compose(
                tendonEnd,
                new THREE.Quaternion(),
                new THREE.Vector3(r, r, r)
              )
            );
          }
          if (validStart && validEnd) {
            mat.compose(
              tendonAvg,
              new THREE.Quaternion().setFromUnitVectors(
                new THREE.Vector3(0, 1, 0),
                tendonEnd.clone().sub(tendonStart).normalize()
              ),
              new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r)
            );
            this.mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
            numWraps++;
          }
        }
      }
      this.mujocoRoot.cylinders.count = numWraps;
      this.mujocoRoot.spheres.count = numWraps > 0 ? numWraps + 1 : 0;
      this.mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
      this.mujocoRoot.spheres.instanceMatrix.needsUpdate = true;
    }

    // Camera update logic from main_new.js
    let activeCamera = this.camera;

    // Debug logging helper
    const logCameraDebug = () => {
      if (this.frameCount % 30 === 0) {
        console.log('=== CAMERA DEBUG ===');
        console.log(`Mode: ${this.cameraMode}`);

        if (this.simulation.qpos && this.simulation.qpos.length >= 7) {
          const qpos = this.simulation.qpos;
          const p = mjcPosToThree(qpos[0], qpos[1], qpos[2]);
          const { roll, pitch, yaw } = this.quatToRPY(qpos[3], qpos[4], qpos[5], qpos[6]);
          console.log('Drone Position:', {
            x: p.x.toFixed(2), y: p.y.toFixed(2), z: p.z.toFixed(2)
          });
          console.log('Drone Rotation (deg):', {
            roll: (roll * 180/Math.PI).toFixed(1),
            pitch: (pitch * 180/Math.PI).toFixed(1),
            yaw: (yaw * 180/Math.PI).toFixed(1)
          });
        }

        console.log('Camera Position:', {
          x: this.droneCamera.position.x.toFixed(2),
          y: this.droneCamera.position.y.toFixed(2),
          z: this.droneCamera.position.z.toFixed(2)
        });
      }
    };

    // --- TRACK MODE ---
    if (this.cameraMode === 'track' && this.simulation?.cam_xpos) {
      const camIdx = 0;
      const addr = camIdx * 3;

      // Base position (converted)
      const p = this.simulation.cam_xpos;
      const camP = mjcPosToThree(p[addr+0], p[addr+1], p[addr+2]);

      // Orientation (prefer cam_xmat)
      let qCam = null;
      if (this.simulation.cam_xmat) {
        const m = this.simulation.cam_xmat;
        const a = camIdx * 9;
        const M = new THREE.Matrix4().set(
          m[a+0], m[a+1], m[a+2], 0,
          m[a+3], m[a+4], m[a+5], 0,
          m[a+6], m[a+7], m[a+8], 0,
          0,      0,      0,      1
        );
        qCam = new THREE.Quaternion().setFromRotationMatrix(M);
        qCam.premultiply(MJ2THREE_ADJ);
        this.droneCamera.quaternion.copy(qCam);
      }

      // Apply a local-space offset
      const qForAxes = qCam ?? this.droneCamera.quaternion;
      const back = new THREE.Vector3(0, 0, 1).applyQuaternion(qForAxes).multiplyScalar(this.params.trackBack);
      const up   = new THREE.Vector3(0, 1, 0).applyQuaternion(qForAxes).multiplyScalar(this.params.trackUp);

      this.droneCamera.position.copy(camP).add(back).add(up);

      // Fallback look-at if no orientation matrix was available
      if (!qCam && this.simulation.qpos?.length >= 3) {
        const look = mjcPosToThree(this.simulation.qpos[0], this.simulation.qpos[1], this.simulation.qpos[2]);
        this.droneCamera.lookAt(look);
      }

      activeCamera = this.droneCamera;
      logCameraDebug();
    }
    // --- FPV MODE ---
    else if (this.cameraMode === 'fpv' && this.simulation?.qpos?.length >= 7) {
      const qpos  = this.simulation.qpos;
      const pos   = mjcPosToThree(qpos[0], qpos[1], qpos[2]);
      const qBody = mjcQuatToThree(qpos[3], qpos[4], qpos[5], qpos[6]);

      // 1) world forward (nose) & up from body axes
      let f = axisVec(this.params.fpvForwardAxis).applyQuaternion(qBody).normalize();
      let u = axisVec(this.params.fpvUpAxis).applyQuaternion(qBody).normalize();

      // 2) optional yaw trim around body-up
      const yawOff = THREE.MathUtils.degToRad(this.params.fpvYawOffsetDeg || 0);
      if (yawOff !== 0) {
        const qYaw = new THREE.Quaternion().setFromAxisAngle(u, yawOff);
        f.applyQuaternion(qYaw).normalize();
      }

      // 3) build a proper RIGHT-HANDED basis
      const r = new THREE.Vector3().crossVectors(f, u).normalize();
      u = new THREE.Vector3().crossVectors(r, f).normalize();

      // 4) three.js cameras look down -Z; make -Z = forward
      const M = new THREE.Matrix4().makeBasis(r, u, f.clone().multiplyScalar(-1));
      this.droneCamera.quaternion.setFromRotationMatrix(M);

      // 5) move the camera to the drone's NOSE
      this.droneCamera.position.copy(pos)
        .add(f.clone().multiplyScalar(this.params.fpvForward))
        .add(u.clone().multiplyScalar(this.params.fpvUp));

      activeCamera = this.droneCamera;
    }

    // HUD + single render
    this.cameraIndicator.textContent = `Camera: ${this.cameraMode.toUpperCase()} (Press C to cycle)`;
    this.renderer.render(this.scene, activeCamera);
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
    return (
      typeof this.params.scene === "string" &&
      this.params.scene.includes("skydio_x2")
    );
  }

  configureSkydioActuators() {
    console.log("=== configureSkydioActuators START ===");
    if (!this.simulation || !this.simulation.ctrl) {
      console.error("No simulation or ctrl!");
      return false;
    }
    const ctrl = this.simulation.ctrl;
    console.log("ctrl length:", ctrl.length, "ctrl values:", Array.from(ctrl));

    if (ctrl.length < 4) {
      console.error("Not enough actuators:", ctrl.length);
      return false;
    }

    this.thrustIndices = [0, 1, 2, 3];
    if (
      this.model &&
      this.model.actuator_ctrlrange &&
      this.model.actuator_ctrlrange.length >= 8
    ) {
      const ranges = this.model.actuator_ctrlrange;
      console.log("Actuator ranges:", Array.from(ranges));
      let min = Number.POSITIVE_INFINITY;
      let max = Number.NEGATIVE_INFINITY;
      for (let i = 0; i < this.thrustIndices.length; i++) {
        const idx = this.thrustIndices[i];
        const lo = ranges[idx * 2 + 0];
        const hi = ranges[idx * 2 + 1];
        if (Number.isFinite(lo)) {
          min = Math.min(min, lo);
        }
        if (Number.isFinite(hi)) {
          max = Math.max(max, hi);
        }
      }
      if (min < max) {
        this.thrustRange.min = min;
        this.thrustRange.max = max;
        console.log("Thrust range set:", this.thrustRange);
      }
    }

    // Only create ctrlTargets if not already initialized
    if (!this.ctrlTargets) {
      this.ctrlTargets = new Float32Array(this.thrustIndices.length);
    }
    this.ctrlTargets.fill(this.hoverThrust);
    for (let i = 0; i < this.thrustIndices.length; i++) {
      ctrl[this.thrustIndices[i]] = this.hoverThrust;
    }
    console.log(
      "Set initial thrust to:",
      this.hoverThrust,
      "on indices:",
      this.thrustIndices
    );

    this.skydioControlConfigured = true;
    console.log("=== configureSkydioActuators SUCCESS ===");
    return true;
  }

  applySkydioKeyboardControl() {
    const manager = this.inputManager;
    const active = this.shouldApplySkydioControl();
    if (manager) {
      manager.setEnabled(active);
    }
    if (!active) {
      return;
    }
    if (!this.skydioControlConfigured && !this.configureSkydioActuators()) {
      return;
    }

    const ascendInput =
      (manager?.isPressed("Space") ? 1 : 0) -
      (manager?.isPressed("KeyZ") ? 1 : 0);
    const pitchInput =
      (manager?.isPressed("KeyS") ? 1 : 0) -
      (manager?.isPressed("KeyW") ? 1 : 0);
    const rollInput =
      (manager?.isPressed("KeyD") ? 1 : 0) -
      (manager?.isPressed("KeyA") ? 1 : 0);

    const ascendOffset = ascendInput * this.ascendGain;
    const pitchOffset = pitchInput * this.pitchGain;
    const rollOffset = rollInput * this.rollGain;

    for (let i = 0; i < this.thrustIndices.length; i++) {
      const target =
        this.hoverThrust +
        ascendOffset +
        pitchOffset * PITCH_SIGNS[i] +
        rollOffset * ROLL_SIGNS[i];
      const clamped = Math.min(
        this.thrustRange.max,
        Math.max(this.thrustRange.min, target)
      );
      this.ctrlTargets[i] = clamped;
    }

    const ctrl = this.simulation.ctrl;
    for (let i = 0; i < this.thrustIndices.length; i++) {
      const idx = this.thrustIndices[i];
      const current = ctrl[idx];
      const desired = this.ctrlTargets[i];
      const next = current + (desired - current) * this.controlSmoothing;
      ctrl[idx] = Math.min(
        this.thrustRange.max,
        Math.max(this.thrustRange.min, next)
      );
    }
  }

  updatePIDControl(nowMs) {
    if (!this.shouldApplySkydioControl()) {
      console.log("Not Skydio scene, skipping PID");
      return;
    }

    if (!this.skydioControlConfigured && !this.configureSkydioActuators()) {
      console.warn("Failed to configure Skydio actuators");
      return;
    }

    let qpos = this.simulation.qpos;

    if (!qpos || qpos.length < 7) {
      console.log(qpos);
      console.error("Invalid qpos!", qpos);
      return;
    }
    const z = qpos[2] || 0;
    const qw = qpos[3] || 1;
    const qx = qpos[4] || 0;
    const qy = qpos[5] || 0;
    const qz = qpos[6] || 0;

    const qNorm = Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (Math.abs(qNorm - 1.0) > 0.1) {
      console.warn("Quaternion not normalized:", qNorm, { qw, qx, qy, qz });
    }

    const { roll, pitch, yaw } = quatToRPY(qw, qx, qy, qz);

    if (
      !Number.isFinite(roll) ||
      !Number.isFinite(pitch) ||
      !Number.isFinite(yaw)
    ) {
      console.error("NaN in angles!", { roll, pitch, yaw, qw, qx, qy, qz });
      return;
    }

    if (this.frameCount && this.frameCount % ftick === 0) {
      console.log("PID State:", {
        z: z.toFixed(3),
        roll: ((roll * 180) / Math.PI).toFixed(1),
        pitch: ((pitch * 180) / Math.PI).toFixed(1),
        yaw: ((yaw * 180) / Math.PI).toFixed(1),
        quat: {
          qw: qw.toFixed(3),
          qx: qx.toFixed(3),
          qy: qy.toFixed(3),
          qz: qz.toFixed(3),
        },
        targets: {
          alt: this.pidTarget.alt,
          roll: ((this.pidTarget.roll * 180) / Math.PI).toFixed(1),
          pitch: ((this.pidTarget.pitch * 180) / Math.PI).toFixed(1),
        },
      });
    }

    const simdt = this.model.getOptions().timestep;

    this.pidTarget.yaw = wrapPI(
      this.pidTarget.yaw + this.pidTarget.yawRateCmd * simdt
    );

    let u_alt, u_roll, u_pitch, u_yaw;

    if (this.params.altitudeOnly) {
      u_alt = 0.0;

      const Kp_roll = 2.0;
      const Kp_pitch = 2.0;
      const Kp_yaw = 1.0;

      const rollErr = this.pidTarget.roll - roll;
      const pitchErr = this.pidTarget.pitch - pitch;
      const yawErr = wrapPI(this.pidTarget.yaw - yaw);

      u_roll = Kp_roll * rollErr;
      u_pitch = Kp_pitch * pitchErr;
      u_yaw = Kp_yaw * -yawErr;

      u_roll = Math.max(-4, Math.min(4, u_roll));
      u_pitch = Math.max(-4, Math.min(4, u_pitch));
      u_yaw = Math.max(-3, Math.min(3, u_yaw));
    } else {
      const qv = this.simulation.qvel || [];
      const wx = qv[3] || 0;
      const wy = qv[4] || 0;
      u_alt = this.altPID.step(this.pidTarget.alt, z, simdt);
      const yawErr = wrapPI(this.pidTarget.yaw - yaw);
      u_roll = this.rollPID.step(this.pidTarget.roll, roll, simdt) + 0.5 * wx;
      u_pitch =
        this.pitchPID.step(this.pidTarget.pitch, pitch, simdt) + 0.5 * wy;
      u_yaw = this.yawPID.step(0.0, -yawErr, simdt);
    }

    if (!Number.isFinite(u_alt)) {
      console.error("NaN in u_alt!", { target: this.pidTarget.alt, z, nowMs });
      return;
    }

    if (
      !Number.isFinite(u_roll) ||
      !Number.isFinite(u_pitch) ||
      !Number.isFinite(u_yaw)
    ) {
      console.error("NaN in attitude control!", {
        u_roll,
        u_pitch,
        u_yaw,
        roll,
        pitch,
        yaw,
      });
      return;
    }

    const T = this.baseHover + u_alt;

    if (!Number.isFinite(T)) {
      console.error("NaN in T!", { baseHover: this.baseHover, u_alt });
      return;
    }
    const desired = new Float32Array(4);
    const YAW_SIGNS = [-1, +1, -1, +1];

    for (let i = 0; i < 4; i++) {
      desired[i] =
        T +
        u_roll * ROLL_SIGNS[i] +
        u_pitch * PITCH_SIGNS[i] +
        u_yaw * YAW_SIGNS[i];
    }

    if (this.frameCount && this.frameCount % ftick === 0) {
      console.log("PID Control:", {
        u_alt: u_alt.toFixed(3),
        u_roll: u_roll.toFixed(3),
        u_pitch: u_pitch.toFixed(3),
        u_yaw: u_yaw.toFixed(3),
        T: T.toFixed(3),
        baseHover: this.baseHover,
        motors: desired.map((d) => d.toFixed(2)),
      });

      console.log("PID Internals:", {
        alt_i: this.altPID.i.toFixed(3),
        roll_i: this.rollPID.i.toFixed(3),
        pitch_i: this.pitchPID.i.toFixed(3),
        yaw_i: this.yawPID.i.toFixed(3),
      });
    }

    for (let i = 0; i < 4; i++) {
      if (!Number.isFinite(desired[i])) {
        console.error("NaN in motor", i, {
          desired,
          T,
          u_alt,
          u_roll,
          u_pitch,
          u_yaw,
        });
        return;
      }
    }

    const ctrl = this.simulation.ctrl;

    if (!ctrl || ctrl.length < 4) {
      console.error("Invalid ctrl array!", ctrl);
      return;
    }

    for (let i = 0; i < 4 && i < this.thrustIndices.length; i++) {
      const idx = this.thrustIndices[i];
      const current = ctrl[idx] || 0;
      const next = current + (desired[i] - current) * this.controlSmoothing;

      if (this.frameCount && this.frameCount % ftick === 0 && i === 0) {
        console.log(
          `Motor ${i}: current=${current.toFixed(2)}, desired=${desired[
            i
          ].toFixed(2)}, next=${next.toFixed(2)}`
        );
      }

      ctrl[idx] = Math.min(
        this.thrustRange.max,
        Math.max(this.thrustRange.min, next)
      );
    }

    if (this.frameCount && this.frameCount % ftick === 0) {
      console.log(
        "Final ctrl:",
        Array.from(ctrl).map((c) => c.toFixed(2))
      );
      console.log("Thrust range:", this.thrustRange);
    }
  }

  updateTargetsFromKeys(dt) {
    const m = this.inputManager;
    if (!m) return;

    const ascend =
      (m.isPressed("Space") ? 1 : 0) - (m.isPressed("KeyZ") ? 1 : 0);
    this.pidTarget.alt = Math.max(0.0, this.pidTarget.alt + ascend * 0.6 * dt);

    const pitchCmd =
      (m.isPressed("KeyS") ? 1 : 0) - (m.isPressed("KeyW") ? 1 : 0);
    if (pitchCmd !== 0) {
      this.pidTarget.pitch = THREE.MathUtils.clamp(
        this.pidTarget.pitch + -pitchCmd * ((12 * Math.PI) / 180) * dt,
        (-20 * Math.PI) / 180,
        (20 * Math.PI) / 180
      );
    } else {
      const decayRate = 3.0;
      this.pidTarget.pitch *= Math.exp(-decayRate * dt);

      if (Math.abs(this.pidTarget.pitch) < 0.01) {
        this.pidTarget.pitch = 0;
      }
      if (Math.abs(this.pidTarget.pitch) < 0.1) {
        this.pitchPID.i *= 0.9;
      }
    }

    const rollCmd =
      (m.isPressed("KeyD") ? 1 : 0) - (m.isPressed("KeyA") ? 1 : 0);
    if (rollCmd !== 0) {
      this.pidTarget.roll = THREE.MathUtils.clamp(
        this.pidTarget.roll + rollCmd * ((12 * Math.PI) / 180) * dt,
        (-20 * Math.PI) / 180,
        (20 * Math.PI) / 180
      );
    } else {
      const decayRate = 3.0;
      this.pidTarget.roll *= Math.exp(-decayRate * dt);

      if (Math.abs(this.pidTarget.roll) < 0.01) {
        this.pidTarget.roll = 0;
      }
      if (Math.abs(this.pidTarget.roll) < 0.1) {
        this.rollPID.i *= 0.9;
      }
    }
  }
}

let demo = new MuJoCoDemo();
await demo.init();