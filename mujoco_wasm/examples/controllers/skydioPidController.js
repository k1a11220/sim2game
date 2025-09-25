export class SkydioPidController {
  constructor(demo) {
    this.demo = demo;
    this.type = 'skydio';
  }

  onSceneActivated() {
    if (this.demo.inputManager) {
      this.demo.inputManager.setEnabled(true);
    }
    this.demo.resetSkydioControlTargets();
  }

  reset() {
    this.demo.resetSkydioControlTargets();
  }

  /**
   * Advance the Skydio PID or manual control for the current step.
   * @param {number} dt - MuJoCo timestep (seconds)
   * @param {number} nowMs - performance.now() timestamp
   */
  update(dt, nowMs) {
    const demo = this.demo;
    if (!demo || !demo.simulation) {
      return;
    }

    if (demo.params.pidEnabled) {
      demo.updateTargetsFromKeys(dt);
      demo.updatePIDControl(nowMs);
    } else {
      demo.applySkydioKeyboardControl();
    }
  }

  dispose() {
    if (this.demo?.inputManager) {
      this.demo.inputManager.setEnabled(false);
      this.demo.inputManager.reset();
    }
  }
}
