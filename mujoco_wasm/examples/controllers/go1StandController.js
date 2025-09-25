export class Go1StandController {
  constructor(demo) {
    this.demo = demo;
    this.type = 'go1';
    this.homeQpos = null;
    this.homeCtrl = null;
  }

  onSceneActivated() {
    if (this.demo?.inputManager) {
      this.demo.inputManager.setEnabled(false);
    }
    this.captureHomeTarget();
    this.reset();
  }

  captureHomeTarget() {
    const { model, simulation } = this.demo;
    if (!model || !simulation) {
      this.homeQpos = null;
      this.homeCtrl = null;
      return;
    }

    const nq = model.nq;
    const nu = model.nu;

    if (model.nkey > 0) {
      this.homeQpos = model.key_qpos.slice(0, nq);
      this.homeCtrl = model.key_ctrl.slice(0, nu);
    } else {
      this.homeQpos = simulation.qpos.slice(0, nq);
      this.homeCtrl = simulation.ctrl.slice(0, nu);
    }
  }

  reset() {
    const { simulation } = this.demo;
    if (!simulation) {
      return;
    }

    if (this.homeCtrl && simulation.ctrl?.length >= this.homeCtrl.length) {
      simulation.ctrl.set(this.homeCtrl);
    }
    if (this.homeQpos && simulation.qpos?.length >= this.homeQpos.length) {
      simulation.qpos.set(this.homeQpos);
      simulation.forward();
    }
  }

  update() {
    const { simulation } = this.demo;
    if (!simulation || !this.homeCtrl) {
      return;
    }

    simulation.ctrl.set(this.homeCtrl);
  }

  dispose() {
    if (this.demo?.inputManager) {
      this.demo.inputManager.reset();
      this.demo.inputManager.setEnabled(false);
    }
  }
}
