import { SkydioPidController } from './skydioPidController.js';
import { Go1StandController } from './go1StandController.js';

/**
 * Select an appropriate controller for the requested scene.
 * @param {string} sceneName
 * @param {import('../main.js').MuJoCoDemo} demo
 */
export function createControllerForScene(sceneName, demo) {
  if (typeof sceneName !== 'string') {
    return null;
  }

  if (sceneName.includes('skydio_x2')) {
    return new SkydioPidController(demo);
  }

  if (sceneName.includes('unitree_go1')) {
    return new Go1StandController(demo);
  }

  return null;
}
