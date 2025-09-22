import { loadMujocoModule } from '../runtime/loadMujoco.js';
import { ensureWorkingDirectory, preloadFiles, DEFAULT_WORKING_DIR } from '../fs.js';
import type { CreateOptions, MujocoModule, WorkerMessage } from '../types.js';

let moduleInstance: MujocoModule | null = null;

async function handleInitMessage(payload: { options?: CreateOptions }, requestId?: number) {
  const options = payload.options ?? {};
  moduleInstance = await loadMujocoModule(options);

  const workingDirectory = options.workingDirectory ?? DEFAULT_WORKING_DIR;
  if (options.mountWorkingFilesystem !== false) {
    ensureWorkingDirectory(moduleInstance, workingDirectory);
  }
  if (options.preloadFiles?.length) {
    await preloadFiles(moduleInstance, options.preloadFiles, workingDirectory);
  }

  self.postMessage({ type: 'ready', requestId } satisfies WorkerMessage);
}

self.addEventListener('message', (event: MessageEvent<WorkerMessage<{ options?: CreateOptions }>>) => {
  const { type, payload = {}, requestId } = event.data;

  if (type === 'init') {
    handleInitMessage(payload, requestId).catch((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      self.postMessage({ type: 'error', requestId, error: message } satisfies WorkerMessage);
    });
    return;
  }

  const errorMessage = moduleInstance
    ? `Unsupported worker message type: ${type}`
    : 'MuJoCo worker is not initialised yet. Call init first.';

  self.postMessage({ type: 'error', requestId, error: errorMessage } satisfies WorkerMessage);
});

export {};
