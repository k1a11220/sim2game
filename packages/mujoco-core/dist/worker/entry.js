import { loadMujocoModule } from '../runtime/loadMujoco.js';
import { ensureWorkingDirectory, preloadFiles, DEFAULT_WORKING_DIR } from '../fs.js';
let moduleInstance = null;
async function handleInitMessage(payload, requestId) {
    const options = payload.options ?? {};
    moduleInstance = await loadMujocoModule(options);
    const workingDirectory = options.workingDirectory ?? DEFAULT_WORKING_DIR;
    if (options.mountWorkingFilesystem !== false) {
        ensureWorkingDirectory(moduleInstance, workingDirectory);
    }
    if (options.preloadFiles?.length) {
        await preloadFiles(moduleInstance, options.preloadFiles, workingDirectory);
    }
    self.postMessage({ type: 'ready', requestId });
}
self.addEventListener('message', (event) => {
    const { type, payload = {}, requestId } = event.data;
    if (type === 'init') {
        handleInitMessage(payload, requestId).catch((error) => {
            const message = error instanceof Error ? error.message : String(error);
            self.postMessage({ type: 'error', requestId, error: message });
        });
        return;
    }
    const errorMessage = moduleInstance
        ? `Unsupported worker message type: ${type}`
        : 'MuJoCo worker is not initialised yet. Call init first.';
    self.postMessage({ type: 'error', requestId, error: errorMessage });
});
//# sourceMappingURL=entry.js.map