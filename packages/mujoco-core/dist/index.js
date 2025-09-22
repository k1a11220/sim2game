export { ensureWorkingDirectory, preloadFiles, DEFAULT_WORKING_DIR as DEFAULT_WORKING_DIRECTORY } from './fs.js';
import { ensureWorkingDirectory, preloadFiles, DEFAULT_WORKING_DIR } from './fs.js';
import { loadMujocoModule } from './runtime/loadMujoco.js';
export async function createMujocoModule(options = {}) {
    const module = await loadMujocoModule(options);
    const workingDirectory = options.workingDirectory ?? DEFAULT_WORKING_DIR;
    if (options.mountWorkingFilesystem !== false) {
        ensureWorkingDirectory(module, workingDirectory);
    }
    if (options.preloadFiles && options.preloadFiles.length > 0) {
        await preloadFiles(module, options.preloadFiles, workingDirectory);
    }
    return { module };
}
export function decodeException(module, error) {
    if (typeof module?.getExceptionMessage === 'function') {
        try {
            const message = module.getExceptionMessage(error);
            if (message) {
                return message;
            }
        }
        catch (nested) {
            // ignore decoding errors and fall back to default handling
        }
    }
    if (error instanceof Error) {
        return error.message;
    }
    return String(error);
}
export { loadMujocoModule } from './runtime/loadMujoco.js';
export { createMujocoWorker } from './worker/client.js';
//# sourceMappingURL=index.js.map