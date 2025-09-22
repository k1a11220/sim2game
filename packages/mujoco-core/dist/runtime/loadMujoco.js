import loadMujocoFactory from '../../dist/mujoco_wasm.js';
const DEFAULT_TIMEOUT_MS = 20000;
function ensureBrowserEnvironment() {
    if (typeof window === 'undefined' && typeof self === 'undefined') {
        throw new Error('MuJoCo WebAssembly can only be initialised in a browser or worker context.');
    }
}
function awaitWithTimeout(promise, timeoutMs) {
    if (!Number.isFinite(timeoutMs) || timeoutMs <= 0) {
        return promise;
    }
    let timeoutHandle;
    return new Promise((resolve, reject) => {
        timeoutHandle = setTimeout(() => {
            timeoutHandle = undefined;
            reject(new Error(`MuJoCo module failed to initialise within ${timeoutMs}ms`));
        }, timeoutMs);
        promise
            .then((value) => {
            if (timeoutHandle) {
                clearTimeout(timeoutHandle);
            }
            resolve(value);
        })
            .catch((err) => {
            if (timeoutHandle) {
                clearTimeout(timeoutHandle);
            }
            reject(err);
        });
    });
}
export async function loadMujocoModule(options = {}) {
    ensureBrowserEnvironment();
    const { locateFile, onRuntimeInitialized, moduleOverrides = {}, readyTimeoutMs = DEFAULT_TIMEOUT_MS, } = options;
    const overrides = { ...moduleOverrides };
    if (locateFile) {
        overrides.locateFile = locateFile;
    }
    if (onRuntimeInitialized) {
        overrides.onRuntimeInitialized = onRuntimeInitialized;
    }
    const module = (await loadMujocoFactory(overrides));
    if (!module || typeof module !== 'object') {
        throw new Error('Failed to initialise MuJoCo module.');
    }
    if ('ready' in module && module.ready instanceof Promise) {
        await awaitWithTimeout(module.ready, readyTimeoutMs);
    }
    return module;
}
//# sourceMappingURL=loadMujoco.js.map