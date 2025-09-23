export const DEFAULT_WORKING_DIR = '/working';
function safeAnalyze(module, target) {
    try {
        return module.FS.analyzePath(target);
    }
    catch (err) {
        return { exists: false };
    }
}
function ensureDirRecursive(module, dirPath) {
    if (!dirPath || dirPath === '/') {
        return;
    }
    const segments = dirPath.split('/').filter(Boolean);
    let current = '';
    for (const segment of segments) {
        current = `${current}/${segment}`;
        const analyzed = safeAnalyze(module, current);
        if (!analyzed.exists) {
            module.FS.mkdir(current);
        }
    }
}
export function ensureWorkingDirectory(module, path = DEFAULT_WORKING_DIR) {
    const analyzed = safeAnalyze(module, path);
    if (!analyzed.exists) {
        ensureDirRecursive(module, path);
    }
    try {
        module.FS.mount(module.MEMFS, { root: '.' }, path);
    }
    catch (err) {
        // MEMFS is already mounted; ignore.
    }
}
function toUint8Array(data) {
    return data instanceof Uint8Array ? data : new Uint8Array(data);
}
async function resolveData(descriptor) {
    const value = typeof descriptor.data === 'function' ? await descriptor.data() : descriptor.data;
    if (descriptor.binary) {
        if (typeof value === 'string') {
            throw new Error(`Binary preload target ${descriptor.targetPath} must provide ArrayBuffer or Uint8Array data.`);
        }
        return toUint8Array(value instanceof ArrayBuffer ? value : value);
    }
    if (typeof value === 'string') {
        return value;
    }
    if (value instanceof ArrayBuffer || value instanceof Uint8Array) {
        const decoder = new TextDecoder();
        const array = value instanceof Uint8Array ? value : new Uint8Array(value);
        return decoder.decode(array);
    }
    throw new Error(`Unsupported data type for ${descriptor.targetPath}`);
}
function ensureParentDir(module, filePath) {
    const parts = filePath.split('/');
    if (parts.length <= 2) {
        return;
    }
    const directory = parts.slice(0, -1).join('/') || '/';
    ensureDirRecursive(module, directory);
}
export async function preloadFiles(module, descriptors = [], baseDir = DEFAULT_WORKING_DIR) {
    for (const descriptor of descriptors) {
        const target = descriptor.targetPath.startsWith('/')
            ? descriptor.targetPath
            : `${baseDir.replace(/\/$/, '')}/${descriptor.targetPath}`;
        ensureParentDir(module, target);
        const data = await resolveData(descriptor);
        module.FS.writeFile(target, typeof data === 'string' ? data : toUint8Array(data));
    }
}
//# sourceMappingURL=fs.js.map