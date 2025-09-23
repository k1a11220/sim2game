import type { MujocoModule, PreloadFileDescriptor } from './types.js';
export declare const DEFAULT_WORKING_DIR = "/working";
export declare function ensureWorkingDirectory(module: MujocoModule, path?: string): void;
export declare function preloadFiles(module: MujocoModule, descriptors?: PreloadFileDescriptor[], baseDir?: string): Promise<void>;
//# sourceMappingURL=fs.d.ts.map