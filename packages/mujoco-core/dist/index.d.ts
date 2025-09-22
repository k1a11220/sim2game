export { ensureWorkingDirectory, preloadFiles, DEFAULT_WORKING_DIR as DEFAULT_WORKING_DIRECTORY } from './fs.js';
export type { MujocoModule, MujocoModel, MujocoState, MujocoSimulation, PreloadFileDescriptor, CreateOptions, MujocoInitResult, } from './types.js';
export type { MujocoWorkerHandle } from './worker/client.js';
import type { CreateOptions, MujocoInitResult, MujocoModule } from './types.js';
export declare function createMujocoModule(options?: CreateOptions): Promise<MujocoInitResult>;
export declare function decodeException(module: MujocoModule, error: unknown): string;
export { loadMujocoModule } from './runtime/loadMujoco.js';
export { createMujocoWorker } from './worker/client.js';
//# sourceMappingURL=index.d.ts.map