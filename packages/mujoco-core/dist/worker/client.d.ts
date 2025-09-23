import type { CreateOptions } from '../types.js';
export interface MujocoWorkerHandle {
    readonly worker: Worker;
    readonly ready: Promise<void>;
    terminate(): void;
    call<T = unknown>(type: string, payload?: unknown): Promise<T>;
}
export declare function createMujocoWorker(options?: CreateOptions): MujocoWorkerHandle;
//# sourceMappingURL=client.d.ts.map