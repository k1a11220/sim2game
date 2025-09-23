import type { CreateOptions, WorkerMessage } from '../types.js';

export interface MujocoWorkerHandle {
  readonly worker: Worker;
  readonly ready: Promise<void>;
  terminate(): void;
  call<T = unknown>(type: string, payload?: unknown): Promise<T>;
}

let requestSequence = 0;

export function createMujocoWorker(options: CreateOptions = {}): MujocoWorkerHandle {
  if (typeof window === 'undefined' || typeof Worker === 'undefined') {
    throw new Error('Web Workers are not available in the current environment.');
  }

  const worker = new Worker(new URL('./entry.js', import.meta.url), { type: 'module' });
  const pending = new Map<number, { resolve: (value: unknown) => void; reject: (reason?: unknown) => void }>();

  worker.addEventListener('message', (event: MessageEvent<WorkerMessage>) => {
    const { requestId, error, payload } = event.data;
    if (typeof requestId !== 'number') {
      return;
    }
    const record = pending.get(requestId);
    if (!record) {
      return;
    }
    pending.delete(requestId);
    if (error) {
      record.reject(new Error(error));
    } else {
      record.resolve(payload);
    }
  });

  const send = <T = unknown>(type: string, payload?: unknown): Promise<T> => {
    const requestId = ++requestSequence;
    const message: WorkerMessage = { type, payload, requestId };
    worker.postMessage(message);
    return new Promise<T>((resolve, reject) => {
      pending.set(requestId, {
        resolve: (value) => resolve(value as T),
        reject,
      });
    });
  };

  const ready = send('init', { options }).then(() => undefined);

  return {
    worker,
    ready,
    terminate() {
      worker.terminate();
      pending.forEach(({ reject }) => reject(new Error('Worker terminated')));
      pending.clear();
    },
    call: send,
  };
}
