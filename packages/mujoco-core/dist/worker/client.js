let requestSequence = 0;
export function createMujocoWorker(options = {}) {
    if (typeof window === 'undefined' || typeof Worker === 'undefined') {
        throw new Error('Web Workers are not available in the current environment.');
    }
    const worker = new Worker(new URL('./entry.js', import.meta.url), { type: 'module' });
    const pending = new Map();
    worker.addEventListener('message', (event) => {
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
        }
        else {
            record.resolve(payload);
        }
    });
    const send = (type, payload) => {
        const requestId = ++requestSequence;
        const message = { type, payload, requestId };
        worker.postMessage(message);
        return new Promise((resolve, reject) => {
            pending.set(requestId, {
                resolve: (value) => resolve(value),
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
//# sourceMappingURL=client.js.map