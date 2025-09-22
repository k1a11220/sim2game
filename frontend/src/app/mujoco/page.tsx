'use client';

import { useEffect, useState } from 'react';
import dynamic from 'next/dynamic';
import { createMujocoModule } from '@sim2game/mujoco-core';

const STATUS = {
  idle: 'idle',
  loading: 'loading',
  ready: 'ready',
  error: 'error',
} as const;

type Status = (typeof STATUS)[keyof typeof STATUS];

const SceneViewer = dynamic(() => import('./scene-viewer'), {
  ssr: false,
  loading: () => <p>Preparing MuJoCo…</p>,
});

export default function MujocoPage() {
  const [status, setStatus] = useState<Status>(STATUS.loading);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;

    (async () => {
      try {
        const { module } = await createMujocoModule({
          locateFile: (path, prefix) =>
            path.endsWith('.wasm') ? `/mujoco/${path}` : `${prefix}${path}`,
          mountWorkingFilesystem: true,
        });

        if (!cancelled) {
          // eslint-disable-next-line no-console
          console.log('MuJoCo ready', module);
          setStatus(STATUS.ready);
        }
      } catch (err) {
        if (!cancelled) {
          setError(err instanceof Error ? err.message : String(err));
          setStatus(STATUS.error);
        }
      }
    })();

    return () => {
      cancelled = true;
    };
  }, []);

  if (status === STATUS.error) {
    return (
      <div className="p-6">
        <h1 className="text-xl font-semibold">MuJoCo 초기화 실패</h1>
        <p className="mt-2 text-sm text-red-500">{error}</p>
      </div>
    );
  }

  if (status !== STATUS.ready) {
    return (
      <div className="p-6">
        <p>MuJoCo 모듈을 준비하고 있어요…</p>
      </div>
    );
  }

  return <SceneViewer />;
}
