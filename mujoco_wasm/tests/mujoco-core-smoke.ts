import { decodeException } from '../packages/mujoco-core/src/index.js';
import type { MujocoModule } from '../packages/mujoco-core/src/types.js';

// Lightweight compile-time smoke check for the helper API.
const fakeModule: Partial<MujocoModule> = {
  getExceptionMessage: () => 'fake',
  FS: {
    analyzePath: () => ({ exists: true }),
    mkdir: () => void 0,
    mount: () => void 0,
    writeFile: () => void 0,
  } as MujocoModule['FS'],
  MEMFS: {},
};

decodeException(fakeModule as MujocoModule, new Error('placeholder'));
