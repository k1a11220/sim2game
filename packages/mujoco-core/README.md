# @sim2game/mujoco-core

High-level utilities for loading the MuJoCo WebAssembly module in browser and worker contexts.

## Features

- Promise-based loader with timeout guards and browser-only checks.
- File-system helpers for mounting `/working` and preloading XML/assets.
- Optional worker bootstrap that prepares the runtime off the main thread.
- Thin TypeScript wrappers around the generated `mujoco_wasm` bundle.

## Quick start

```ts
import {
  createMujocoModule,
  DEFAULT_WORKING_DIRECTORY,
  preloadFiles,
} from '@sim2game/mujoco-core';

const { module } = await createMujocoModule({
  locateFile: (path, prefix) =>
    path.endsWith('.wasm') ? `/mujoco/${path}` : `${prefix}${path}`,
  preloadFiles: [
    {
      targetPath: `${DEFAULT_WORKING_DIRECTORY}/examples/scenes/humanoid.xml`,
      data: () => fetch('/scenes/humanoid.xml').then((r) => r.text()),
    },
  ],
});

const model = module.Model.load_from_xml('/working/examples/scenes/humanoid.xml');
const state = new module.State(model);
const simulation = new module.Simulation(model, state);
simulation.step();
```

## Worker bootstrap

```ts
import { createMujocoWorker } from '@sim2game/mujoco-core/worker';

const handle = createMujocoWorker({ readyTimeoutMs: 30_000 });
await handle.ready;
// Additional RPC handlers can be added via `handle.call(...)`.
```

## Building locally

Run `npm run build` inside the package after building the MuJoCo wasm artefacts at the repository root. The build step copies `dist/mujoco_wasm.{js,wasm,d.ts}` into the package and emits TypeScript output.

> **Note**
> The repository operates without network access. Install `typescript` ahead of time or use a vendored copy if you need to execute the build script locally.
