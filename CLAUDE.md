# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a MuJoCo WebAssembly simulation project with a Next.js frontend that provides real-time physics simulation in the browser. The project integrates MuJoCo physics engine via WebAssembly with interactive 3D visualization using Three.js.

## Architecture

### Directory Structure
- `/frontend` - Next.js 15 application with React 19
- `/packages/mujoco-core` - TypeScript wrapper for MuJoCo WASM runtime
- `/mujoco_wasm` - MuJoCo WebAssembly implementation and examples
- `/de_dust2-cs-map` - CS map assets for simulation environments

### Key Components
- **Frontend**: Next.js app with dynamic MuJoCo scene viewer at `/mujoco` route
- **MuJoCo Core Package**: Provides TypeScript bindings and worker support for MuJoCo WASM
- **MuJoCo Examples**: Interactive demos with drone control (Skydio X2) and robot simulations (Boston Dynamics Spot)

## Build Commands

### Frontend (Next.js)
```bash
cd frontend
npm install
npm run dev        # Development server with Turbopack
npm run build      # Production build with Turbopack
npm run lint       # Run ESLint
```

### MuJoCo Core Package
```bash
cd packages/mujoco-core
npm install
npm run build      # Sync WASM files and compile TypeScript
npm run lint       # Type check without building
npm run clean      # Remove dist directory
```

### MuJoCo WASM
The MuJoCo WASM files are built from C++ source. Pre-built artifacts are located in `mujoco_wasm/dist/`.

## Development Workflow

### Running the Application
1. Start the Next.js development server: `cd frontend && npm run dev`
2. Access the simulation at `http://localhost:3000/mujoco`

### Working with MuJoCo Scenes
- Scene files are XML-based MuJoCo models located in `mujoco_wasm/examples/scenes/`
- Current scenes include Skydio X2 drone and Boston Dynamics Spot robot
- Scene viewer uses Three.js for rendering and OrbitControls for camera navigation

### Keyboard Controls (Skydio Drone)
- `Space`: Ascend
- `Z`: Descend
- `W/S`: Pitch forward/backward
- `A/D`: Roll left/right

## Key Implementation Details

### MuJoCo Module Loading
The frontend loads MuJoCo via the `createMujocoModule` function which:
- Locates WASM files at `/mujoco/` public path
- Mounts a working filesystem for scene assets
- Returns initialized module and filesystem access

### Scene Configuration
Scenes are defined in `mujoco_wasm/examples/scenes/index.json` and loaded dynamically. Each scene includes:
- XML model definition
- Asset files (meshes, textures)
- Control parameters

### TypeScript Configuration
- Base config at `/tsconfig.base.json` with strict mode enabled
- Package-specific configs extend the base
- ES modules with Node16 module resolution

## Testing
Currently no automated tests are configured. Manual testing via the development server.

## Important Notes
- WASM files must be synced from `mujoco_wasm/dist` to `packages/mujoco-core/dist` before building the core package
- The frontend uses dynamic imports for the scene viewer to avoid SSR issues with WebGL
- Control smoothing is applied to drone inputs for realistic physics behavior