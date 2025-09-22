import type {
  Model as RawModel,
  Simulation as RawSimulation,
  State as RawState,
  mujoco as RawMujocoModule,
} from '../dist/mujoco_wasm.js';

export interface AnalyzePathResult {
  object?: unknown;
  name?: string;
  parent?: unknown;
  mount?: unknown;
  exists: boolean;
}

export interface FileSystemLike {
  analyzePath(path: string): AnalyzePathResult;
  mkdir(path: string): void;
  mkdirTree?(path: string): void;
  mount(type: unknown, opts: Record<string, unknown>, mountpoint: string): void;
  writeFile(path: string, data: string | Uint8Array, opts?: Record<string, unknown>): void;
  readFile?(path: string, opts?: Record<string, unknown>): Uint8Array | string;
}

export type MujocoModule = RawMujocoModule & {
  FS: RawMujocoModule['FS'] & FileSystemLike;
  MEMFS: RawMujocoModule['MEMFS'];
  getExceptionMessage?(error: unknown): string;
};

export type MujocoModel = RawModel;
export type MujocoState = RawState;
export type MujocoSimulation = RawSimulation;

export interface CreateOptions {
  locateFile?: (path: string, prefix: string) => string;
  onRuntimeInitialized?: () => void;
  mountWorkingFilesystem?: boolean;
  workingDirectory?: string;
  preloadFiles?: PreloadFileDescriptor[];
  moduleOverrides?: Partial<MujocoModule>;
  readyTimeoutMs?: number;
}

export interface PreloadFileDescriptor {
  targetPath: string;
  binary?: boolean;
  data: string | ArrayBuffer | Uint8Array | (() => Promise<string | ArrayBuffer | Uint8Array>);
}

export interface MujocoInitResult {
  module: MujocoModule;
}

export interface WorkerMessage<T = unknown> {
  type: string;
  payload?: T;
  requestId?: number;
  error?: string;
}
