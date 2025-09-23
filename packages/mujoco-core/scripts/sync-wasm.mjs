import { fileURLToPath } from 'node:url';
import { mkdir, copyFile, access } from 'node:fs/promises';
import path from 'node:path';

const packageRoot = path.resolve(fileURLToPath(new URL('..', import.meta.url)));
const repoRoot = path.resolve(packageRoot, '..', '..');
const destDir = path.join(packageRoot, 'dist');
const artifacts = ['mujoco_wasm.js', 'mujoco_wasm.wasm', 'mujoco_wasm.d.ts'];

async function resolveSourceDir() {
  const candidates = [
    path.join(repoRoot, 'dist'),
    path.join(repoRoot, 'mujoco_wasm', 'dist'),
  ];

  for (const candidate of candidates) {
    try {
      await access(path.join(candidate, artifacts[0]));
      return candidate;
    } catch (err) {
      // try next candidate
    }
  }

  throw new Error(
    `Missing wasm artifacts. Checked: ${candidates.join(', ')}. Build mujoco_wasm before syncing.`,
  );
}

async function main() {
  const sourceDir = await resolveSourceDir();
  await mkdir(destDir, { recursive: true });
  for (const name of artifacts) {
    await copyFile(path.join(sourceDir, name), path.join(destDir, name));
  }
}

main().catch((err) => {
  console.error(err instanceof Error ? err.message : err);
  process.exitCode = 1;
});
