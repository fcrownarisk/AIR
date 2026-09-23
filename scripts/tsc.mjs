#!/usr/bin/env node
/**
 * TypeScript launcher that keeps the compiler OUTSIDE this project's tree.
 *
 * This project has no runtime dependencies — Node 22.18+ executes the .ts
 * sources directly via type stripping. The only tool needed is `tsc`, and it is
 * never used at runtime, so installing it here would add ~670 files / ~34 MB of
 * node_modules for nothing.
 *
 * Resolution order (a conventional local `npm install` must keep winning,
 * because CI and contributors who do not use this layout rely on it):
 *   1. $AIRCRAFT_TS_HOME                       explicit override
 *   2. <project>/node_modules                  conventional install
 *   3. ~/.workbuddy/toolchains/<project>       pinned out-of-tree toolchain
 *   4. <node>/workspace/node_modules           shared managed workspace
 *
 * The subtlety that makes this file necessary rather than optional:
 * TypeScript discovers @types by walking *ancestor* directories of the tsconfig.
 * An out-of-tree toolchain is not an ancestor, so the compiler fails with
 * TS2688 ("Cannot find type definition file for 'node'") unless --typeRoots is
 * passed explicitly. That flag is added at invocation time and never written
 * into tsconfig.json, which would bake an absolute machine path into the repo.
 */

import { spawn } from 'node:child_process';
import { existsSync, readFileSync } from 'node:fs';
import { homedir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));
const PROJECT_ROOT = resolve(HERE, '..');

/** @returns {{dir: string, entry: string, version: string, typeRoots: string|null, source: string}|null} */
function inspect(nodeModulesDir, source) {
  if (!nodeModulesDir) return null;
  const pkgPath = join(nodeModulesDir, 'typescript', 'package.json');
  if (!existsSync(pkgPath)) return null;

  let version = 'unknown';
  try {
    version = JSON.parse(readFileSync(pkgPath, 'utf8')).version;
  } catch {
    return null;
  }

  // tsc.js exists across 5.x and 7.x; 7.x reduces it to a small shim. Fall back
  // to bin/tsc rather than assuming a native binary layout.
  const candidates = [
    join(nodeModulesDir, 'typescript', 'lib', 'tsc.js'),
    join(nodeModulesDir, 'typescript', 'bin', 'tsc'),
  ];
  const entry = candidates.find((p) => existsSync(p));
  if (!entry) return null;

  const typesDir = join(nodeModulesDir, '@types');
  return { dir: nodeModulesDir, entry, version, typeRoots: existsSync(typesDir) ? typesDir : null, source };
}

const home = homedir();
const candidates = [
  inspect(process.env.AIRCRAFT_TS_HOME ? join(process.env.AIRCRAFT_TS_HOME, 'node_modules') : null, 'AIRCRAFT_TS_HOME'),
  inspect(join(PROJECT_ROOT, 'node_modules'), 'project node_modules'),
  inspect(join(home, '.workbuddy', 'toolchains', 'AirCraft', 'node_modules'), 'pinned toolchain'),
  inspect(join(home, '.workbuddy', 'binaries', 'node', 'workspace', 'node_modules'), 'managed workspace'),
].filter((c) => c !== null);

const chosen = candidates[0];

if (process.argv.includes('--where')) {
  if (!chosen) {
    console.error('no TypeScript toolchain found; run scripts/setup-toolchain.sh');
    process.exit(1);
  }
  console.log(`typescript ${chosen.version}`);
  console.log(`source:     ${chosen.source}`);
  console.log(`entry:      ${chosen.entry}`);
  console.log(`typeRoots:  ${chosen.typeRoots ?? '(none)'}`);
  process.exit(0);
}

if (!chosen) {
  console.error('No TypeScript toolchain found. Run: bash scripts/setup-toolchain.sh');
  process.exit(1);
}

// Warn on version drift: a wrapper that silently falls through to a different
// compiler produces a build that does not match CI, with nothing in the repo to
// explain why.
try {
  const lock = JSON.parse(readFileSync(join(PROJECT_ROOT, 'package-lock.json'), 'utf8'));
  const pinned = lock?.packages?.['node_modules/typescript']?.version;
  if (pinned && pinned !== chosen.version) {
    console.error(`warning: toolchain differs from the lockfile (${chosen.source})`);
    console.error(`         typescript: using ${chosen.version}, lockfile pins ${pinned}`);
    console.error('         the build may not match what CI produces');
  }
} catch {
  /* no lockfile — nothing to compare against */
}

const finalArgs = [...process.argv.slice(2)];
if (chosen.typeRoots) finalArgs.push('--typeRoots', chosen.typeRoots);

const child = spawn(process.execPath, [chosen.entry, ...finalArgs], { stdio: 'inherit' });
child.on('exit', (code, signal) => process.exit(signal ? 1 : (code ?? 1)));
