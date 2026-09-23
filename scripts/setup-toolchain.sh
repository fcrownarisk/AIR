#!/usr/bin/env bash
# Install the TypeScript toolchain OUTSIDE the project tree.
#
# Rationale: this project has zero runtime dependencies, so `node_modules` would
# be 100% dev tooling. A local install of typescript + @types/node is ~670 files
# / ~34 MB. Hosting it under ~/.workbuddy/toolchains keeps the project tree at a
# few dozen files while `npm run typecheck` still works identically.
#
# If you would rather have a conventional layout, `npm install` in the project
# root still wins — scripts/tsc.mjs checks the project's own node_modules first.
set -euo pipefail

TS_VERSION="${TS_VERSION:-5.9.3}"
TYPES_NODE_VERSION="${TYPES_NODE_VERSION:-22.13.9}"
TS_HOME="${AIRCRAFT_TS_HOME:-$HOME/.workbuddy/toolchains/AirCraft}"
NODE_DIR="${NODE_DIR:-$HOME/.workbuddy/binaries/node/versions/22.22.2-2}"

if [ -x "$NODE_DIR/npm.cmd" ]; then
  NPM="$NODE_DIR/npm.cmd"
elif [ -x "$NODE_DIR/bin/npm" ]; then
  NPM="$NODE_DIR/bin/npm"
else
  NPM="$(command -v npm)"
fi

echo "toolchain home : $TS_HOME"
echo "npm            : $NPM"

mkdir -p "$TS_HOME"
cd "$TS_HOME"

if [ ! -f package.json ]; then
  printf '{\n  "name": "aircraft-prototype-toolchain",\n  "private": true,\n  "version": "0.0.0"\n}\n' > package.json
fi

"$NPM" install --no-audit --no-fund --save-exact \
  "typescript@$TS_VERSION" "@types/node@$TYPES_NODE_VERSION"

echo
echo "installed. verify with:  node scripts/tsc.mjs --where"
