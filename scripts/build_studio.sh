#!/usr/bin/env bash
# Builds the studio SPA and bundles it into the python package
# (python/botrail/_studio), so `maturin develop/build` ships it in the wheel.
set -euo pipefail
cd "$(dirname "$0")/.."

(cd studio && pnpm install && pnpm build)

rm -rf python/botrail/_studio
cp -r studio/dist python/botrail/_studio
echo "studio assets bundled into python/botrail/_studio"
