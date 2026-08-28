#!/usr/bin/env bash
# Builds the browser-complete demo: botrail core compiled to wasm, bundled
# with the studio UI into studio/dist-wasm (deployable to any static host).
#
# The wasm package lives outside studio/public so regular server-bundled
# builds don't ship it; it is copied into the demo output after the build.
set -euo pipefail
cd "$(dirname "$0")/.."

wasm-pack build crates/botrail-wasm --target web --release \
  --out-dir ../../studio/public-wasm/wasm --no-typescript

(cd studio && pnpm install && pnpm exec tsc -b && VITE_BACKEND=wasm pnpm exec vite build --outDir dist-wasm)
cp -r studio/public-wasm/wasm studio/dist-wasm/wasm

# The demo cell: the hand-authored layout layer plus the catalog equipment
# (belt, rack, guarding) pre-baked by scripts/bake_demo_equipment.py — the
# browser cannot order from the catalog at run time. The robot itself is not
# bundled: the browser pulls NVIDIA's Franka straight from their CDN (which
# sends `Access-Control-Allow-Origin: *`), so ~10 MB of third-party asset
# stays out of the deployed artifact.
mkdir -p studio/dist-wasm/cell
cp examples/assets/factory.usda examples/assets/factory_equipment.usda \
  studio/dist-wasm/cell/

echo "wasm demo built at studio/dist-wasm (serve statically to try it)"
