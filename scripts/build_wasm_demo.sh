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

echo "wasm demo built at studio/dist-wasm (serve statically to try it)"
