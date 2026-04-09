#!/usr/bin/env bash
# Regenerates studio/src/generated/ (TypeScript wire types) from the Rust
# wire types in crates/botrail-scene/src/wire.rs via ts-rs.
# Run this after changing the wire types, then commit the result.
set -euo pipefail
cd "$(dirname "$0")/.."

export TS_RS_EXPORT_DIR="$PWD/studio/src/generated"
rm -rf "$TS_RS_EXPORT_DIR"
cargo test -p botrail-scene --features ts export_bindings --quiet

echo "generated into studio/src/generated:"
ls "$TS_RS_EXPORT_DIR"
