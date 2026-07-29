# Development notes

Contributor-facing details that used to live in the README. For the overall
architecture and roadmap, see [DESIGN.md](DESIGN.md).

## Studio backends

The studio UI talks to a `SessionBackend`: a WebSocket connection to the
Python server by default, or the in-browser wasm session in demo builds
(`VITE_BACKEND=wasm` at build time, or add `?wasm` to the URL when the wasm
assets are present). Both speak the same wire protocol.

## Wire protocol

The Rust types in `crates/botrail-scene/src/wire.rs` are the source of
truth. After changing them, regenerate the TypeScript side
(`studio/src/generated/`, committed) with:

```bash
./scripts/gen_protocol.sh
```

## Frontend dev loop

Vite hot reload against a running botrail server:

```bash
# terminal 1: serve a scene on a fixed port
python -c "
import botrail as bt
scene = bt.Scene(bt.Robot.from_urdf('examples/simple_arm.urdf'))
bt.studio(scene, port=8765, open_browser=False)
"

# terminal 2: vite dev server, proxies /ws and /meshes to 127.0.0.1:8765
cd studio && pnpm dev
```

## Repository layout

```
crates/botrail-model   URDF/Xacro -> indexed kinematic tree (via xurdf)
crates/botrail-mesh    minimal STL/OBJ triangle-mesh loading
crates/botrail-kin     forward kinematics, Jacobian, DLS inverse kinematics
crates/botrail-collide parry3d collision checking (solid shapes, VHACD, ACM)
crates/botrail-plan    RRT-Connect + shortcut smoothing
crates/botrail-traj    time parameterization + trajectory sampling
crates/botrail-scene   scene state, motions, projects + JSON wire protocol
crates/botrail-session shared wire dispatch + planning helpers (hub & wasm)
crates/botrail-usd     USD importer (openusd): scenes, frames, articulations
crates/botrail-py      pyo3 bindings + axum server (websocket, meshes, SPA)
crates/botrail-wasm    browser-complete session (same wire protocol, no server)
crates/botrail-bench   standalone perf probes (not shipped)
studio/                web UI (vite + React + react-three-fiber)
python/botrail/        python package (high-level API, bundled studio assets)
examples/              demo: Isaac Franka + hand-authored USD factory cell
docs/                  architecture (DESIGN.md) + these notes
```

## Extra test harnesses

- URScript export against a real controller simulator:
  `scripts/ursim_test.sh` boots URSim in docker and replays an exported
  script (auto-skips unless `BOTRAIL_URSIM_HOST` is set).
- Isaac asset golden tests: `BOTRAIL_ISAAC_DIR=<dir with franka.usd/ur10.usd>
  python -m pytest python/tests/test_usd_robot.py -k golden` (auto-skips
  unless the env var is set).
