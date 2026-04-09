# botrail

**ROS-free robot motion authoring with a web-based 3D studio.**

botrail is a lightweight alternative to heavyweight motion-planning stacks:
`pip install botrail`, a few lines of Python, and you get an interactive 3D
studio in your browser for building environments, constraints, and motions.
The core is written in Rust (no system dependencies), with URDF **and Xacro**
support via [xurdf](https://github.com/neka-nat/xurdf) — no ROS required.

> **Status: M1.** URDF/Xacro loading, FK / Jacobian / damped-least-squares
> IK, and a live studio viewer: joint sliders plus a draggable TCP gizmo with
> server-side IK tracking and reachability feedback. See
> [docs/DESIGN.md](docs/DESIGN.md) for the roadmap (collision checking,
> planning, motion editing, wasm).

## Quickstart

```python
import botrail as bt

robot = bt.Robot.from_urdf("examples/simple_arm.urdf")  # or from_xacro(...)
scene = bt.Scene(robot)
bt.studio(scene)  # opens the 3D studio in your browser
```

In the studio, drag the TCP gizmo to pose the arm interactively (the server
solves IK live and flags unreachable targets), or use the joint sliders.
Everything is mirrored in Python:

```python
scene.set_joint_positions([0.4, -0.9, 1.2, 0.3, 0.8, -0.5])
position, quaternion = scene.link_pose("tool0")   # FK
result = robot.ik(position, quaternion)           # IK (check result.converged)
scene.set_tcp_target((0.3, 0.1, 0.5))             # IK + apply + push to browser
```

## Development setup

Requirements: Rust (stable), Python >= 3.9, [maturin](https://maturin.rs),
[uv](https://docs.astral.sh/uv/), Node 20+ with pnpm.

```bash
# 1. Build the studio UI and bundle it into the python package
./scripts/build_studio.sh

# 2. Build and install the extension module into a venv
uv venv .venv && source .venv/bin/activate
maturin develop --uv

# 3. Run the demo
python examples/demo.py
```

Tests:

```bash
cargo test -p botrail-model -p botrail-kin -p botrail-scene   # Rust core
python -m pytest python/tests                                  # Python bindings
```

Wire protocol: the Rust types in `crates/botrail-scene/src/wire.rs` are the
source of truth. After changing them, regenerate the TypeScript side
(`studio/src/generated/`, committed) with:

```bash
./scripts/gen_protocol.sh
```

Frontend dev loop (vite hot reload against a running botrail server):

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
crates/botrail-kin     forward kinematics
crates/botrail-scene   scene state + JSON wire protocol (source of truth)
crates/botrail-py      pyo3 bindings + axum server (websocket, meshes, SPA)
studio/                web UI (vite + React + react-three-fiber)
python/botrail/        python package (high-level API, bundled studio assets)
examples/              primitive-only sample arm + demo script
docs/DESIGN.md         architecture & roadmap
```

## License

MIT OR Apache-2.0
