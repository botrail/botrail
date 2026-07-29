# botrail

**ROS-free robot motion authoring with a web-based 3D studio.**

botrail is a lightweight alternative to heavyweight motion-planning stacks:
`pip install botrail`, a few lines of Python, and you get an interactive 3D
studio in your browser for building environments, constraints, and motions.
The core is written in Rust (no system dependencies), with URDF **and Xacro**
support via [xurdf](https://github.com/neka-nat/xurdf) — no ROS required.

> **Status: M5.** URDF/Xacro loading, FK / IK with a draggable TCP gizmo,
> collision checking (auto-generated ACM, obstacle editing, live
> highlighting), motion planning (RRT-Connect + time parameterization with
> studio playback), motion authoring (waypoint sequences, Cartesian-line
> segments, path constraints, `.botrail` projects, Python codegen) — and a
> **browser-complete wasm build**: the entire core runs client-side, so the
> studio works as a static page with no server at all. Mesh collision
> shapes are in: STL/OBJ links and obstacles collide via cached VHACD
> convex decompositions. USD scenes (usda/usdc/usdz, incl. references,
> variants, instancing, Isaac-style `omniverse://` paths via search dirs)
> import as world-posed obstacles and named mount frames, normalized to
> meters / Z-up. Robots themselves can come from USD articulations
> (UsdPhysics joints/bodies, Isaac Sim style) via `bt.Robot.from_usd` —
> the studio then renders the original stage client-side with
> [three-usd-robot](https://github.com/neka-nat/three-usd-robot) (full
> visual fidelity, client-side FK, joint-only wire traffic).

**Try it in your browser** (no install): the demo deploys to GitHub Pages
from `main` (`.github/workflows/pages.yml`) — or build it locally:

```bash
./scripts/build_wasm_demo.sh          # needs wasm-pack + wasm32 target
python -m http.server -d studio/dist-wasm 8899   # any static server works
```

The studio UI talks to a `SessionBackend`: a WebSocket connection to the
Python server by default, or the in-browser wasm session in demo builds
(`VITE_BACKEND=wasm`, or add `?wasm` to the URL when the assets are
present). Both speak the same wire protocol.

## Quickstart

```python
import botrail as bt

robot = bt.Robot.from_urdf("examples/simple_arm.urdf")  # or from_xacro(...)
robot = bt.Robot.from_usd("franka.usd")                 # or a USD articulation
scene = bt.Scene(robot)
bt.studio(scene)  # opens the 3D studio in your browser
```

In the studio, drag the TCP gizmo to pose the arm interactively (the server
solves IK live and flags unreachable targets), or use the joint sliders.
Everything is mirrored in Python:

```python
scene.set_joint_positions([0.4, -0.9, 1.2, 0.3, 0.8, -0.5])
scene.set_robot_base_pose((1.0, 0.5, 0.0))        # place the robot in the world
position, quaternion = scene.link_pose("tool0")   # FK (world frame)
result = robot.ik(position, quaternion)           # IK (check result.converged)
scene.set_tcp_target((0.3, 0.1, 0.5))             # IK + apply + push to browser

scene.add_box("table", size=(0.6, 0.6, 0.05), position=(0.4, 0.0, 0.0))
scene.add_mesh("crate", "crate.stl", position=(0.3, 0.2, 0.1))  # VHACD, cached
scene.in_collision()                              # False

scene.load_usd("cell.usda", prefix="env")         # USD stage -> obstacles+frames
scene.set_robot_base_pose(*scene.frame("env/World/mount"))  # place on a frame
scene.min_obstacle_distance()                     # clearance in meters
scene.check_collisions()                          # [(("link", ...), ("obstacle", ...)), ...]

traj = scene.plan([0.0, 1.2, -0.9, 0.4, 0.0, 0.0])  # RRT-Connect + time param
traj = scene.plan_to_pose((0.4, 0.1, 0.3))           # IK, then plan
traj.duration, traj.sample(0.5)                      # seconds / joint values
traj.export_csv("motion.csv", dt=0.008)              # or export_json(...)

scene.add_segment("pick", goal=[0.5, 0.9, -1.2, 0.3, 0.0, 0.0])
scene.add_segment("pick", kind="cartesian_line",     # straight TCP descent,
                  orientation_cone=((0, 0, 1), (0, 0, 1), 0.35))  # tool upright
traj = scene.plan_motion("pick")                     # one trajectory, rest at waypoints

scene.save_project("cell.botrail")                   # JSON, or zip when meshes are referenced
scene = bt.Scene.load_project("cell.botrail")
print(scene.generate_python())                       # script reproducing it all
```

Planned trajectories are pushed to the studio automatically and can be
previewed there (goal ghost, playback slider with segment markers). In the
UI you can capture waypoints from the posed robot, plan and replay whole
motions, and save/load `.botrail` projects or export them as Python.

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
examples/              primitive-only sample arm + demo script
docs/DESIGN.md         architecture & roadmap
```

## License

MIT OR Apache-2.0
