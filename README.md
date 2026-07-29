# botrail

**ROS-free robot motion authoring with a web-based 3D studio.**

`pip install botrail`, a few lines of Python, and you get an interactive 3D
studio in your browser for building environments and motions. The core is
written in Rust — no ROS, no system dependencies.

## Highlights

- **Robots from URDF, Xacro, or USD** — including Isaac Sim articulations
  (`bt.Robot.from_usd("franka.usd")`), rendered at full visual fidelity
  with [three-usd-robot](https://github.com/neka-nat/three-usd-robot).
- **USD scene import** (usda/usdc/usdz, references, variants, instancing) —
  stages become obstacles and named mount frames, normalized to meters / Z-up.
- **Interactive posing** — draggable TCP gizmo with live IK, joint sliders.
- **Collision checking** — primitives and STL/OBJ meshes (cached VHACD convex
  decomposition), live highlighting, clearance readout.
- **Motion planning & authoring** — RRT-Connect with time parameterization,
  waypoint motions with Cartesian-line segments and path constraints,
  trajectory playback in the studio.
- **Portable projects** — save/load `.botrail` files (meshes and USD stages
  bundled), regenerate any scene as a Python script, export trajectories to
  CSV/JSON or robot programs (URScript).
- **Runs entirely in the browser** — the wasm build serves the full studio as
  a static page, no server; drop a USD file straight into the viewport.

## Try it

Run the bundled demo — a Franka Panda in a small USD factory cell (the first
run downloads NVIDIA's official Franka asset, ~10 MB):

```bash
python examples/demo.py
```

Or try the browser-only build (deploys to GitHub Pages from `main`, or build
it locally):

```bash
./scripts/build_wasm_demo.sh          # needs wasm-pack + wasm32 target
python -m http.server -d studio/dist-wasm 8899
```

## Quickstart

```python
import botrail as bt

robot = bt.Robot.from_urdf("robot.urdf")   # or from_xacro(...) / from_usd(...)
scene = bt.Scene(robot)

scene.load_usd("cell.usda", prefix="env")                # obstacles + frames
scene.set_robot_base_pose(*scene.frame("env/World/mount"))
scene.add_box("table", size=(0.6, 0.6, 0.05), position=(0.4, 0.0, 0.0))

bt.studio(scene)  # opens the 3D studio in your browser
```

Everything you do in the studio is mirrored in Python, and vice versa:

```python
scene.set_tcp_target((0.3, 0.1, 0.5))         # live IK, pushed to the browser
scene.in_collision()                          # False
scene.min_obstacle_distance()                 # clearance in meters

traj = scene.plan_to_pose((0.4, 0.1, 0.3))    # IK, then RRT-Connect + time param
traj.export_csv("motion.csv", dt=0.008)

scene.save_project("cell.botrail")            # meshes/USD bundled when needed
print(scene.generate_python())                # script reproducing the scene
```

## Development

Requirements: Rust (stable), Python >= 3.9, [maturin](https://maturin.rs),
[uv](https://docs.astral.sh/uv/), Node 20+ with pnpm.

```bash
./scripts/build_studio.sh                 # build the studio UI into the package
uv venv .venv && source .venv/bin/activate
maturin develop --uv
python examples/demo.py
```

Tests:

```bash
cargo test                                # Rust workspace
python -m pytest python/tests             # Python bindings
```

Architecture and contributor notes live in [docs/DESIGN.md](docs/DESIGN.md)
and [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md).

## License

MIT
