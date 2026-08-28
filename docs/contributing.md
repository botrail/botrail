# Contributing

botrail is a Rust workspace with pyo3 bindings and a React studio bundled into
the Python package. This page covers building it from source. For a reader's
view of how the pieces fit, see [Architecture](concepts/architecture.md).

## Licensing contributions

botrail uses a source-available and commercial licensing model. Before an
external contribution can be merged, the contributor must enter into a
Contributor License Agreement (CLA) with UnRobotics Inc. that permits the
contribution to be distributed under botrail's public and commercial licenses.
Opening a pull request does not by itself grant these additional rights or
guarantee that the contribution will be accepted.

Please use the [UnRobotics Inc. contact form](https://www.un-robotics.com/#contact)
before contributing so the CLA can be completed. See [Licensing](license.md)
for the licenses that apply to the repository.

## Build from source

Requirements: Rust (stable), Python 3.9+, [maturin](https://maturin.rs),
[uv](https://docs.astral.sh/uv/), and Node 20+ with pnpm.

```bash
./scripts/build_studio.sh          # build the studio UI into the package
uv venv .venv && source .venv/bin/activate
maturin develop --uv
python examples/basics/demo.py
```

!!! warning "Rebuild the studio bundle after touching `studio/`"

    `bt.studio()` serves the copy under `python/botrail/_studio/`, not
    `studio/dist`. Skipping `build_studio.sh` after a UI change means you keep
    testing the old bundle.

## Tests

```bash
cargo test                          # Rust workspace
python -m pytest python/tests       # Python bindings
```

Two extra harnesses skip themselves unless you opt in:

* **URScript against a real controller** — `scripts/ursim_test.sh` boots URSim
  in Docker and replays an exported script (needs `BOTRAIL_URSIM_HOST`).
* **Isaac asset golden tests** —
  `BOTRAIL_ISAAC_DIR=<dir with franka.usd/ur10.usd> python -m pytest python/tests/test_usd_robot.py -k golden`.

## Frontend dev loop

The studio talks to a `SessionBackend`: a WebSocket to the Python server by
default, or the in-browser wasm session in demo builds (`VITE_BACKEND=wasm` at
build time, or `?wasm` in the URL when the wasm assets are present). Both speak
the same wire protocol.

```bash
# terminal 1: serve a scene on a fixed port
python -c "
import botrail as bt
scene = bt.Scene(bt.Robot.from_urdf('examples/assets/simple_arm.urdf'))
bt.studio(scene, port=8765, open_browser=False)
"

# terminal 2: vite dev server, proxies /ws and /meshes to 127.0.0.1:8765
cd studio && pnpm dev
```

The Rust types in `crates/botrail-scene/src/wire.rs` are the source of truth for
the protocol. After changing them, regenerate the committed TypeScript side with
`./scripts/gen_protocol.sh`. The `.botrail` JSON Schema is generated from the
same types (`crates/botrail-scene/src/project.rs`, feature `schema`); after
changing anything a project file carries, refresh the committed copy with
`python -c "import botrail as bt; open('docs/assets/project.schema.json','w').write(bt.project_schema())"`
— a test compares the two.

## Repository layout

```text
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
docs/                  this documentation site (mkdocs)
```

## Working on these docs

The site is MkDocs Material. The API reference is generated from the pyo3
docstrings, so **the extension has to be importable** before it will build:

```bash
uv sync --group docs        # or: uv pip install --group docs
maturin develop --uv        # mkdocstrings imports botrail to read its docstrings
mkdocs serve                # http://127.0.0.1:8000
```

`mkdocs build --strict` is what CI runs; it fails on broken internal links and
unresolved API cross-references.

!!! note "Where API documentation comes from"

    Method documentation is the `///` comments in `crates/botrail-py/src/lib.rs`,
    read from the imported module at build time. `python/botrail/_core.pyi`
    carries the signatures for type checkers, but no prose — so the place to
    improve a docstring is always the Rust source.
