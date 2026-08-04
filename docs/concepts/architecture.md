# Architecture

A reader's map of how botrail is put together — enough to reason about what
runs where, and why the same studio works from Python and from a static web
page. (Build instructions and the full crate list live in
[Contributing](../contributing.md).)

## The shape

```text
                ┌───────────────────────────────────┐
                │      botrail studio (web UI)      │
                │   TypeScript · React · three.js   │
                └────────────────┬──────────────────┘
                                 │  SessionBackend interface
              ┌──────────────────┴──────────────────┐
              │ WebSocket RPC                       │ in-process calls
    ┌─────────┴─────────┐               ┌───────────┴───────┐
    │  Python process   │               │   wasm build      │
    │  botrail (pyo3)   │               │  (browser-only)   │
    └─────────┬─────────┘               └───────────┬───────┘
              └──────────────────┬──────────────────┘
                    ┌────────────┴────────────┐
                    │     shared dispatch     │
                    └────────────┬────────────┘
        ┌────────────────────────┴────────────────────────┐
        │  Rust core                                      │
        │  model · kinematics · collision · planning ·    │
        │  trajectories · scene (motions, sequences,      │
        │  rollout) · USD import/export                   │
        └─────────────────────────────────────────────────┘
```

Everything below the UI is **Rust**, compiled twice: into a Python extension
(pyo3) and into WebAssembly. The wheel you `pip install` carries the core, an
embedded web server, and the built studio — which is why installation is one
step and offline.

## The decisions that matter

**The UI doesn't know its backend.** The studio depends on one TypeScript
interface — fetch the scene, apply changes, request plans, receive
trajectories — with two implementations: WebSocket RPC to the Python server,
or direct wasm calls in the page. Write the UI once, get the
[browser-only build](../guides/browser-only.md) for free.

**The protocol has one source of truth.** Scene, trajectory, and command
types are Rust types with serialization derived; the TypeScript side is
generated from them. The two ends of the wire cannot drift apart silently.

**Python and the studio share one operation model.** Everything the UI does
lands as the same wire messages the API sends — which is the invariant behind
"everything you do in the studio is mirrored in Python, and vice versa," and
behind the studio's **Export .py** producing a script that recreates UI work.

**The core doesn't know USD.** USD import/export is a boundary layer: stages
come in already normalized to meters and Z-up as plain obstacles, frames, and
robot models; export takes a finished scene-plus-timeline and authors a layer.
The geometry/planning core stays format-agnostic — and identical in the wasm
build.

**USD robot links are named by prim path.** Server and browser refer to the
same link by the same string, so collision highlighting, goal ghosts, and
recording playback resolve 1:1 with no mapping tables.

## Deliberately small dependencies

The generic pieces live outside botrail: URDF/Xacro parsing
([xurdf](https://github.com/neka-nat/xurdf)), USD robot rendering in the
browser ([three-usd-robot](https://github.com/neka-nat/three-usd-robot)), and
the USD file format layer. botrail itself is the cell model, the rollout, and
the studio — the parts that are actually about cells.
