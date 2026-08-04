# Browser-only botrail

The studio does not need the Python server. The core compiles to WebAssembly,
and the wasm build bundles it with the UI into a **static page**: the full
studio — posing, planning, collision, USD import — running entirely in the
browser. No install, no server, no GPU; host it anywhere that serves files.

**Try it now: [the live demo](https://botrail.github.io/botrail/demo/)** — a
Franka in the factory cell, served from GitHub Pages. The robot is NVIDIA's
official Isaac asset, fetched straight from their CDN; the page itself is just
static files.

You can also drop a USD file straight into the viewport — stages become the
scene, articulations become robots, the same import pipeline as everywhere
else.

## How it works

The studio talks to a `SessionBackend` interface with two implementations: a
WebSocket connection to the Python server, or the in-browser wasm session.
Same UI, same wire protocol, same Rust core —
[Architecture](../concepts/architecture.md) has the picture. A server-mode
studio build switches into wasm mode with `?wasm` in the URL when the wasm
assets are present.

## Building it

```bash
./scripts/build_wasm_demo.sh              # needs wasm-pack + the wasm32 target
python -m http.server -d studio/dist-wasm 8899
```

The output of `build_wasm_demo.sh` is `studio/dist-wasm/` — deployable to any
static host as-is (this documentation site's `/demo/` is exactly that,
deployed by CI on every push).

## What it is for

* **Trying botrail before installing it** — the demo is the pitch.
* **Sharing a cell as a URL** — no "install this first" in the email.
* **Machines you can't install on** — a shop-floor terminal with a browser.

The Python API is, of course, the one thing a static page cannot give you:
scripted authoring, baking from code, pytest, exports to disk. The browser
build is the studio side of botrail, self-contained.
