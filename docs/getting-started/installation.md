# Installation

```bash
pip install botrail
```

That is the whole install. botrail ships as a binary wheel with the Rust core
and the studio UI bundled in — there is **no ROS, no system package, and no GPU**
to set up.

Check it:

```bash
python -c "import botrail; print(botrail.__version__)"
```

## Requirements

| | |
| --- | --- |
| Python | 3.9 or newer |
| Platforms | Linux, macOS, Windows (wheels are published for all three) |
| Optional | A browser, for the studio UI — it opens automatically |
| Optional | [usdview], Omniverse, or Blender, to view exported USD animations |

[usdview]: https://openusd.org/release/toolset.html#usdview

Nothing else is required to plan motions, bake cycles, or export results. The
studio is served by botrail itself on `127.0.0.1`, so it works offline.

!!! note "Robot assets are not bundled"

    botrail loads *your* robot from URDF, Xacro, or USD. The repository's
    examples download NVIDIA's official Franka asset (~10 MB) on first run and
    cache it; the [Quickstart](quickstart.md) instead uses a small
    primitive-geometry arm you can paste into a file, so it needs no downloads.

## Using uv

```bash
uv add botrail          # in a project
uv pip install botrail  # into the active environment
```

## Installing from source

You need this only to work on botrail itself — the wheel is self-contained.
Building from source additionally requires Rust (stable),
[maturin](https://maturin.rs), and Node 20+ with pnpm for the studio bundle:

```bash
git clone https://github.com/botrail/botrail
cd botrail
./scripts/build_studio.sh          # bundles the studio UI into the package
uv venv .venv && source .venv/bin/activate
maturin develop --uv
```

!!! warning "The studio must be built before the package"

    `bt.studio()` serves the bundle that `scripts/build_studio.sh` writes into
    `python/botrail/_studio/`. In a fresh checkout that directory does not
    exist yet and the launcher raises a `FileNotFoundError` telling you so.

See [Contributing](../contributing.md) for the development loop and the test
suites.

## Next

Load a robot and open the studio in the [Quickstart](quickstart.md).
