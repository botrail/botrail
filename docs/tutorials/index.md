# Tutorials

Each tutorial walks through one of the repository's example scripts — the same
files that ship in `examples/` and run in this project's CI. The code shown is
included from those files, and the printed numbers are real output.

Most of them use NVIDIA's official Isaac Sim Franka asset; the first run
downloads it (~10 MB) into the botrail cache, once. Two tutorials run on a
primitive-geometry arm straight from the checkout, with no downloads at all.

| Tutorial | What it teaches | Downloads |
| --- | --- | --- |
| [Pose and plan](pose-and-plan.md) | A USD robot in a USD cell, teaching grasps by IK, the studio | Franka |
| [Pick from a moving belt](sequence-cell.md) | Conveyor tracking, guarded ramps, grasping, a full PLC sequence | Franka |
| [Verify the cell in CI](verify-in-ci.md) | Turning a bake into a pytest regression suite | none |
| [Two arms, one belt](two-robots.md) | Multiple robots, zone interlocks, tick-checked arm-vs-arm collision | Franka |
| [Parameter sweeps](parameter-sweep.md) | The cell as a function of its layout; reading the numbers that move | none |
| [Export and replay USD](replay-usd.md) | Baking animations for usdview/Omniverse/Blender, and playing them back | Franka |

If you haven't yet, do the [Getting started](../getting-started/installation.md)
pages first — they introduce the vocabulary (scenes, motions, sequences,
timelines) these tutorials build on.
