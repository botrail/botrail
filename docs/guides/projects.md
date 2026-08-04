# Projects

A `.botrail` file is the whole cell in one artifact: robots, joint state,
obstacles, frames, motions, sequences, signals, sensors, and devices.

```python
scene.save_project("cell.botrail")
scene = bt.Scene.load_project("cell.botrail")

scene.simulate_sequence("cycle")     # sequences load ready to bake
```

## The format

Plain **JSON** when everything is self-contained — it diffs in git like any
other text. When mesh files are referenced, the project becomes a **zip
archive** (`project.json` + `assets/`) with the meshes bundled, so the file
stays portable across machines. You don't choose; the saver does.

Robots round-trip by the right mechanism for their source: URDF robots rebuild
from the embedded XML (the file is self-contained), USD robots re-import from
the referenced stage path (a 100 MB Isaac asset is not copied into every
save).

## Project or script?

Both capture the cell; they serve different moments.

| | `.botrail` project | `generate_python()` |
| --- | --- | --- |
| Nature | Data — load it back as-is | Code — read it, edit it, review it |
| Meshes | Bundled when needed | Referenced by path |
| Best for | Handing a cell to someone, save-and-resume | Turning studio work into a maintained source file |

A comfortable workflow is: build interactively, `save_project` as you go, and
when the cell settles, `generate_python()` once and make the script the source
of truth — with the bake numbers pinned by a
[regression test](../tutorials/verify-in-ci.md).

The studio's **Save** / **Load** buttons read and write the same `.botrail`
format, and **Export .py** is `generate_python` — the file formats and the UI
are the same feature.
