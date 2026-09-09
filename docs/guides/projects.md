# Projects

A `.botrail` file is the whole cell in one artifact: robots, joint state,
obstacles, frames, motions, sequences, signals, sensors, devices, scenarios,
the I/O map, and the [parts](parts-and-bom.md) pinned to all of them.

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

For Python replay without catalog access, load the portable project and embed
its captured catalog sources explicitly:

```python
scene = bt.Scene.load_project("cell.botrail")
code = scene.generate_python(embed_catalog=True)
```

This preserves catalog IDs, revisions, frames and declarations for the host,
adapters and tools. The default keeps pinned catalog downloads in ordinary
public-model scripts. Meshes and USD stages remain file references in either
script mode: keep the extracted files available. To move the cell to another
machine, transfer the `.botrail` file and generate the script after loading it.
