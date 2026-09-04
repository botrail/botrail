# Scene

The cell: one or more robots, the obstacles around them, the frames they mount
on, the devices and sensors that give the environment behavior, the named
motions, and the sequences that drive it all.

Units are **meters**, and the world is **Z-up**. Orientations are quaternions in
`(x, y, z, w)` order. Wherever a method takes `robot=None`, it acts on the
scene's first robot.

A scene is also the live link to the studio: state changes made from Python are
pushed to connected browsers, and edits made in the browser are visible here.

```python
import botrail as bt

scene = bt.Scene(bt.Robot.from_urdf("arm.urdf"))
scene.add_box("table", size=(0.6, 0.6, 0.05), position=(0.4, 0.0, 0.0))
scene.load_usd("cell.usda", prefix="env")
scene.set_robot_base_pose(*scene.frame("env/World/mount"))
```

::: botrail.Scene

## IoPoint

Returned by [`Scene.io_points`][botrail.Scene.io_points]: one derived I/O
point of the cell (see [The I/O map](../../guides/io-map.md)).

::: botrail.IoPoint

## IoReport

Returned by [`Scene.io_report`][botrail.Scene.io_report]: the findings over
the derived I/O map, by severity.

::: botrail.IoReport

## IoFinding

One entry of an [`IoReport`][botrail.IoReport].

::: botrail.IoFinding

## IoMap

Returned by [`Scene.io_map`][botrail.Scene.io_map]: the assignment layer as
authored (nodes, bindings, declarations) — hand it to `to_script(io=...)`.

::: botrail.IoMap

## Bom

Returned by [`Scene.bom`][botrail.Scene.bom]: the bill of materials derived
from the scene's parts (see [Parts and the BOM](../../guides/parts-and-bom.md)).

::: botrail.Bom

## InterlockTable

Returned by [`Scene.interlocks`][botrail.Scene.interlocks]: every output a
step switches against the condition that admits the step (see
[The interlock table](../../guides/io-map.md#the-interlock-table)).

::: botrail.InterlockTable

## CellReport

Returned by [`Scene.cell_report`][botrail.Scene.cell_report]: cycles, I/O,
scenarios, machines, BOM totals, footprint and deliverable digests in one
page (see [Layout sheet and cell report](../../guides/layout-and-report.md)).

::: botrail.CellReport
