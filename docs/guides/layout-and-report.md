# The layout sheet and the cell report

Two more documents a cell hands over, both *derived* from the scene the way
the [I/O list](io-map.md) and the [BOM](parts-and-bom.md) are: the
**layout sheet** — the cell seen from above, as a proposal drawing or the
plant's 2D CAD wants it — and the **cell report** — one page that gathers
the numbers everything else measured, with the digest of every file
written from the same source.

```python
scene.export_layout("layout.svg", scale=200)   # for the review
scene.export_layout("layout.dxf")              # for the 2D CAD (mm)

report = scene.cell_report({"cycle": tl}, scenarios=runs,
                           deliverables=["layout.svg", "layout.dxf", "bom.csv", "io.csv"])
report.save("cell_report.md")
report.save("cell_report.json")
```

## The layout sheet

[`Scene.layout`][botrail.Scene.layout] projects the scene onto the floor and
renders it; [`Scene.export_layout`][botrail.Scene.export_layout] writes it,
picking the format from the extension:

| format | what it is |
|---|---|
| `svg` | a self-contained drawing (`scale` pixels per metre, default 100) — opens in a browser, drops into a slide |
| `dxf` | a minimal R12 file — `LINE` / `POLYLINE` / `CIRCLE` / `TEXT` on named layers, millimetres (`units="m"` for metres) — opens in any 2D CAD, no dependency on either side |
| `json` | the drawn items in world metres, for a front end that wants to draw them itself |

What is on it, layer by layer (the DXF layer names in capitals):

* **EQUIPMENT** — every visible obstacle as its footprint: the convex hull of a
  primitive, the bounding rectangle of a mesh.
* **GROUND** — floor slabs and painted markings, i.e. anything whose top is at
  or below `ground_z` (2 cm by default): drawn faint, and left out of the
  extents so a 20 m floor does not shrink the cell to a stamp.
* **ROBOT** / **REACH** — each robot's base as a mark, and, when the robot came
  from the [catalog](robots.md#the-model-catalog) with a `reach_mm`, its reach
  as a dashed circle.
* **DEVICE** — conveyor and sink zones (dashed, with the direction of travel),
  a linear axis's travel, a vehicle's route with its stations.
* **SENSOR** — zone boxes and beams.
* **FRAME** — named frames as crosses, labelled by their last segment.
* **LABEL** — names on the things worth naming: a [part](parts-and-bom.md)
  pinned to an obstacle or a group labels it (`Pedestal (PD-500)`); the rest
  label by their name-derived unit, so a USD subtree reads once as `Conveyor`
  and twelve fence panels once as `fence`. A ring-shaped group (a fence, a
  guard) is labelled above its top edge, a compact one at its centre.
* **DIM** / **GRID** — the overall width and depth, and a metre grid.

`frames=False`, `labels=False`, `reach=False`, `grid=None` switch the extras
off; `title=` names the sheet.

The extent the sheet measures is also a query of its own:
[`Scene.footprint`][botrail.Scene.footprint] returns `min`, `max`, `width`,
`depth`, `area` and `height` — the plan-view bounding rectangle of the
equipment, ground excluded — so `assert scene.footprint()["area"] <= 20`
is a one-liner.

What the sheet is *not*: a drawing with tolerances, a title-block standard,
or a projection of every prim's true outline. It is the plan a proposal
needs and a layout engineer can trace over — one kind of drawing, honestly
derived.

## The cell report

[`Scene.cell_report`][botrail.Scene.cell_report] gathers, in one
[`CellReport`][botrail.CellReport]:

| section | from | what |
|---|---|---|
| `robots` | the scene | name, DOF, base position, catalog identity and reach when known |
| `cycles` | the timelines you pass | duration, step spans, robot busy time and utilization, the branches taken, and the tightest clearance re-scanned against the scene each timeline was baked from (`clearance_dt`, `None` to skip) |
| `io` | the [I/O map](io-map.md) | point counts by kind and status, node usage, lint findings |
| `scenarios` | a `ScenarioRuns` | the matrix — which scenario completed at what cycle, which stalled and why |
| `machines` | the parts, devices, sensors and nodes | every machine tool (`machine_tool.*` part): its door — an axis the machine drives or a loose leaf — with drive, stroke and end-of-travel lanes, the panel's buttons, and the controller hosting its program |
| `bom` | the [BOM](parts-and-bom.md) | line count, unidentified count, quantity per category, numeric totals |
| `footprint` | the layout | the plan-view extent |
| `deliverables` | the paths you pass | size and SHA-256 of every file — the evidence that the drawing, the list and the program are one cell |

Pass the cycles as a `{name: timeline}` dict, a list, or a single
`SequenceTimeline`; a `ScenarioRuns` passed as `scenarios=` fills the matrix
and, when no timelines are given, supplies the cycles too. The matrix is
the FAT sheet's verdict column: a fault authored as a scenario (a stuck
switch, an open wire, the E-stop in) either lets the cycle through or
stalls it, and the row says which and at what step. Every section is
a plain dict or list — `report.cycles[0]["duration"]`,
`report.footprint["area"]`, `report.io["unbound"]` — and the report renders
the same data as Markdown (`to_markdown()` / `save("…md")`) or JSON
(`to_json()` / `save("…json")`), with every optional value present as
`null` so the JSON keeps one shape whether or not a bake was supplied.

The report *reads*; it does not judge. The numbers it shows are the ones a
regression test asserts on:

```python
report = build_cell().cell_report({"cycle": tl})
assert report.cycle_time("cycle") <= 8.0
assert report.min_clearance() > 0.05
assert report.io["unbound"] == 0
assert report.footprint["area"] <= 20.0
assert report.bom["unidentified"] == 0
```

## The document set as one thing

Because every deliverable is derived from the same scene, a set of them is
a *unit*: write them, hash them into the report, and a later run can say
which ones an edit touched — by name, not by guess. Moving a photo-eye
changes the layout sheet and the generated script and leaves the BOM and
the I/O list byte-identical; adding a fence panel changes the BOM too. The
repository's own tests pin exactly that
([`python/tests/test_deliverables.py`](https://github.com/botrail/botrail/blob/main/python/tests/test_deliverables.py)),
and the [Hand over the cell](../tutorials/hand-over.md) tutorial writes the
whole set from one script. The set has a control-design half too — the
[I/O list, the handshake spec, the interlock table](io-map.md#the-interlock-table)
and the [PLCopen file](offline-commissioning.md) — derived from the same
sequences the bake ran.
