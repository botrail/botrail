# The layout sheet and the cell report

Two more documents a cell hands over, both *derived* from the scene the way
the [I/O list](io-map.md) and the [BOM](parts-and-bom.md) are: the
**layout sheet** — the cell seen from above, as a proposal drawing or the
plant's 2D CAD wants it — and the **cell report** — one page that gathers
the numbers everything else measured, with digests of supplied attachments.
For a document set with a recorded common input revision, use
[batch export and verification](#the-document-set-as-one-thing).

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
| `deliverables` | the paths you pass | size and SHA-256; `origin=external_attachment` because these calls do not establish the file's input revision |

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

Pass this report to [`bt.review(scene, report=report)`](design-review.md)
to list missing design inputs, known subtotals and unperformed evaluations
alongside these observations. A completed scenario is an execution result;
its expected-behaviour acceptance still needs the project's test conditions.

## The document set as one thing

Use `bt.export_cell` to generate the selected files from an independent
snapshot and fresh timelines. Its `sequences` scope applies to the bake,
I/O list, topology, interlocks, PLCopen, robot scripts and report I/O summary.
The saved project and generated Python retain the full authored cell;
this includes the [physical connection plan](connections.md). `exports=["connections"]`
writes the requirements CSV/Markdown/JSON and a per-power-supply capacity CSV.
The main batch report also contains these results. Physical requirements
cover the entire cell regardless of the selected operating programs;
the manifest records which programs were evaluated. Multiple programs
produce separate robot scripts. Unsupported scripts and lowering warnings
are recorded as unresolved issues, together with PLCopen stub blocks and
failed scenario executions.

```python
manifest = bt.export_cell(scene, "deliverables/rev1", name="cell",
                          sequences=["pick"], scenarios=True)
verified = bt.verify_export(manifest, scene=scene)
assert verified["same_revision"]
review = bt.review(scene, manifest=manifest, stage="design",
                   required=["deliverables"])
review.save("design_review.md")  # keep later review files outside the package
```

The output directory must be new or empty. Files are generated in a
temporary directory, verified, then moved into place. A failed export leaves
no partial package. Existing files can be included with `attachments=[...]`;
they are copied under `attachments/` and stay `external_attachment`.

`<name>_manifest.json` records:

| field | recorded evidence |
|---|---|
| `input` / `input_sha256` | serialized authored project, catalog IDs and recorded revisions, resolved local geometry paths with SHA-256 and size |
| `conditions` | ordered program and scenario sets, simulation scan interval and time limit, planner stride, kinematic mode, clearance sampling, USD rate, layout scale and controller export defaults |
| `generator` | botrail version, native extension hash, Python implementation hashes and manifest validator version |
| `run_sha256` | fingerprint of input hash, conditions and generator |
| `files` | relative path, format, origin, SHA-256, size; generated files also carry input/run fingerprints |
| `issues` | missing catalog revisions, omitted scripts, lowering warnings, PLCopen stubs and scenario execution failures |

The report includes provenance and exporter issues. The manifest hashes
both report formats as well as the other files; the report lists the files
generated before it. `verify_export` detects changed, missing and unlisted
files, inconsistent manifest metadata and paths escaping the package.
`scene=` also compares the current serialized authored input and observed
asset hashes. For example, replacing a freshly generated DI5 script with an
old DI2 script fails verification, even when the I/O CSV still matches.

`ok` means the integrity checks succeeded. `same_revision` additionally
requires generated files only; external attachments never count as verified
generated evidence. These flags do not establish design acceptance: a package
can have matching revisions and still contain stub implementations or failed
scenario executions. Requiring `deliverables` in `bt.review` keeps those
issues visible as unresolved work.

Geometry hashes identify the local inputs observed at export; assets are
checked again before publication. The snapshot retains loaded models and
colliders without reconstructing them through the project serializer. A
manifest is not a portable asset archive or a signed provenance certificate,
and verification does not rerun simulation. `.botrail` portability remains
subject to the existing project format's asset support. Catalog revisions
that were not recorded are reported as unknown, rather than looked up later.

::: botrail.export_cell

::: botrail.verify_export
