# The `botrail` command

Installed with the package: bake, check and export a cell without writing
Python. A *cell* argument is either a `.botrail` project or a Python file —
the file is run (not as `__main__`) and its top-level `scene` is taken, or
its `build()` / `build_cell()` / `build_scene()` is called and must return
a `Scene`. Everything prints JSON unless said otherwise. Exit codes: 0 ok,
1 findings with errors or a failed bake / stalled scenario, 2 the cell
could not be loaded or the arguments were wrong (`{"ok": false, "error":
"..."}` on stdout).

## `botrail check <cell>`

Loads the cell, derives the I/O map and lints it, walks every sequence,
lists unidentified BOM lines (with what the cell asks of them), compares
every line's [requirements](../guides/selection.md) with what its part says,
and counts what is in the scene — the same list as `scene.check()`:

```json
{
  "ok": true,
  "cell": "cell.py",
  "robots": ["simple_arm"],
  "counts": {"obstacles": 19, "frames": 0, "sensors": 1, "devices": 1,
             "sequences": 1, "scenarios": 2, "parts": 8, "bom_rows": 8},
  "findings": [{"severity": "info", "code": "unidentified_part", "target": "eye",
                "message": "eye (sensor.photoelectric) has no maker, model or catalog reference — needs sensing_range_mm >= 200"}],
  "requirements": {"lines": 8, "short": 0, "unknown": 2, "unidentified": 1}
}
```

`findings[].severity` is `error` / `warning` / `info` (the I/O lint codes,
`sequence` for a program that cannot be walked, `unidentified_part`,
`spec_short` when a part's stated spec falls short of what the cell asks,
`spec_unknown` when an identified part states no value, and
`requirement_incomplete` when a requirement could not be derived — a
grasped part with no `mass_kg`); exit 1 when any is an error.
`requirements` counts the BOM lines by the outcome of that comparison.

## `botrail review <cell>`

Lists design information gaps and available evidence using
[`bt.review`](../guides/design-review.md). `check().ok` and the `check` command
keep their existing meanings; the review reports `ready` for its stated scope.

| option | meaning |
|---|---|
| `--stage concept\|design` | required review groups (default concept) |
| `--require NAME` (repeatable) | additional required group or exact item ID |
| `--simulate` | bake the selected programs before reviewing |
| `--manifest PATH` | verify a batch export against the current cell and review its report; mutually exclusive with a new simulation |
| `--scenarios` | bake all scenarios; execution and expected-result acceptance remain separate |
| `--config PATH` | JSON object with `required`, `totals` and/or `annotations` |
| `--report PATH` / `--markdown` | save `.json`/`.md`, or print Markdown |

Also accepts the bake options below (`--sequence`, `--max-duration`,
`--clearance-dt`, `--no-clearance`, `--title`). Exit 0 means no review blockers,
1 means unresolved items or a failed bake, and 2 means invalid input.

## `botrail connections <cell>`

Checks declared [equipment interfaces and supply capacity](../guides/connections.md)
without baking. Prints JSON, or Markdown with `--markdown`. Use `--report PATH`
for `.json`/`.md`, `--csv PATH` for the connection requirements table and
`--power PATH` for the per-power-supply capacity CSV. Exit 0 means the declared
requirements are resolved, 1 means failures or incomplete information, and
2 means invalid input/output arguments. The table includes required but
unconnected ports and identifies missing consumption in each source's budget.

## `botrail simulate <cell>`

Bakes the sequences — all together, or the `--sequence NAME` set — and
prints the [cell report](../guides/layout-and-report.md#the-cell-report).

| option | meaning |
|---|---|
| `--sequence NAME` (repeatable) | programs to bake together (default: all) |
| `--scenarios` | bake the whole scenario matrix; the report gets the table and the cycles; exit 1 if any scenario stalled |
| `--max-duration S` | bake time limit (default 120) |
| `--clearance-dt S` / `--no-clearance` | clearance re-scan step, or skip it |
| `--report PATH` | also write the report (`.json` or `.md`) |
| `--usd PATH` (`--fps`) | also write the first baked cycle as USD |
| `--title` | report title |
| `--markdown` | print Markdown instead of JSON |

## `botrail export <cell> --out DIR`

Writes the document set — pick with `--project --python --bom --io
--topology --plc --interlocks --layout --usd --script --connections --report`, or `--all`
(the default when nothing is picked). Files are named after the cell
(`--name` overrides the stem): `<stem>.botrail`, `<stem>.py`,
`<stem>_bom.csv|.md`, `<stem>_io.csv`, `<stem>_topology.mmd`,
`<stem>.plcopen.xml`, `<stem>_interlocks.md|.csv` (the [interlock
table](../guides/io-map.md#the-interlock-table)), `<stem>_layout.svg|.dxf`
(`--scale` for the SVG),
`<stem>_<cycle>.usda` per baked cycle (`--fps`), `<stem>.script` for one
program or `<stem>_<program>.script` for each of several programs, and
`<stem>_report.md|.json`. Connection outputs are `<stem>_connections.csv|.md|.json`
and `<stem>_power.csv`. Every export also writes `<stem>_manifest.json`,
including hashes of both report formats. `--sequence` scopes every
program-dependent document as well as the bake.

`DIR` must be new or empty. Generation uses one isolated cell snapshot and
publishes the directory only after export and integrity checks finish.
Besides the `simulate` bake options, accepts `--dt` (scan interval, default
0.01 s), `--plan-resolution` (planner stride, default 0.05), and repeatable
`--attach PATH` for unverified external attachments. Bakes are kinematic.
The JSON response and report retain export warnings, PLCopen stub blocks,
omitted scripts and failed scenario executions in `issues`. Exit 0 means
the package was generated; it does not mean those issues are resolved.

## `botrail verify-export MANIFEST [--cell CELL]`

Verifies the [manifest and every file](../guides/layout-and-report.md#the-document-set-as-one-thing).
`--cell` also compares the current authored definition and observed geometry
asset hashes. Exit 0 requires intact generated files from a common revision;
1 means mismatch, missing/unlisted files, malformed manifest or unverified
external attachments. `ok` reports integrity; `same_revision` also excludes
external attachments. Exporter `issues` are returned separately and remain
subject to the design review. This command does not rerun simulation.

## `botrail schema [--out FILE]`

The JSON Schema (draft 2020-12) of the `.botrail` project file. See below.

## `botrail studio <cell> [--port N]`

Opens the cell in the studio.

## The project schema

`botrail schema` and `bt.project_schema()` return the schema of the
`.botrail` file, generated from the Rust types `Scene.load_project` reads —
so a project that validates is a project that loads, and its descriptions
are the types' doc comments. A copy ships with the docs:
[`project.schema.json`](../assets/project.schema.json).

```python
import json, jsonschema, botrail as bt
schema = json.loads(bt.project_schema())
jsonschema.Draft202012Validator(schema).validate(json.load(open("cell.botrail")))
```

The schema describes the *file*; the loader also checks what a schema
cannot (a binding onto a node the file does not have, a part pinned to a
missing obstacle), so `botrail check` is the definitive test.
