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
--topology --plc --interlocks --layout --usd --script --report`, or `--all`
(the default when nothing is picked). Files are named after the cell
(`--name` overrides the stem): `<stem>.botrail`, `<stem>.py`,
`<stem>_bom.csv|.md`, `<stem>_io.csv`, `<stem>_topology.mmd`,
`<stem>.plcopen.xml`, `<stem>_interlocks.md|.csv` (the [interlock
table](../guides/io-map.md#the-interlock-table)), `<stem>_layout.svg|.dxf`
(`--scale` for the SVG),
`<stem>_<cycle>.usda` per baked cycle (`--fps`), `<stem>.script` (the robot
program — a warning on stderr when the dialect cannot take the cell, e.g. a
7-axis arm), and `<stem>_report.md|.json` last, with the digests of
everything written before it. Takes the same bake options as `simulate`.

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
