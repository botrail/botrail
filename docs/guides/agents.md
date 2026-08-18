# Working with agents and automation

botrail was built to be driven by code, and that makes it a good substrate
for an agent: the cell is text, every result is JSON, the bake is
deterministic, and nothing needs a GUI. This page is the loop an agent (or
a CI job, or you in a shell) runs, and where each piece lives.

## The loop

1. **Write the cell** — a Python file using the API, or a `.botrail` project
   written against [the JSON Schema](../reference/cli.md#the-project-schema).
2. **Check it** — `botrail check cell.py` loads it, lints it, counts what is
   in it, and prints JSON. Exit code 1 means an error-severity finding.
3. **Bake it** — `botrail simulate cell.py --scenarios --report r.json` runs
   the sequences (and the whole scenario matrix) and prints the
   [cell report](layout-and-report.md#the-cell-report): cycle times, step
   spans, clearance, I/O counts, scenario results, BOM totals, footprint.
4. **Read the numbers, change the cell, go to 2.** The bake is
   bit-identical for the same input, so a number that moved was moved by
   the edit — there is nothing to average, nothing to re-run.
5. **Hand it over** — `botrail export cell.py --out deliverables/ --all` writes
   the [document set](../tutorials/hand-over.md) with the report's digests.

```bash
botrail check cell.py                                    # {"ok": true, "counts": {...}, "findings": []}
botrail simulate cell.py --scenarios --report r.json     # the report as JSON on stdout too
botrail export cell.py --out deliverables/ --all         # project, python, bom, io, topology, layout, usd, script, report
```

The same loop from Python is the API these commands call —
`bt.Scene.load_project`, `scene.io_report()`, `scene.simulate_scenarios()`,
`scene.cell_report()`, the `export_*` methods — so an agent that prefers to
stay in Python loses nothing.

## What to read, in what order

For an agent learning the API, the shortest path through the docs is:

1. [Your first cell](../getting-started/first-cell.md) — scene, obstacles,
   sequence, bake, the vocabulary.
2. [Sequences](sequences.md) and [Sensors and devices](sensors-and-devices.md)
   — the process layer: steps, actions, transitions, what the environment
   does.
3. [Timeline assertions](timeline-assertions.md) — how a bake is read
   (`step_span`, `signal`, `min_clearance`) and asserted on.
4. [Parts and the BOM](parts-and-bom.md), [Standard parts](standard-parts.md),
   [The I/O map](io-map.md), [Layout sheet and cell report](layout-and-report.md)
   — the engineering documents and how each is derived.
5. The [API reference](../reference/api/scene.md) — every method's docstring;
   the same text `help(bt.Scene)` shows.

The repository's `examples/` are complete, runnable cells (the docs'
tutorials walk through them), and `python/tests/` shows what is asserted
about each feature — both are good few-shot material.

## Conventions that make the output machine-friendly

* **JSON everywhere.** `check`, `simulate` (without `--markdown`), `export`
  and `schema` print JSON; every report object has `to_json()`; the
  [cell report](layout-and-report.md#the-cell-report) keeps one shape
  (`null` for what was not measured, never a missing key); `Bom.rows`,
  `CellReport.cycles` and friends are plain dicts and lists.
* **Names, not indices.** Everything is addressed by the name you gave it —
  obstacles, frames, sensors, devices, sequences, steps, parts. Findings
  quote those names.
* **Errors are `ValueError` with a sentence**, and the CLI turns them into
  `{"ok": false, "error": "..."}` with exit code 2 (could not load) or 1
  (loaded, but findings or a failed bake).
* **Determinism.** Same input, same timeline, same report — a diff between
  two runs is a diff between two cells. `export` hashes what it writes so
  the diff can be taken at the file level too.
* **The project schema.** `botrail schema` (or `bt.project_schema()`) is
  generated from the Rust types the loader reads; a `.botrail` that validates
  is a `.botrail` that loads, and its descriptions are the docstrings.

## Studies

When the question is *which* layout rather than *this* layout, the same
loop runs over a grid: [`bt.sweep`][botrail.study.sweep] bakes a cell
authored as a function of its parameters at every point and returns a table
(rows in grid order, failed variants as rows with the reason);
[`bt.optimize`][botrail.study.optimize] searches the grid — exhaustively or
by coordinate descent — for the best feasible point under constraints on
the metrics. Both are deterministic and return every evaluated row, so an
agent can read the whole search, not just its answer
([Parameter sweeps](../tutorials/parameter-sweep.md)).

## What botrail does not do here

It does not design the cell for you. There is no layout generator and no
"design agent" inside the package — those live outside (a script, an agent,
a person) and use botrail as their hands and their verifier: author, check,
bake, read, repeat; `optimize` searches a space *you* wrote down. That is
deliberate: the package stays a deterministic engine with a text interface,
which is exactly what makes it composable with whatever does the thinking.
